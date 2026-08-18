#include "sound_frequency.h"

#ifdef USE_ESP32

#include <sys/param.h>

#include "esp_dsp.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include <cmath>
#include <cstdint>
#include <cstring>

namespace esphome::sound_frequency {

static const char *const TAG = "sound_frequency";

static const uint32_t MAX_FILL_DURATION_MS = 30;
static const uint32_t RING_BUFFER_DURATION_MS = 120;
/// Cap on the number of FFT frames averaged into one measurement window (bounds CPU at high sample rates)
static const uint32_t MAX_FFT_FRAMES = 32;

void SoundFrequencyComponent::dump_config() {
  ESP_LOGCONFIG(TAG,
                "Sound Frequency Component:\n"
                "  Measurement Duration: %" PRIu32 " ms\n"
                "  Window Size: %" PRIu16 " samples\n"
                "  Min Frequency: %f Hz\n"
                "  Max Frequency: %f Hz\n"
                "  Peak Threshold: %f dB",
                this->measurement_duration_ms_, this->window_size_, this->min_frequency_hz_, this->max_frequency_hz_,
                this->peak_threshold_db_);
  LOG_SENSOR("  ", "Frequency:", this->frequency_sensor_);
  LOG_SENSOR("  ", "Peak Magnitude:", this->peak_magnitude_sensor_);
}

void SoundFrequencyComponent::setup() {
  this->microphone_source_->add_data_callback([this](const std::vector<uint8_t> &data) {
    auto temp_ring_buffer = this->ring_buffer_.lock();
    if (temp_ring_buffer != nullptr) {
      temp_ring_buffer->write((void *) data.data(), data.size());
    }
  });

  // One-time esp-dsp initialization: FFT coefficient table and Hann window.
  // The window size is a config constant, so all DSP buffers can be allocated here as well -
  // no heap allocation happens from loop() after this point.
  if (const auto init_res = dsps_fft2r_init_fc32(nullptr, this->window_size_); init_res != ESP_OK) {
    ESP_LOGE(TAG, "esp-dsp FFT table init failed (code %d)", init_res);
    return;
  }

  const uint32_t n = this->window_size_;
  // esp-dsp SIMD kernels require 16-byte aligned buffers
  void *aligned_work = nullptr;
  if (posix_memalign(&aligned_work, 16, 2 * n * sizeof(float)) != 0 || aligned_work == nullptr) {
    ESP_LOGE(TAG, "Failed to allocate FFT work buffer (%u bytes)", 2 * n * static_cast<uint32_t>(sizeof(float)));
    return;
  }

  float *window = static_cast<float *>(malloc(n * sizeof(float)));
  float *accum = static_cast<float *>(malloc((n / 2) * sizeof(float)));
  if (window == nullptr || accum == nullptr) {
    free(window);
    free(accum);
    free(aligned_work);
    ESP_LOGE(TAG, "Failed to allocate DSP buffers");
    return;
  }

  this->dsp_initialized_ = true;
  this->work_ = static_cast<float *>(aligned_work);
  this->window_ = window;
  this->accum_ = accum;

  dsps_wind_hann_f32(this->window_, n);
  memset(this->accum_, 0, (n / 2) * sizeof(float));

  if (!this->microphone_source_->is_passive()) {
    // Automatically start the microphone if not in passive mode
    this->microphone_source_->start();
  }
}

void SoundFrequencyComponent::loop() {
  if ((this->frequency_sensor_ == nullptr) && (this->peak_magnitude_sensor_ == nullptr)) {
    // No sensors configured, nothing to do
    return;
  }

  if (!this->dsp_initialized_ || this->window_ == nullptr || this->work_ == nullptr || this->accum_ == nullptr) {
    return;
  }

  if (this->microphone_source_->is_running() && !this->status_has_error()) {
    // Allocate buffers
    if (this->start_()) {
      this->status_clear_warning();
    } else {
      return;
    }
  } else {
    if (!this->status_has_warning()) {
      this->status_set_warning(LOG_STR("Microphone is not running, can't compute dominant frequency"));

      // Deallocate buffers, if necessary
      this->stop_();

      // Reset sensor outputs
      if (this->frequency_sensor_ != nullptr) {
        this->frequency_sensor_->publish_state(NAN);
      }
      if (this->peak_magnitude_sensor_ != nullptr) {
        this->peak_magnitude_sensor_->publish_state(NAN);
      }

      // Reset accumulators
      memset(this->accum_, 0, (this->window_size_ / 2) * sizeof(float));
      this->frame_count_ = 0;
      this->window_sample_count_ = 0;
    }

    return;
  }

  if (this->status_has_error()) {
    return;
  }

  const auto &stream_info = this->microphone_source_->get_audio_stream_info();

  // The sample rate is inherited from the microphone device and only known at runtime,
  // so the in-band bin range is recomputed here (once per loop until it becomes valid)
  if (!this->band_valid_) {
    const float fs = static_cast<float>(stream_info.get_sample_rate());
    this->sample_rate_hz_ = fs;

    const uint32_t n = this->window_size_;
    uint32_t k_min = static_cast<uint32_t>(std::ceil((this->min_frequency_hz_ * static_cast<double>(n)) / fs));
    uint32_t k_max = static_cast<uint32_t>((this->max_frequency_hz_ * static_cast<double>(n) / fs));
    if (k_min < 1) {
      k_min = 1;  // exclude DC bin
    }
    if (k_max > n / 2 - 1) {
      k_max = n / 2 - 1;  // exclude the Nyquist bin
    }

    this->k_min_ = k_min;
    this->k_max_ = k_max;
    this->band_valid_ = (k_min < k_max);
    if (!this->band_valid_) {
      ESP_LOGW(
          TAG,
          "Frequency band %.0f-%.0f Hz is outside the analyzable range for a sample rate of %u Hz and window size %u",
          this->min_frequency_hz_, this->max_frequency_hz_, stream_info.get_sample_rate(), n);
    }
  }

  // Expose a chunk of the ring buffer's internal storage - don't block to avoid slowing the main loop.
  // pre_shift is ignored by RingBufferAudioSource (no intermediate transfer buffer to compact).
  this->audio_source_->fill(0, false);

  if (this->audio_source_->available() < stream_info.samples_to_bytes(this->window_size_)) {
    // Not enough audio for a full FFT window yet - wait for more without consuming anything
    return;
  }

  const uint32_t samples_in_window = stream_info.ms_to_samples(this->measurement_duration_ms_);

  // Process exactly one N-sample frame per loop invocation (one FFT per call bounds the CPU load)
  if (!this->process_fft_frame_(reinterpret_cast<const int16_t *>(this->audio_source_->data()))) {
    return;
  }
  this->audio_source_->consume(stream_info.samples_to_bytes(this->window_size_));
  this->window_sample_count_ += this->window_size_;

  if (this->band_valid_) {
    // Emit the window when the measurement duration has been covered or the frame cap is reached
    if (this->window_sample_count_ >= samples_in_window || this->frame_count_ >= MAX_FFT_FRAMES) {
      this->emit_window_();
    }
  } else {
    // Band invalid: drain the window so the counters stay bounded, but publish nothing new
    memset(this->accum_, 0, (this->window_size_ / 2) * sizeof(float));
    this->frame_count_ = 0;
    this->window_sample_count_ = 0;
  }
}

bool SoundFrequencyComponent::process_fft_frame_(const int16_t *samples) {
  const uint32_t n = this->window_size_;

  // Normalize to [-1, 1) and apply the Hann window. Only one real signal is analyzed, so the
  // imaginary (second) input of the packed complex pair is zeroed.
  for (uint32_t i = 0; i < n; ++i) {
    const float sample = static_cast<float>(samples[i]) / 32768.0f;
    this->work_[2 * i] = sample * this->window_[i];
    this->work_[2 * i + 1] = 0.0f;
  }

  // Real FFT packed as a complex transform, then unpacked back to the real-signal spectrum
  dsps_fft2r_fc32(this->work_, n);
  dsps_bit_rev_fc32(this->work_, n);
  dsps_cplx2reC_fc32(this->work_, n);

  // Accumulate the power spectrum (periodogram averaging over the measurement window)
  for (uint32_t k = 0; k < n / 2; ++k) {
    const float re = this->work_[2 * k];
    const float im = this->work_[2 * k + 1];
    this->accum_[k] += re * re + im * im;
  }
  this->frame_count_++;

  return true;
}

void SoundFrequencyComponent::emit_window_() {
  const uint32_t n = this->window_size_;

  // Average the accumulated periodogram over the frames collected for this window
  if (this->frame_count_ > 0) {
    const float inv_frames = 1.0f / static_cast<float>(this->frame_count_);
    for (uint32_t k = 0; k < n / 2; ++k) {
      this->accum_[k] *= inv_frames;
    }
  }

  // Peak-pick the strongest in-band bin
  uint32_t k_star = 0;
  float peak_p = 0.0f;
  if (this->band_valid_) {
    k_star = this->k_min_;
    peak_p = this->accum_[k_star];
    for (uint32_t k = this->k_min_ + 1; k <= this->k_max_; ++k) {
      if (this->accum_[k] > peak_p) {
        peak_p = this->accum_[k];
        k_star = k;
      }
    }
  }

  // Approximate dBFS for a bin-centered full-scale sine (the Hann window halves the in-band energy)
  const float ref = static_cast<float>(n);
  const float peak_db = (peak_p > 0.0f) ? 10.0f * log10f(peak_p / ref) : -300.0f;

  // Publish the peak level unconditionally so automations can gate on loudness independent of frequency
  if (this->peak_magnitude_sensor_ != nullptr) {
    this->peak_magnitude_sensor_->publish_state(peak_db);
  }

  bool publish_frequency = false;
  float frequency_hz = NAN;
  if (this->band_valid_ && peak_db >= this->peak_threshold_db_) {
    // Sub-bin refinement with a parabolic fit on the log-magnitude around the peak
    float alpha = 0.0f;
    if (k_star > 0 && k_star + 1 < n / 2) {
      const float p_m1 = this->accum_[k_star - 1];
      const float p_p1 = this->accum_[k_star + 1];
      if (p_m1 > 0.0f && peak_p > 0.0f && p_p1 > 0.0f) {
        const float db_m1 = 0.5f * log10f(p_m1);
        const float db_0 = 0.5f * log10f(peak_p);
        const float db_p1 = 0.5f * log10f(p_p1);
        const float denom = db_m1 - 2.0f * db_0 + db_p1;
        if (denom != 0.0f) {
          alpha = 0.5f * (db_m1 - db_p1) / denom;
          if (alpha > 0.5f) {
            alpha = 0.5f;
          } else if (alpha < -0.5f) {
            alpha = -0.5f;
          }
        }
      }
    }

    frequency_hz = (static_cast<float>(k_star) + alpha) * this->sample_rate_hz_ / static_cast<float>(n);
    publish_frequency = true;
  }

  if (this->frequency_sensor_ != nullptr) {
    this->frequency_sensor_->publish_state(publish_frequency ? frequency_hz : NAN);
  }

  // Reset accumulators for the next measurement window
  memset(this->accum_, 0, (n / 2) * sizeof(float));
  this->frame_count_ = 0;
  this->window_sample_count_ = 0;
}

void SoundFrequencyComponent::start() {
  if (this->microphone_source_->is_passive()) {
    ESP_LOGW(TAG, "Can't start the microphone in passive mode");
    return;
  }
  this->microphone_source_->start();
}

void SoundFrequencyComponent::stop() {
  if (this->microphone_source_->is_passive()) {
    ESP_LOGW(TAG, "Can't stop the microphone in passive mode");
    return;
  }
  this->microphone_source_->stop();
}

bool SoundFrequencyComponent::start_() {
  if (this->audio_source_ != nullptr) {
    return true;
  }

  const auto &stream_info = this->microphone_source_->get_audio_stream_info();
  const size_t bytes_per_frame = stream_info.frames_to_bytes(1);

  // Allocate a ring buffer for the microphone callback to write into. Round the size down to a multiple
  // of bytes_per_frame so the wrap boundary stays frame-aligned and avoids unnecessary single-frame splices.
  this->ring_buffer_.reset();  // Reset pointer to any previous ring buffer allocation
  const size_t ring_buffer_size =
      (stream_info.ms_to_bytes(RING_BUFFER_DURATION_MS) / bytes_per_frame) * bytes_per_frame;
  std::shared_ptr<ring_buffer::RingBuffer> temp_ring_buffer = ring_buffer::RingBuffer::create(ring_buffer_size);
  if (temp_ring_buffer == nullptr) {
    this->status_momentary_error("ring_buffer", 15000);
    return false;
  }

  // Zero-copy source that reads directly from the ring buffer's internal storage. Frame-aligned reads
  // ensure multi-channel frames are never split across the ring buffer's wrap boundary.
  this->audio_source_ = audio::RingBufferAudioSource::create(
      temp_ring_buffer, stream_info.ms_to_bytes(MAX_FILL_DURATION_MS), static_cast<uint8_t>(bytes_per_frame));
  if (this->audio_source_ == nullptr) {
    this->status_momentary_error("audio_source", 15000);
    return false;
  }

  this->ring_buffer_ = temp_ring_buffer;
  this->status_clear_error();
  return true;
}

void SoundFrequencyComponent::stop_() { this->audio_source_.reset(); }

}  // namespace esphome::sound_frequency

#endif
