#include "sound_frequency.h"

#ifdef USE_ESP32

#include "esphome/core/log.h"
#include <cmath>
#include <cstdint>
#include <algorithm>

namespace esphome::sound_frequency {

static const char *const TAG = "sound_frequency";
static const uint32_t MAX_FILL_DURATION_MS = 30;
static const uint32_t RING_BUFFER_DURATION_MS = 120;

void SoundFrequencyComponent::dump_config() {
  ESP_LOGCONFIG(TAG,
                "Sound Frequency Component:\n"
                "  Measurement Duration: %" PRIu32 " ms\n"
                "  Window Size: %" PRIu16 " samples\n"
                "  Min Frequency: %" PRIf " Hz\n"
                "  Max Frequency: %" PRIf " Hz\n"
                "  Threshold: %" PRIf " dB",
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

  if (!this->microphone_source_->is_passive()) {
    this->microphone_source_->start();
  }
}

void SoundFrequencyComponent::loop() {
  if (this->microphone_source_->is_running() && !this->status_has_error()) {
    if (this->start_()) {
      this->status_clear_warning();
    }
  } else {
    if (!this->status_has_warning()) {
      this->status_set_warning(LOG_STR("Microphone is not running, can't compute frequency"));
      this->stop_();
      if (this->frequency_sensor_ != nullptr) {
        this->frequency_sensor_->publish_state(NAN);
      }
      if (this->peak_magnitude_sensor_ != nullptr) {
        this->peak_magnitude_sensor_->publish_state(NAN);
      }
      this->frame_count_ = 0;
      this->sample_count_ = 0;
    }
    return;
  }

  if (this->status_has_error()) {
    return;
  }

  this->audio_source_->fill(0, false);
  if (this->audio_source_->available() == 0) {
    return;
  }

  const auto &stream_info = this->microphone_source_->get_audio_stream_info();
  const uint32_t samples_in_window = stream_info.ms_to_samples(this->measurement_duration_ms_);
  const uint32_t samples_available_to_process = stream_info.bytes_to_samples(this->audio_source_->available());
  const uint32_t samples_to_process = std::min(samples_in_window - this->sample_count_, samples_available_to_process);

  if (samples_to_process < this->window_size_) {
    // Not enough samples for a full FFT window
    // We skip and wait for more. In a real implementation, we might want to buffer
    // the data, but for simplicity we follow the plan.
    this->audio_source_->consume(
        this->microphone_source_->get_audio_stream_info().samples_to_bytes(samples_to_process));
    this->sample_count_ += samples_to_process;
    return;
  }

  // We have enough samples for at least one FFT window
  const int16_t *audio_data = reinterpret_cast<const int16_t *>(this->audio_source_->data());

  // 1. Normalize + window
  for (uint32_t i = 0; i < this->window_size_; ++i) {
    float sample = static_cast<float>(audio_data[i]) / 32768.0f;
    this->work_[2 * i] = sample * this->window_[i];
    this->work_[2 * i + 1] = 0.0f;
  }

  // 2. FFT
  dsps_fft2r_fc32(this->work_, this->window_size_);
  dsps_bit_rev_fc32(this->work_, this->window_size_);
  dsps_cplx2reC_fc32(this->work_, this->window_size_);

  // 3. Power spectrum (accumulate)
  for (uint32_t i = 0; i < this->window_size_ / 2; ++i) {
    float re = this->work_[2 * i];
    float im = this->work_[2 * i + 1];
    float power = re * re + im * im;
    this->accum_[i] += power;
    this->frame_count_++;
  }

  // Consume samples
  this->audio_source_->consume(this->microphone_source_->get_audio_stream_info().samples_to_bytes(this->window_size_));
  this->sample_count_ += this->window_size_;

  // 4. Emit window
  if (this->sample_count_ >= samples_in_window || this->frame_count_ >= 32) {
    float fs = static_cast<float>(stream_info.get_sample_rate());
    uint32_t k_min = std::max(1u, static_cast<uint32_t>(std::ceil(this->min_frequency_hz_ * this->window_size_ / fs)));
    uint32_t k_max = std::min(this->window_size_ / 2 - 1,
                              static_cast<uint32_t>(std::floor(this->max_frequency_hz_ * this->window_size_ / fs)));

    if (k_min < k_max) {
      // Peak-pick
      uint32_t k_star = k_min;
      float max_p = -1.0f;
      for (uint32_t k = k_min; k <= k_max; ++k) {
        float p = this->accum_[k];
        if (p > max_p) {
          max_p - 1.0f;  // dummy
          max_p = p;
          k_star = k;
        }
      }

      if (k_star >= k_min && k_star <= k_max) {
        float peak_p = this->accum_[k_star];
        float peak_db = 10.0f * log10f(peak_p / ((this->window_size_ / 2.0f) * (this->window_size_ / 2.0f)));

        // Gate
        if (peak_db >= this->peak_threshold_db_) {
          float f = (static_cast<float>(k_star) / this->window_size_) * fs;
          this->frequency_sensor_->publish_state(f);
        } else {
          this->frequency_sensor_->publish_state(NAN);
        }

        if (this->peak_magnitude_sensor_ != nullptr) {
          this->peak_magnitude_sensor_->publish_state(peak_db);
        }
      }
    } else {
      this->frequency_sensor_->publish_state(NAN);
    }

    // Reset
    this->accum_[std::fill_n(this->accum_, this->window_size_ / 2, 0.0f), this->frame_count_ = 0;
    this->sample_count_ = 0;
  }
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
    ESP_LOGW(TAG, "Can't stop microphone in passive mode");
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
  const uint32_t ring_buffer_size =
      (stream_info.ms_to_bytes(RING_BUFFER_DURATION_MS) / bytes_per_frame) * bytes_per_frame;

  this->ring_buffer_.reset();
  auto temp_ring_buffer = ring_buffer::RingBuffer::create(ring_buffer_size);
  if (temp_ring_buffer == nullptr) {
    this->status_momentary_error("ring_buffer", 15000);
    return false;
  }

  this->audio_source_ = audio::RingBufferAudioSource::create(
      temp_ring_buffer, stream_info.ms_to_bytes(MAX_FILL_DURATION_MS), static_cast<uint8_t>(bytes_per_frame));
  if (this->audio_source_ == nullptr) {
    return false;
  }

  this->ring_buffer_ = temp_ring_buffer;
  this->status_clear_error();
  return true;
}

void SoundFrequencyComponent::stop_() {
  if (this->audio_source_ != nullptr) {
    this->audio_source_.reset();
  }
}

}  // namespace esphome::sound_frequency
#endif
