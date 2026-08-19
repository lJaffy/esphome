#include "tone_sequence.h"

#ifdef USE_ESP32

#include <sys/param.h>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include <cmath>
#include <cstring>
#include <algorithm>

namespace esphome::tone_sequence {

static const char *const TAG = "tone_sequence";

static const uint32_t MAX_FILL_DURATION_MS = 30;
static const uint32_t RING_BUFFER_DURATION_MS = 120;
static const uint32_t MAX_FRAMES_PER_TICK = 16;
static const uint32_t DIAG_INTERVAL_MS = 5000;

// ──────────────────────────────────────────────
//  Configuration
// ──────────────────────────────────────────────

void ToneSequenceComponent::dump_config() {
  ESP_LOGCONFIG(TAG,
                "Tone Sequence Detector:\n"
                "  Window Size: %" PRIu16 " samples\n"
                "  Tick Interval: %" PRIu32 " ms\n"
                "  Pattern Duration: %" PRIu32 " ms\n"
                "  Tolerance: ±%.1f Hz\n"
                "  Threshold: %.1f dB\n"
                "  Tones (%lu):",
                this->window_size_, this->tick_interval_ms_, this->pattern_duration_ms_, this->tolerance_hz_,
                this->threshold_db_, (unsigned long) this->pattern_tones_.size());
  for (size_t i = 0; i < this->pattern_tones_.size(); ++i) {
    ESP_LOGCONFIG(TAG, "    [%u] = %.1f Hz", (unsigned) i, this->pattern_tones_[i]);
  }
  if (this->detected_sensor_ != nullptr) {
    LOG_BINARY_SENSOR("  ", "Detected:", this->detected_sensor_);
  }
}

// ──────────────────────────────────────────────
//  Setup – one-time allocation
// ──────────────────────────────────────────────

void ToneSequenceComponent::setup() {
  this->microphone_source_->add_data_callback([this](const std::vector<uint8_t> &data) {
    auto rb = this->ring_buffer_.lock();
    if (rb != nullptr) {
      rb->write((void *) data.data(), data.size());
    }
  });

  const uint32_t n = this->window_size_;
  this->num_tones_ = this->pattern_tones_.size();
  if (this->num_tones_ == 0) {
    ESP_LOGE(TAG, "No tones configured – nothing to do");
    return;
  }

  // Hann window (N floats)
  float *window = static_cast<float *>(malloc(n * sizeof(float)));
  if (window == nullptr) {
    ESP_LOGE(TAG, "Failed to allocate window buffer (%" PRIu32 " bytes)", n * sizeof(float));
    return;
  }
  for (uint32_t i = 0; i < n; ++i) {
    window[i] = 0.5f * (1.0f - cosf(2.0f * (float) M_PI * (float) i / (float) (n - 1)));
  }
  this->window_ = window;

  // Goertzel buffers (one slot per expected tone)
  const uint32_t nt = this->num_tones_;
  float *accum = static_cast<float *>(malloc(nt * sizeof(float)));
  float *v1 = static_cast<float *>(malloc(nt * sizeof(float)));
  float *v2 = static_cast<float *>(malloc(nt * sizeof(float)));
  float *c2 = static_cast<float *>(malloc(nt * sizeof(float)));
  if (accum == nullptr || v1 == nullptr || v2 == nullptr || c2 == nullptr) {
    free(accum);
    free(v1);
    free(v2);
    free(c2);
    free(window);
    ESP_LOGE(TAG, "Failed to allocate Goertzel buffers");
    return;
  }

  this->accum_ = accum;
  this->g_v1_ = v1;
  this->g_v2_ = v2;
  this->g_c2_ = c2;
  memset(accum, 0, nt * sizeof(float));

  // The Goertzel coefficients (2·cos(2πk/N)) depend on the sample rate,
  // which is only known at runtime. We store the target frequencies and
  // compute the coefficients on the first loop() call.
  this->sample_rate_hz_ = 0.0f;
  this->dsp_ready_ = true;

  if (!this->microphone_source_->is_passive()) {
    this->microphone_source_->start();
  }
}

// ──────────────────────────────────────────────
//  Main loop
// ──────────────────────────────────────────────

void ToneSequenceComponent::loop() {
  if (this->detected_sensor_ == nullptr) {
    return;
  }

  if (!this->dsp_ready_ || this->window_ == nullptr || this->accum_ == nullptr || this->g_v1_ == nullptr ||
      this->g_v2_ == nullptr || this->g_c2_ == nullptr) {
    return;
  }

  if (this->microphone_source_->is_running() && !this->status_has_error()) {
    if (this->start_()) {
      this->status_clear_warning();
    } else {
      ESP_LOGW(TAG, "Buffer allocation failed");
      return;
    }
  } else {
    if (!this->status_has_warning()) {
      this->status_set_warning(LOG_STR("Microphone is not running"));
    }
    this->stop_();
    if (this->detected_sensor_ != nullptr) {
      this->detected_sensor_->publish_state(false);
    }
    return;
  }

  if (this->status_has_error()) {
    return;
  }

  const auto &stream_info = this->microphone_source_->get_audio_stream_info();

  // First loop with a valid sample rate: compute Goertzel coefficients
  if (this->sample_rate_hz_ == 0.0f) {
    this->sample_rate_hz_ = static_cast<float>(stream_info.get_sample_rate());
    const uint32_t n = this->window_size_;
    for (uint32_t t = 0; t < this->num_tones_; ++t) {
      const float k = (this->pattern_tones_[t] * static_cast<float>(n)) / this->sample_rate_hz_;
      this->g_c2_[t] = 2.0f * cosf(2.0f * (float) M_PI * k / static_cast<float>(n));
    }
    ESP_LOGI(TAG, "Goertzel init: %lu tones, N=%" PRIu16 ", fs=%" PRIu32 " Hz", (unsigned long) this->num_tones_,
             this->window_size_, (uint32_t) stream_info.get_sample_rate());
  }

  const uint32_t samples_in_tick = stream_info.ms_to_samples(this->tick_interval_ms_);

  // ── Stage samples and run Goertzel on each complete frame ──
  while (this->frame_count_ < MAX_FRAMES_PER_TICK && this->tick_sample_count_ < samples_in_tick) {
    if (this->frame_buf_offset_ >= this->window_size_) {
      this->process_frame_(this->frame_buf_);
      this->frame_buf_offset_ = 0;
      this->frame_count_++;
      this->tick_sample_count_ += this->window_size_;
    }

    this->audio_source_->fill(0, false);
    const uint32_t available = stream_info.bytes_to_samples(this->audio_source_->available());
    if (available == 0)
      break;

    const int16_t *data = reinterpret_cast<const int16_t *>(this->audio_source_->mutable_data());
    const uint32_t need = this->window_size_ - this->frame_buf_offset_;
    const uint32_t take = std::min(available, need);
    std::memcpy(this->frame_buf_ + this->frame_buf_offset_, data, stream_info.samples_to_bytes(take));
    this->audio_source_->consume(stream_info.samples_to_bytes(take));
    this->frame_buf_offset_ += take;
  }

  // ── Emit when the tick window is full or frame cap is hit ──
  if (this->frame_count_ > 0 &&
      (this->tick_sample_count_ >= samples_in_tick || this->frame_count_ >= MAX_FRAMES_PER_TICK)) {
    this->emit_tick_();
  }

  // ── Release the latched detection after the hold period expires ──
  if (this->detected_latched_ && millis() >= this->release_until_ms_) {
    this->detected_latched_ = false;
    if (this->detected_sensor_ != nullptr) {
      this->detected_sensor_->publish_state(false);
    }
    ESP_LOGD(TAG, "Detection released after %lu ms hold", (unsigned long) this->release_time_ms_);
  }

  // ── Pattern deadline check (runs every loop, not just on tick) ──
  if (this->pattern_active_) {
    const uint32_t elapsed = millis() - this->pattern_start_ms_;
    if (elapsed > this->pattern_duration_ms_) {
      ESP_LOGD(TAG, "Pattern timed out after %lu ms (matched %u/%lu)", (unsigned long) elapsed, this->match_index_,
               (unsigned long) this->num_tones_);
      this->reset_pattern_();
    }
  }
}

// ──────────────────────────────────────────────
//  Goertzel frame processing
// ──────────────────────────────────────────────

void ToneSequenceComponent::process_frame_(const int16_t *samples) {
  const uint32_t n = this->window_size_;
  const uint32_t nt = this->num_tones_;

  // Reset IIR state
  for (uint32_t t = 0; t < nt; ++t) {
    this->g_v1_[t] = 0.0f;
    this->g_v2_[t] = 0.0f;
  }

  // IIR recursion: v[n] = c2·v[n-1] - v[n-2] + x[n]
  // Process sample-by-sample; all tones share the same input.
  for (uint32_t i = 0; i < n; ++i) {
    const float x = (static_cast<float>(samples[i]) / 32768.0f) * this->window_[i];
    for (uint32_t t = 0; t < nt; ++t) {
      const float v = this->g_c2_[t] * this->g_v1_[t] - this->g_v2_[t] + x;
      this->g_v2_[t] = this->g_v1_[t];
      this->g_v1_[t] = v;
    }
  }

  // Optimized magnitude-squared (no complex arithmetic, no atan2):
  //   |X[k]|² = v1² + v2² - c2·v1·v2
  // Accumulate power across frames (periodogram averaging within the tick).
  for (uint32_t t = 0; t < nt; ++t) {
    const float v1 = this->g_v1_[t];
    const float v2 = this->g_v2_[t];
    this->accum_[t] += v1 * v1 + v2 * v2 - this->g_c2_[t] * v1 * v2;
  }
}

// ──────────────────────────────────────────────
//  Tick emission & pattern evaluation
// ──────────────────────────────────────────────

void ToneSequenceComponent::emit_tick_() {
  const uint32_t n = this->window_size_;
  const uint32_t nt = this->num_tones_;

  // Average over the frames in this tick
  if (this->frame_count_ > 0) {
    const float inv = 1.0f / static_cast<float>(this->frame_count_);
    for (uint32_t t = 0; t < nt; ++t) {
      this->accum_[t] *= inv;
    }
  }

  // Find the strongest tone above threshold
  float peak_db = -300.0f;
  uint32_t peak_idx = 0;
  const float ref = static_cast<float>(n);

  for (uint32_t t = 0; t < nt; ++t) {
    const float p = this->accum_[t];
    const float db = (p > 0.0f) ? 10.0f * log10f(p / ref) : -300.0f;
    if (db > peak_db) {
      peak_db = db;
      peak_idx = t;
    }
  }

  // The dominant frequency is the expected tone at peak_idx (since each Goertzel
  // filter is tuned to exactly one pattern tone).
  float dominant_hz = this->pattern_tones_[peak_idx];

  // Feed the state machine
  this->evaluate_pattern_(dominant_hz, peak_db);

  // Reset for next tick
  memset(this->accum_, 0, nt * sizeof(float));
  this->frame_count_ = 0;
  this->tick_sample_count_ = 0;
}

void ToneSequenceComponent::evaluate_pattern_(float dominant_hz, float peak_db) {
  if (!this->pattern_active_) {
    // ── IDLE: waiting for the first tone in the sequence ──
    if (peak_db >= this->threshold_db_) {
      // Is the dominant tone close to pattern_tones_[0]?
      if (std::fabs(dominant_hz - this->pattern_tones_[0]) <= this->tolerance_hz_) {
        this->pattern_active_ = true;
        this->match_index_ = 1;
        this->pattern_start_ms_ = millis();
        ESP_LOGI(TAG, "Pattern started: tone 1/%lu matched (%.1f Hz, %.1f dB)", (unsigned long) this->num_tones_,
                 dominant_hz, peak_db);
      }
    }
    return;
  }

  // ── MATCHING ──
  if (peak_db < this->threshold_db_) {
    // Silence – simply wait, no penalty
    return;
  }

  // A tone is present. Check if it is the expected next one.
  const float expected = this->pattern_tones_[this->match_index_];
  if (std::fabs(dominant_hz - expected) <= this->tolerance_hz_) {
    // Correct tone – advance
    ESP_LOGD(TAG, "Tone %lu/%lu matched (%.1f Hz, %.1f dB)", (unsigned long) (this->match_index_ + 1),
             (unsigned long) this->num_tones_, dominant_hz, peak_db);
    this->match_index_++;

    if (this->match_index_ >= this->num_tones_) {
      const uint32_t elapsed = millis() - this->pattern_start_ms_;
      ESP_LOGI(TAG, "PATTERN DETETECTED in %lu ms (%lu tones)", (unsigned long) elapsed,
               (unsigned long) this->num_tones_);

      // Latch True and set a release deadline
      this->detected_latched_ = true;
      this->release_until_ms_ = millis() + this->release_time_ms_;

      if (this->detected_sensor_ != nullptr) {
        this->detected_sensor_->publish_state(true);
      }

      // Reset the matching state but do NOT publish false here
      this->pattern_active_ = false;
      this->match_index_ = 0;
      return;
    }
  }
  // Wrong tone: ignored – do not reset, do not advance.
}
void ToneSequenceComponent::reset_pattern_() {
  this->pattern_active_ = false;
  this->match_index_ = 0;
  // Only release the sensor if we're not currently in a hold period
  if (!this->detected_latched_ && this->detected_sensor_ != nullptr) {
    this->detected_sensor_->publish_state(false);
  }
}

// ──────────────────────────────────────────────
//  Public start/stop
// ──────────────────────────────────────────────

void ToneSequenceComponent::start() {
  if (this->microphone_source_->is_passive()) {
    ESP_LOGW(TAG, "Cannot start microphone in passive mode");
    return;
  }
  this->microphone_source_->start();
}

void ToneSequenceComponent::stop() {
  if (this->microphone_source_->is_passive()) {
    ESP_LOGW(TAG, "Cannot stop microphone in passive mode");
    return;
  }
  this->microphone_source_->stop();
}

// ──────────────────────────────────────────────
//  Internal buffer management
// ──────────────────────────────────────────────
bool ToneSequenceComponent::start_() {
  if (this->audio_source_ != nullptr) {
    return true;
  }

  const auto &stream_info = this->microphone_source_->get_audio_stream_info();
  const size_t bpf = stream_info.frames_to_bytes(1);

  this->ring_buffer_.reset();
  const size_t rb_size = (stream_info.ms_to_bytes(RING_BUFFER_DURATION_MS) / bpf) * bpf;
  std::shared_ptr<ring_buffer::RingBuffer> rb = ring_buffer::RingBuffer::create(rb_size);
  if (rb == nullptr) {
    this->status_momentary_error("ring_buffer", 15000);
    return false;
  }

  this->audio_source_ = audio::RingBufferAudioSource::create(rb, stream_info.ms_to_bytes(MAX_FILL_DURATION_MS),
                                                             static_cast<uint8_t>(bpf));
  if (this->audio_source_ == nullptr) {
    this->status_momentary_error("audio_source", 15000);
    return false;
  }
  this->ring_buffer_ = rb;  // shared_ptr → weak_ptr works fine

  this->frame_buf_ = static_cast<int16_t *>(malloc((this->window_size_ + 1) * sizeof(int16_t)));
  if (this->frame_buf_ == nullptr) {
    ESP_LOGE(TAG, "Failed to allocate frame buffer");
    this->audio_source_.reset();
    this->ring_buffer_.reset();
    this->status_momentary_error("frame_buf", 15000);
    return false;
  }

  this->status_clear_error();
  return true;
}

void ToneSequenceComponent::stop_() {
  this->audio_source_.reset();
  if (this->frame_buf_ != nullptr) {
    free(this->frame_buf_);
    this->frame_buf_ = nullptr;
  }
  this->frame_buf_offset_ = 0;
}

}  // namespace esphome::tone_sequence

#endif
