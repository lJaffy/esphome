#pragma once

#ifdef USE_ESP32

#include <cstdint>
#include <vector>

#include "esphome/components/audio/audio_transfer_buffer.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/microphone/microphone_source.h"
#include "esphome/components/ring_buffer/ring_buffer.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"

namespace esphome::tone_sequence {

class ToneSequenceComponent : public Component {
 public:
  void dump_config() override;
  void setup() override;
  void loop() override;

  float get_setup_priority() const override { return setup_priority::AFTER_CONNECTION; }

  // ── Configuration setters (called from to_code) ──
  void set_microphone_source(microphone::MicrophoneSource *mic) { this->microphone_source_ = mic; }
  void set_window_size(uint16_t n) { this->window_size_ = n; }
  void set_tick_interval(uint32_t ms) { this->tick_interval_ms_ = ms; }
  void set_pattern(const std::vector<float> &tones) { this->pattern_tones_ = tones; }
  void set_pattern_duration(uint32_t ms) { this->pattern_duration_ms_ = ms; }
  void set_tolerance_hz(float hz) { this->tolerance_hz_ = hz; }
  void set_threshold_db(float db) { this->threshold_db_ = db; }
  void set_detected_sensor(binary_sensor::BinarySensor *sensor) { this->detected_sensor_ = sensor; }
  void set_release_time(uint32_t ms) { this->release_time_ms_ = ms; }
  uint32_t release_time_ms_{3000};  ///< how long the sensor stays True after detection

  /// @brief Starts the microphone if not already running
  void start();
  /// @brief Stops the microphone
  void stop();

 protected:
  bool start_();
  void stop_();

  /// Runs the Goertzel IIR for each expected tone over one N-sample frame,
  /// accumulating power into ``accum_``.
  void process_frame_(const int16_t *samples);

  /// Averages the accumulated power, picks the dominant tone, and feeds
  /// the pattern state machine.
  void emit_tick_();

  /// Advances or maintains the pattern-match state based on the current tick's
  /// dominant frequency and level.
  void evaluate_pattern_(float dominant_hz, float peak_db);

  /// Resets the pattern state machine and publishes False on the detected sensor.
  void reset_pattern_();

  // ── Configuration ──
  microphone::MicrophoneSource *microphone_source_{nullptr};
  binary_sensor::BinarySensor *detected_sensor_{nullptr};

  uint16_t window_size_{1024};          ///< N – samples per Goertzel frame
  uint32_t tick_interval_ms_{100};      ///< measurement window per tick
  std::vector<float> pattern_tones_;    ///< ordered list of expected frequencies (Hz)
  uint32_t pattern_duration_ms_{2000};  ///< total deadline for the full sequence
  float tolerance_hz_{50.0f};           ///< ± Hz around each expected tone
  float threshold_db_{-50.0f};          ///< minimum dBFS for a tone to count

  // ── Audio pipeline ──
  std::unique_ptr<audio::RingBufferAudioSource> audio_source_;
  std::weak_ptr<ring_buffer::RingBuffer> ring_buffer_;
  int16_t *frame_buf_{nullptr};
  uint32_t frame_buf_offset_{0};

  // ── Goertzel DSP state (allocated in setup) ──
  float *window_{nullptr};  ///< Hann window, N floats
  float *accum_{nullptr};   ///< power accumulator, num_tones floats
  float *g_v1_{nullptr};    ///< IIR state v1, num_tones floats
  float *g_v2_{nullptr};    ///< IIR state v2, num_tones floats
  float *g_c2_{nullptr};    ///< 2·cos(2πk/N) per tone, num_tones floats

  uint32_t num_tones_{0};       ///< pattern_tones_.size()
  float sample_rate_hz_{0.0f};  ///< runtime, from the microphone

  bool dsp_ready_{false};

  // ── Tick assembly ──
  uint32_t frame_count_{0};
  uint32_t tick_sample_count_{0};

  // ── Pattern state machine ──
  bool pattern_active_{false};
  uint8_t match_index_{0};        ///< which tone we are waiting for (0-based)
  uint32_t pattern_start_ms_{0};  ///< millis() when tone 0 first matched
  uint32_t release_until_ms_{0};  ///< millis() deadline for holding True
  bool detected_latched_{false};  ///< true while the sensor is in its hold period

  // ── Diagnostics ──
  uint32_t diag_log_ms_{0};
};

// ── Automation actions ──
template<typename... Ts> class StartAction : public Action<Ts...>, public Parented<ToneSequenceComponent> {
 public:
  void play(const Ts &...x) override { this->parent_->start(); }
};

template<typename... Ts> class StopAction : public Action<Ts...>, public Parented<ToneSequenceComponent> {
 public:
  void play(const Ts &...x) override { this->parent_->stop(); }
};

}  // namespace esphome::tone_sequence

#endif
