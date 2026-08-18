#pragma once

#ifdef USE_ESP32

#include "esphome/components/audio/audio_transfer_buffer.h"
#include "esphome/components/microphone/microphone_source.h"
#include "esphome/components/ring_buffer/ring_buffer.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"

#ifdef USE_ESP32
#include "esp_dsp.h"
#endif

namespace esphome::sound_frequency {

class SoundFrequencyComponent : public Component {
 public:
  void dump_config() override;
  void setup() override;
  void loop() override;

  float get_setup_priority() const override { return setup_priority::AFTER_CONNECTION; }

  void set_measurement_duration(uint32_t measurement_duration_ms) {
    this->measurement_duration_ms_ = measurement_duration_ms;
  }
  void set_microphone_source(microphone::MicrophoneSource *microphone_source) {
    this->microphone_source_ = microphone_source;
  }
  void set_window_size(uint16_t window_size) { this->window_size_ = window_size; }
  void set_min_frequency_hz(float min_frequency_hz) { this->min_frequency_hz_ = min_frequency_hz; }
  void set_max_frequency_hz(float max_frequency_hz) { this->max_frequency_hz_ = max_frequency_hz; }
  void set_peak_threshold_db(float peak_threshold_db) { this->peak_threshold_db_ = peak_threshold_db; }
  void set_frequency_sensor(sensor::Sensor *frequency_sensor) { this->frequency_sensor_ = frequency_sensor; }
  void set_peak_magnitude_sensor(sensor::Sensor *peak_magnitude_sensor) {
    this->peak_magnitude_sensor_ = peak_magnitude_sensor;
  }

  void start();
  void stop();

 protected:
  bool start_();
  void stop_();

  microphone::MicrophoneSource *microphone_source_{nullptr};
  sensor::Sensor *frequency_sensor_{nullptr};
  sensor::Sensor *peak_magnitude_sensor_{nullptr};

  std::unique_ptr<audio::RingBufferAudioSource> audio_source_;
  std::weak_ptr<ring_buffer::RingBuffer> ring_buffer_;

  uint16_t window_size_{1024};
  float min_frequency_hz_{100.0f};
  float max_frequency_hz_{12000.0f};
  float peak_threshold_db_{-50.0f};

  uint32_t measurement_duration_ms_{1000};

  float *window_{nullptr};
  float *work_{nullptr};
  float *accum_{nullptr};
  uint32_t frame_count_{0};
  uint32_t sample_count_{0};
};

template<typename... Ts> class StartAction : public Action<Ts...>, public Parented<SoundFrequencyComponent> {
 public:
  void play(const Ts &...x) override { this->parent_->start(); }
};

template<typename... Ts> class StopAction : public Action<Ts...>, public Parented<SoundFrequencyComponent> {
 public:
  void play(const Ts &...x) override { this->parent_->stop(); }
};

}  // namespace esphome::sound_frequency

#endif
