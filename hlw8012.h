#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/sensor/sensor.h"
//#include "esphome/components/pulse_counter/pulse_counter_sensor.h"
//#include "esphome/components/pulse_meter/pulse_meter_sensor.h"

#include <cinttypes>

namespace esphome {
namespace hlw8012 {
    
enum HLW8012InitialMode { HLW8012_INITIAL_MODE_CURRENT = 0, HLW8012_INITIAL_MODE_VOLTAGE };

enum HLW8012SensorModels {
  HLW8012_SENSOR_MODEL_HLW8012 = 0,
  HLW8012_SENSOR_MODEL_CSE7759,
  HLW8012_SENSOR_MODEL_BL0937
};


class HLW8012Component : public PollingComponent {
 public:

  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override;
  void update() override;

  void set_initial_mode(HLW8012InitialMode initial_mode) {
    current_mode_ = initial_mode != HLW8012_INITIAL_MODE_CURRENT;
  }
  void set_sensor_model(HLW8012SensorModels sensor_model) { sensor_model_ = sensor_model; }
  void set_change_mode_every(uint32_t change_mode_every) { change_mode_every_ = change_mode_every; }
  void set_current_resistor(float current_resistor) { current_resistor_ = current_resistor; }
  void set_voltage_divider(float voltage_divider) { voltage_divider_ = voltage_divider; }
  void set_sel_pin(GPIOPin *sel_pin) { sel_pin_ = sel_pin; }
  void set_cf_pin(InternalGPIOPin *cf_pin) { cf_pin_ = cf_pin; }
  void set_cf1_pin(InternalGPIOPin *cf1_pin) { cf1_pin_ = cf1_pin; }
  void set_voltage_sensor(sensor::Sensor *voltage_sensor) { voltage_sensor_ = voltage_sensor; }
  void set_current_sensor(sensor::Sensor *current_sensor) { current_sensor_ = current_sensor; }
  void set_power_sensor(sensor::Sensor *power_sensor) { power_sensor_ = power_sensor; }
  void set_energy_sensor(sensor::Sensor *energy_sensor) { energy_sensor_ = energy_sensor; energy_sensor_->publish_state(0);}

 protected:
  int32_t nth_value_{0};
  bool current_mode_{false};
  int32_t change_mode_at_{0};
  int32_t change_mode_every_{8};
  float current_resistor_{0.001};
  float voltage_divider_{2351};
  HLW8012SensorModels sensor_model_{HLW8012_SENSOR_MODEL_HLW8012};
  GPIOPin *sel_pin_{nullptr};

  InternalGPIOPin *cf1_pin_{nullptr};
  volatile uint32_t isr_cf1_counter_{0};
  uint32_t cf1_timer_{0};
  static void cf1_intr(HLW8012Component *sensor);

  InternalGPIOPin *cf_pin_{nullptr};
  volatile uint32_t isr_cf_counter_{0};
  uint32_t cf_timer_{0};
  static void cf_intr(HLW8012Component *sensor);
  
  float getHz(uint32_t count, uint32_t delta_time);

  sensor::Sensor *voltage_sensor_{nullptr};
  sensor::Sensor *current_sensor_{nullptr};
  sensor::Sensor *power_sensor_{nullptr};
  sensor::Sensor *energy_sensor_{nullptr};

  float voltage_multiplier_{0.0f};
  float current_multiplier_{0.0f};
  float power_multiplier_{0.0f};
};

}  // namespace hlw8012
}  // namespace esphome
