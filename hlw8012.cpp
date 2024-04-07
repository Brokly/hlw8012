#include "hlw8012.h"
#include "esphome/core/log.h"



namespace esphome {
namespace hlw8012 {

static const char *const TAG = "hlw8012";

// valid for HLW8012 and CSE7759
static const uint32_t HLW8012_CLOCK_FREQUENCY = 3579000;

void HLW8012Component::setup() {
  float reference_voltage = 0;
  ESP_LOGCONFIG(TAG, "Setting up HLW8012...");
  this->sel_pin_->setup();
  this->sel_pin_->digital_write(this->current_mode_);

  this->cf_pin_->setup();
  this->cf_timer_= esphome::micros();
  this->cf_pin_->attach_interrupt(HLW8012Component::cf_intr, this, gpio::INTERRUPT_RISING_EDGE);
  
  this->cf1_pin_->setup();
  this->cf1_timer_= esphome::micros();
  this->cf1_pin_->attach_interrupt(HLW8012Component::cf1_intr, this, gpio::INTERRUPT_RISING_EDGE);

  // Initialize multipliers
  if (this->sensor_model_ == HLW8012_SENSOR_MODEL_BL0937) {
    reference_voltage = 1.218f;
    this->power_multiplier_ =
        reference_voltage * reference_voltage * this->voltage_divider_ / this->current_resistor_ / 1721506.0f;
    this->current_multiplier_ = reference_voltage / this->current_resistor_ / 94638.0f;
    this->voltage_multiplier_ = reference_voltage * this->voltage_divider_ / 15397.0f;
  } else {
    // HLW8012 and CSE7759 have same reference specs
    reference_voltage = 2.43f;
    this->power_multiplier_ = reference_voltage * reference_voltage * this->voltage_divider_ / this->current_resistor_ *
                              64.0f / 24.0f / HLW8012_CLOCK_FREQUENCY;
    this->current_multiplier_ = reference_voltage / this->current_resistor_ * 512.0f / 24.0f / HLW8012_CLOCK_FREQUENCY;
    this->voltage_multiplier_ = reference_voltage * this->voltage_divider_ * 256.0f / HLW8012_CLOCK_FREQUENCY;
  }
}
void HLW8012Component::dump_config() {
  ESP_LOGCONFIG(TAG, "HLW8012:");
  LOG_PIN("  SEL Pin: ", this->sel_pin_)
  LOG_PIN("  CF Pin: ", this->cf_pin_)
  LOG_PIN("  CF1 Pin: ", this->cf1_pin_)
  ESP_LOGCONFIG(TAG, "  Change measurement mode every %" PRIu32, this->change_mode_every_);
  ESP_LOGCONFIG(TAG, "  Current resistor: %.1f mΩ", this->current_resistor_ * 1000.0f);
  ESP_LOGCONFIG(TAG, "  Voltage Divider: %.1f", this->voltage_divider_);
  LOG_UPDATE_INTERVAL(this)
  LOG_SENSOR("  ", "Voltage", this->voltage_sensor_)
  LOG_SENSOR("  ", "Current", this->current_sensor_)
  LOG_SENSOR("  ", "Power", this->power_sensor_)
  LOG_SENSOR("  ", "Energy", this->energy_sensor_)
}
float HLW8012Component::get_setup_priority() const { return setup_priority::DATA; }

float HLW8012Component::getHz(uint32_t* old_count, uint32_t* old_time, uint32_t now_count, uint32_t now){
   float delta_count=now_count-*old_count;
   *old_count=now_count;
   float delta_time=now-*old_time;
   *old_time=now;
   if(delta_time>0){
      return (delta_count*1000000)/delta_time;      
   }
   return 0.0;   
}

void HLW8012Component::update() {
  uint32_t raw_cf=this->isr_cf_counter_-this->cf_counter_;
  float cf_hz=getHz(&(this->cf_counter_), &(this->cf_timer_), this->isr_cf_counter_, this->isr_cf_time_);
  float cf1_hz=getHz(&(this->cf1_counter_), &(this->cf1_timer_), this->isr_cf1_counter_, this->isr_cf1_time_);

  if (this->nth_value_++ < 2) {
    return;
  }

  float power = cf_hz * this->power_multiplier_;

  if (this->change_mode_at_ != 0) {
    // Only read cf1 or cf after one cycle. Apparently it's quite unstable after being changed.
    if (this->current_mode_) {
      float voltage = cf1_hz * this->voltage_multiplier_;
      ESP_LOGD(TAG, "Got power=%.1fW, voltage=%.1fV", power, voltage);
      if (this->voltage_sensor_ != nullptr) {
        this->voltage_sensor_->publish_state(voltage);
      }
    } else {
      float current = cf1_hz * this->current_multiplier_;
      ESP_LOGD(TAG, "Got power=%.1fW, current=%.2fA", power, current);
      if (this->current_sensor_ != nullptr) {
        this->current_sensor_->publish_state(current);
      }
    }
  }

  if (this->power_sensor_ != nullptr) {
    this->power_sensor_->publish_state(power);
  }

  if (this->energy_sensor_ != nullptr) {
    cf_total_pulses_ += raw_cf;
    float energy = cf_total_pulses_ * this->power_multiplier_ / 3600;
    this->energy_sensor_->publish_state(energy);
  }

  if (this->change_mode_every_ != 0 && this->change_mode_at_++ == this->change_mode_every_) {
    this->current_mode_ = !this->current_mode_;
    ESP_LOGV(TAG, "Changing mode to %s mode", this->current_mode_ ? "VOLTAGE" : "CURRENT");
    this->change_mode_at_ = 0;
    this->sel_pin_->digital_write(this->current_mode_);
  }

}

void IRAM_ATTR HLW8012Component::cf_intr(HLW8012Component *sensor) {
   sensor->isr_cf_time_=esphome::micros();
   sensor->isr_cf_counter_++;
}

void IRAM_ATTR HLW8012Component::cf1_intr(HLW8012Component *sensor) {
   sensor->isr_cf1_time_=esphome::micros();
   sensor->isr_cf1_counter_++;
}

}  // namespace hlw8012
}  // namespace esphome
