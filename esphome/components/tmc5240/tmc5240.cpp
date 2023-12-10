#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

#include "tmc5240.h"

namespace esphome {
namespace tmc5240 {

static const char *TAG = "tmc5240";

TMC5240::TMC5240() {
  // Global list of driver to work around TMC channel index
  this->comp_index_ = tmc5240_global_component_index++;
  components[this->comp_index_] = this;

  // Initialize TMC-API object
  tmc_fillCRC8Table((uint8_t) 0b100000111, true, 0);
  tmc5240_init(&this->driver_, this->comp_index_, &this->config_, &tmc5240_defaultRegisterResetState[0]);
}

bool TMC5240::reset_() { return tmc5240_reset(&this->driver_); }
bool TMC5240::restore_() { return tmc5240_restore(&this->driver_); }
void TMC5240::update_registers_() { return tmc5240_periodicJob(&this->driver_, 0); }

void TMC5240::setup() {
  this->enn_pin_->setup();
  this->enn_pin_->digital_write(true);

  // Fill default configuration for driver (TODO: retry call instead)
  if (!tmc5240_reset(&this->driver_)) {
    this->mark_failed();
    return;
  }
  // Write initial configuration
  for (uint8_t i = 0; i < 255 && this->driver_.config->state != CONFIG_READY; i++) {
    this->update_registers_();
  }

  if (this->driver_.config->state != CONFIG_READY) {
    ESP_LOGE(TAG, "Failed to communicate with the driver");
    this->mark_failed();
    return;
  }

  this->set_vmax(this->max_speed_);
  this->set_amax(this->acceleration_);
  this->set_dmax(this->deceleration_);

  this->set_enc_const(50);
}

void TMC5240::loop() {
  this->update_registers_();
  // this->current_position = this->get_xactual();

  // Close encoder -> motor loop
  this->current_position = this->get_x_enc();
  this->set_xactual(this->current_position);

  if (!this->has_reached_target()) {
    this->enn_pin_->digital_write(false);
    this->set_vmax(this->max_speed_);
    this->set_amax(this->acceleration_);
    this->set_dmax(this->deceleration_);
    tmc5240_moveTo(&this->driver_, this->target_position, this->max_speed_);
  } else {
    this->enn_pin_->digital_write(true);
  }
}

/** TMC-API wrappers **/
extern "C" {

void tmc5240_writeDatagram(TMC5240TypeDef *tmc5240, uint8_t address, uint8_t x1, uint8_t x2, uint8_t x3, uint8_t x4) {
  TMC5240 *comp = components[tmc5240->config->channel];

  uint8_t data[5] = {(uint8_t) (address | TMC5240_WRITE_BIT), x1, x2, x3, x4};
  comp->read_write(data, 5);
  int32_t value = _8_32(x1, x2, x3, x4);

  // Write to the shadow register and mark the register dirty
  address = TMC_ADDRESS(address);
  tmc5240->config->shadowRegister[address] = value;
  tmc5240->registerAccess[address] |= TMC_ACCESS_DIRTY;
}

void tmc5240_writeInt(TMC5240TypeDef *tmc5240, uint8_t address, int32_t value) {
  tmc5240_writeDatagram(tmc5240, address, BYTE(value, 3), BYTE(value, 2), BYTE(value, 1), BYTE(value, 0));
}

int32_t tmc5240_readInt(TMC5240TypeDef *tmc5240, uint8_t address) {
  TMC5240 *comp = components[tmc5240->config->channel];

  address = TMC_ADDRESS(address);

  if (!TMC_IS_READABLE(tmc5240->registerAccess[address]))
    return tmc5240->config->shadowRegister[address];

  uint8_t data[5] = {address, 0, 0, 0, 0};
  comp->read_write(data, 5);
  comp->read_write(data, 5);

  return _8_32(data[1], data[2], data[3], data[4]);
}
}
/** TMC-API wrappers end **/

}  // namespace tmc5240
}  // namespace esphome
