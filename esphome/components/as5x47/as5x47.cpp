#include "as5x47.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace as5x47 {

static const char *TAG = "as5x47";

void AS5X47Component::dump_config() {
  ESP_LOGCONFIG(TAG, "AS5X47:");
  LOG_PIN("  CS Pin: ", this->cs_);
  ESP_LOGCONFIG(TAG, "  SPI Mode: %d", this->mode_);

  ReadDataFrame readDataFrame;
  readDataFrame = this->read_register_(ERRFL_REG);
  Errfl errfl;
  errfl.raw = readDataFrame.values.data;
  ESP_LOGCONFIG(TAG, "  ERRFL Register:");
  ESP_LOGCONFIG(TAG, "   Reading Error: %d", readDataFrame.values.ef);
  ESP_LOGCONFIG(TAG, "   FRERR: %d", errfl.values.frerr);
  ESP_LOGCONFIG(TAG, "   INVCOMM: %d", errfl.values.invcomm);
  ESP_LOGCONFIG(TAG, "   PARERR: %d", errfl.values.parerr);

  readDataFrame = this->read_register_(PROG_REG);
  Prog prog;
  prog.raw = readDataFrame.values.data;
  ESP_LOGCONFIG(TAG, "  PROG Register: ");
  ESP_LOGCONFIG(TAG, "    Reading Error: %d", readDataFrame.values.ef);
  ESP_LOGCONFIG(TAG, "    PROGEN: %d", prog.values.progen);
  ESP_LOGCONFIG(TAG, "    OTPREF: %d", prog.values.otpref);
  ESP_LOGCONFIG(TAG, "    PROGOTP: %d", prog.values.progotp);
  ESP_LOGCONFIG(TAG, "    PROVER: %d", prog.values.progver);

  readDataFrame = this->read_register_(DIAGAGC_REG);
  Diaagc diaagc;
  diaagc.raw = readDataFrame.values.data;
  ESP_LOGCONFIG(TAG, "  DIAAGC Register: ");
  ESP_LOGCONFIG(TAG, "    Reading Error: %d", readDataFrame.values.ef);
  ESP_LOGCONFIG(TAG, "    AGC: %d", diaagc.values.agc);
  ESP_LOGCONFIG(TAG, "    LF: %d", diaagc.values.lf);
  ESP_LOGCONFIG(TAG, "    COF: %d", diaagc.values.cof);
  ESP_LOGCONFIG(TAG, "    MAGH: %d", diaagc.values.magh);
  ESP_LOGCONFIG(TAG, "    MAGL: %d", diaagc.values.magl);

  readDataFrame = this->read_register_(MAG_REG);
  Mag mag;
  mag.raw = readDataFrame.values.data;
  ESP_LOGCONFIG(TAG, "  MAG Register: ");
  ESP_LOGCONFIG(TAG, "    Reading Error: %d", readDataFrame.values.ef);
  ESP_LOGCONFIG(TAG, "    CMAG: %d", mag.values.cmag);

  readDataFrame = this->read_register_(ANGLE_REG);
  Angle angle;
  angle.raw = readDataFrame.values.data;
  ESP_LOGCONFIG(TAG, "  ANGLE Register: ");
  ESP_LOGCONFIG(TAG, "    Reading Error: %d", readDataFrame.values.ef);
  ESP_LOGCONFIG(TAG, "    CORDICANG: %d", angle.values.cordicang);

  readDataFrame = this->read_register_(ANGLECOM_REG);
  Anglecom anglecom;
  anglecom.raw = readDataFrame.values.data;
  ESP_LOGCONFIG(TAG, "  ANGLECOM Register: ");
  ESP_LOGCONFIG(TAG, "    Reading Error: %d", readDataFrame.values.ef);
  ESP_LOGCONFIG(TAG, "    DAECANG: %d", anglecom.values.daecang);

  readDataFrame = this->read_register_(ZPOSM_REG);
  Zposm zposm;
  zposm.raw = readDataFrame.values.data;
  ESP_LOGCONFIG(TAG, "  ZPOSM Register: ");
  ESP_LOGCONFIG(TAG, "    Reading Error: %d", readDataFrame.values.ef);
  ESP_LOGCONFIG(TAG, "    ZPOSM: %d", zposm.values.zposm);

  readDataFrame = this->read_register_(ZPOSL_REG);
  Zposl zposl;
  zposl.raw = readDataFrame.values.data;
  ESP_LOGCONFIG(TAG, "  ZPOSL Register: ");
  ESP_LOGCONFIG(TAG, "    Reading Error: %d", readDataFrame.values.ef);
  ESP_LOGCONFIG(TAG, "    ZPOSL: %d", zposl.values.zposl);
  ESP_LOGCONFIG(TAG, "    COMP_L_ERROR_EN: %d", zposl.values.compLerrorEn);
  ESP_LOGCONFIG(TAG, "    COMP_H_ERROR_EN: %d", zposl.values.compHerrorEn);

  readDataFrame = this->read_register_(SETTINGS1_REG);
  Settings1 settings1;
  settings1.raw = readDataFrame.values.data;
  ESP_LOGCONFIG(TAG, "  SETTINGS1 Register: ");
  ESP_LOGCONFIG(TAG, "    Reading Error:%d ", readDataFrame.values.ef);
  ESP_LOGCONFIG(TAG, "    NOISESET: %d", settings1.values.noiseset);
  ESP_LOGCONFIG(TAG, "    DIR: %d", settings1.values.dir);
  ESP_LOGCONFIG(TAG, "    UVW_ABI: %d", settings1.values.uvw_abi);
  ESP_LOGCONFIG(TAG, "    DAECDIS: %d", settings1.values.daecdis);
  ESP_LOGCONFIG(TAG, "    ABIBIN: %d", settings1.values.abibin);
  ESP_LOGCONFIG(TAG, "    DATASELECT: %d", settings1.values.dataselect);
  ESP_LOGCONFIG(TAG, "    PWMON: %d", settings1.values.pwmon);

  readDataFrame = this->read_register_(SETTINGS2_REG);
  Settings2 settings2;
  settings2.raw = readDataFrame.values.data;
  ESP_LOGCONFIG(TAG, "  SETTINGS2 Register: ");
  ESP_LOGCONFIG(TAG, "    Reading Error:%d ", readDataFrame.values.ef);
  ESP_LOGCONFIG(TAG, "    UVWPP: %d", settings2.values.uvwpp);
  ESP_LOGCONFIG(TAG, "    HYS: %d", settings2.values.hys);
  ESP_LOGCONFIG(TAG, "    ABIRES: %d", settings2.values.abires);
};

void AS5X47Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up device...");
  this->spi_setup();
  while (!this->spi_is_ready()) {
  }
};
void AS5X47Component::loop() { ESP_LOGD(TAG, "angle=%f", this->read_angle()); };

bool AS5X47Component::is_even_(uint16_t data) {
  int count = 0;
  unsigned int b = 1;
  for (unsigned int i = 0; i < 15; i++) {
    if (data & (b << i)) {
      count++;
    }
  }

  return (count % 2 != 0);
}

uint16_t AS5X47Component::transfer16(uint16_t data) {
  const uint8_t msb = data >> 8;
  const uint8_t lsb = (uint8_t) data;
  uint8_t buff[2] = {msb, lsb};

  this->enable();
  this->transfer_array(buff, 2);
  this->disable();

  return (uint16_t) ((buff[0] << 8) | buff[1]);
}

void AS5X47Component::write_data_(uint16_t command, uint16_t value) {
  this->transfer16(command);
  this->transfer16(value);
}

uint16_t AS5X47Component::read_data_(uint16_t command, uint16_t nop_command) {
  this->transfer16(command);
  return this->transfer16(nop_command);
}

void AS5X47Component::write_register_(uint16_t register_address, uint16_t register_value) {
  CommandFrame command;
  command.values.rw = WRITE;
  command.values.command_frame = register_address;
  command.values.parc = this->is_even_(command.raw);

  WriteDataFrame content_frame;
  content_frame.values.data = register_value;
  content_frame.values.low = 0;
  content_frame.values.pard = this->is_even_(content_frame.raw);
  this->write_data_(command.raw, content_frame.raw);
}

ReadDataFrame AS5X47Component::read_register_(uint16_t register_address) {
  CommandFrame command;
  command.values.rw = READ;
  command.values.command_frame = register_address;
  command.values.parc = this->is_even_(command.raw);

  CommandFrame nop_command;
  nop_command.values.rw = READ;
  nop_command.values.command_frame = NOP_REG;
  nop_command.values.parc = this->is_even_(nop_command.raw);

  ReadDataFrame receivedFrame;
  receivedFrame.raw = this->read_data_(command.raw, nop_command.raw);
  return receivedFrame;
}

float AS5X47Component::read_angle() {
  ReadDataFrame readDataFrame = this->read_register_(ANGLE_REG);
  Angle angle;
  angle.raw = readDataFrame.values.data;
  return angle.values.cordicang / 16384. * 360.;
}

}  // namespace as5x47
}  // namespace esphome
