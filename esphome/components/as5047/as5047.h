#pragma once

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"

#include "esphome/components/spi/spi.h"

namespace esphome {
namespace as5047 {

#define AS5047P_ERRMSG_COUNT 140
#define AS5047P_ERRMSG_MAX_LEN 80

/* -------------------------------------------------------------------------------- */
/* -- CONSTANTS SECTION             	                                         -- */
/* -------------------------------------------------------------------------------- */
#define AS5047P_OPT_ENABLED true
#define AS5047P_OPT_DISABLED false

#define AS5047P_ACCESS_WRITE false
#define AS5047P_ACCESS_READ true

#define AS5047P_FRAME_PARD (1 << 15)
#define AS5047P_FRAME_EF (1 << 14)
#define AS5047P_FRAME_DATA 0x3FFF

#define AS5047P_ABIRES_100 100
#define AS5047P_ABIRES_200 200
#define AS5047P_ABIRES_400 400
#define AS5047P_ABIRES_800 800
#define AS5047P_ABIRES_1200 1200
#define AS5047P_ABIRES_1600 1600
#define AS5047P_ABIRES_2000 2000
#define AS5047P_ABIRES_4000 4000
#define AS5047P_ABIRES_1024 1024
#define AS5047P_ABIRES_2048 2048
#define AS5047P_ABIRES_4096 4096

// --- Volatile registers
#define AS5047P_NOP 0x0000
#define AS5047P_ERRFL 0x0001
#define AS5047P_PROG 0x0003
#define AS5047P_DIAAGC 0x3FFC
#define AS5047P_MAG 0x3FFD
#define AS5047P_ANGLEUNC 0x3FFE
#define AS5047P_ANGLECOM 0x3FFF

// --- Non-volatile registers
#define AS5047P_ZPOSM 0x0016
#define AS5047P_ZPOSL 0x0017
#define AS5047P_SETTINGS1 0x0018
#define AS5047P_SETTINGS2 0x0019

// --- Fields in registers
#define AS5047P_ERRFL_PARERR (1 << 2)
#define AS5047P_ERRFL_INVCOMM (1 << 1)
#define AS5047P_ERRFL_FRERR (1 << 0)
#define AS5047P_PROG_PROGVER (1 << 6)
#define AS5047P_PROG_PROGOTP (1 << 3)
#define AS5047P_PROG_OTPREF (1 << 2)
#define AS5047P_PROG_PROGEN (1 << 0)
#define AS5047P_DIAAGC_MAGL (1 << 11)
#define AS5047P_DIAAGC_MAGH (1 << 10)
#define AS5047P_DIAAGC_COF (1 << 9)
#define AS5047P_DIAAGC_LF (1 << 8)
#define AS5047P_DIAAGC_AGC (0x00FF << 0)
#define AS5047P_MAG_CMAG (0x3FFF << 0)
#define AS5047P_ANGLEUNC_CORDICANG (0x3FFF << 0)
#define AS5047P_ANGLECOM_DAECANG (0x3FFF << 0)
#define AS5047P_ZPOSM_ZPOSM (0x00FF << 0)
#define AS5047P_ZPOSL_COMP_H_ERR_EN (1 << 7)
#define AS5047P_ZPOSL_COMP_I_ERR_EN (1 << 6)
#define AS5047P_ZPOSL_ZPOSL (0x003F << 0)
#define AS5047P_SETTINGS1_BIT0 (1 << 0)
#define AS5047P_SETTINGS1_NOISESET (1 << 1)
#define AS5047P_SETTINGS1_DIR (1 << 2)
#define AS5047P_SETTINGS1_UVW_ABI (1 << 3)
#define AS5047P_SETTINGS1_DAECDIS (1 << 4)
#define AS5047P_SETTINGS1_ABIBIN (1 << 5)
#define AS5047P_SETTINGS1_DATASEL (1 << 6)
#define AS5047P_SETTINGS1_PWMON (1 << 7)
#define AS5047P_SETTINGS2_UVWPP (0x0007 << 0)
#define AS5047P_SETTINGS2_HYS (0x0003 << 3)
#define AS5047P_SETTINGS2_ABIRES (0x0007 << 5)

struct Error {
  int16_t errorCode;
  char *msg;
};

static char errMesage[AS5047P_ERRMSG_COUNT][AS5047P_ERRMSG_MAX_LEN] = {
    "No error pending",                                                            // 	[0]
    "Low level SPI access returned error",                                         // 	[1]
    "Framing error occurred in last Tx frame",                                     // 	[2]
    "Parity bit error occurred in Rx frame",                                       // 	[3]
    "",                                                                            // 	[4]
    "",                                                                            // 	[5]
    "",                                                                            // 	[6]
    "",                                                                            // 	[7]
    "",                                                                            // 	[8]
    "",                                                                            // 	[9]
    "Register with a given address doesn\'t exist",                                // 	[10]
    "Device not accessible on SPI line",                                           // 	[11]
    "",                                                                            // 	[12]
    "",                                                                            // 	[13]
    "",                                                                            // 	[14]
    "",                                                                            // 	[15]
    "",                                                                            // 	[16]
    "",                                                                            // 	[17]
    "",                                                                            // 	[18]
    "",                                                                            // 	[19]
    "Register with a given address doesn\'t have write access or doesn\'t exist",  // 	[20]
    "Register content verification failed after write operation",                  // 	[21]
    "Device not accessible on SPI line",                                           // 	[22]
    "",                                                                            // 	[23]
    "",                                                                            // 	[24]
    "",                                                                            // 	[25]
    "",                                                                            // 	[26]
    "",                                                                            // 	[27]
    "",                                                                            // 	[28]
    "",                                                                            // 	[29]
    "Low level hardware (SPI) init failed",                                        // 	[30]
    "Cant acknowledge the error in ERRFL register in init routine",                // 	[31]
    "",                                                                            // 	[32]
    "",                                                                            // 	[33]
    "",                                                                            // 	[34]
    "",                                                                            // 	[35]
    "",                                                                            // 	[36]
    "",                                                                            // 	[37]
    "",                                                                            // 	[38]
    "",                                                                            // 	[39]
    "Encoder not initialized while setting factory settings attempt",              // 	[40]
    "",                                                                            // 	[41]
    "",                                                                            // 	[42]
    "",                                                                            // 	[43]
    "",                                                                            // 	[44]
    "",                                                                            // 	[45]
    "",                                                                            // 	[46]
    "",                                                                            // 	[47]
    "",                                                                            // 	[48]
    "",                                                                            // 	[49]
    "Cant acknowledge the error in ERRFL register in reset routine",               // 	[50]
    "Encoder not initialized while reset attempt",                                 // 	[51]
    "",                                                                            // 	[52]
    "",                                                                            // 	[53]
    "",                                                                            // 	[54]
    "",                                                                            // 	[55]
    "",                                                                            // 	[56]
    "",                                                                            // 	[57]
    "",                                                                            // 	[58]
    "",                                                                            // 	[59]
    "Encoder not initialized while set field in register attempt",                 // 	[60]
    "",                                                                            // 	[61]
    "",                                                                            // 	[62]
    "",                                                                            // 	[63]
    "",                                                                            // 	[64]
    "",                                                                            // 	[65]
    "",                                                                            // 	[66]
    "",                                                                            // 	[67]
    "",                                                                            // 	[68]
    "",                                                                            // 	[69]
    "Device not accessible on SPI line",                                           // 	[70]
    "Magnetic field strength too low",                                             // 	[71]
    "Magnetic field strength too high",                                            // 	[72]
    "CORDIC overflow",                                                             // 	[73]
    "Magnet offset compensation error",                                            // 	[74]
    "Encoder not initialized while position read attempt",                         // 	[75]
    "Encoder not calibrated (zero pos not set) while position read attempt",       // 	[76]
    "",                                                                            // 	[77]
    "",                                                                            // 	[78]
    "",                                                                            // 	[79]
    "Encoder not initialized while zero position set attempt",                     // 	[80]
    "",                                                                            // 	[81]
    "",                                                                            // 	[82]
    "",                                                                            // 	[83]
    "",                                                                            // 	[84]
    "",                                                                            // 	[85]
    "",                                                                            // 	[86]
    "",                                                                            // 	[87]
    "",                                                                            // 	[88]
    "",                                                                            // 	[89]
    "Chosen ABI resolution is not correct",                                        // 	[90]
    "Encoder not initialized while ABI resolution set attempt",                    // 	[91]
    "",                                                                            // 	[92]
    "",                                                                            // 	[93]
    "",                                                                            // 	[94]
    "",                                                                            // 	[95]
    "",                                                                            // 	[96]
    "",                                                                            // 	[97]
    "",                                                                            // 	[98]
    "",                                                                            // 	[99]
    "Timeout fired while waiting for OTP burn finish",                             // 	[100]
    "Guardband test failed while OTP burn. Reprogramming not allowed",             // 	[101]
    "Encoder not initialized while OTP burn attempt",                              // 	[102]
    "",                                                                            // 	[103]
    "",                                                                            // 	[104]
    "",                                                                            // 	[105]
    "",                                                                            // 	[106]
    "",                                                                            // 	[107]
    "",                                                                            // 	[108]
    "",                                                                            // 	[109]
    "",                                                                            // 	[110]
    "",                                                                            // 	[111]
    "",                                                                            // 	[112]
    "",                                                                            // 	[113]
    "",                                                                            // 	[114]
    "",                                                                            // 	[115]
    "",                                                                            // 	[116]
    "",                                                                            // 	[117]
    "",                                                                            // 	[118]
    "",                                                                            // 	[119]
    "",                                                                            // 	[120]
    "",                                                                            // 	[121]
    "",                                                                            // 	[122]
    "",                                                                            // 	[123]
    "",                                                                            // 	[124]
    "",                                                                            // 	[125]
    "",                                                                            // 	[126]
    "",                                                                            // 	[127]
    "",                                                                            // 	[128]
    "",                                                                            // 	[129]
    "",                                                                            // 	[130]
    "",                                                                            // 	[131]
    "",                                                                            // 	[132]
    "",                                                                            // 	[133]
    "",                                                                            // 	[134]
    "",                                                                            // 	[135]
    "",                                                                            // 	[136]
    "",                                                                            // 	[137]
    "",                                                                            // 	[138]
    "",                                                                            // 	[139]
};

class AS5047Component : public Component,
                        public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_LOW,
                                              spi::CLOCK_PHASE_TRAILING, spi::DATA_RATE_10MHZ> {
 public:
  AS5047Component() = default;
  void dump_config() override;
  void setup() override;

  int16_t read_write_raw(int16_t frameTx, bool rw);
  int16_t read_register(uint16_t regAddr, bool devReachCheck);
  int16_t write_register(uint16_t regAddr, int16_t newRegContent, bool writeVerif, bool devReachCheck);
  int16_t set_factory_settings();
  int16_t error_ack();
  int16_t set_field_in_register(uint16_t regAddr, uint16_t fieldMask, uint16_t fieldVal);
  int16_t read_position(bool extendedDiag);
  int16_t set_zero_position();
  int16_t set_abi_resolution(uint16_t resolution);
  int16_t burn_otp(uint16_t yesImSure);
  bool error_pending();

 protected:
  uint8_t calc_parity_(uint32_t v);
  bool is_parity_ok_(uint16_t frameRx);
  void handle_error_(int16_t errCode);
  void clear_error_();

  bool zero_pos_calibrated_{false};
  bool initialized_{false};
  Error error_;
};

}  // namespace as5047
}  // namespace esphome
