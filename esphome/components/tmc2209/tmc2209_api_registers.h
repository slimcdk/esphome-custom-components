#pragma once

namespace esphome {
namespace tmc2209 {

/*******************************************************************************
* Copyright © 2019 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2024 Analog Devices Inc. All Rights Reserved.
* This software is proprietary to Analog Devices, Inc. and its licensors.
*******************************************************************************/

// constants

#define MOTORS           1
#define REGISTER_COUNT   128
#define TMC_WRITE_BIT    0x80
#define TMC_ADDRESS_MASK 0x7F
#define MAX_VELOCITY     (int32_t) 2147483647
#define MAX_ACCELERATION (uint32_t) 16777215uL
#define IC_VERSION_33    0x21

// ===== TMC2209 & 2202 & TMC2209 & 2220 & 2225 "Donkey Kong" family register set =====

#define GCONF         0x00
#define GSTAT         0x01
#define IFCNT         0x02
#define SLAVECONF     0x03
#define OTP_PROG      0x04
#define OTP_READ      0x05
#define IOIN          0x06
#define FACTORY_CONF  0x07

#define IHOLD_IRUN    0x10
#define TPOWERDOWN    0x11
#define TSTEP         0x12
#define TPWMTHRS      0x13
#define TCOOLTHRS     0x14

#define VACTUAL       0x22

#define SGTHRS        0x40
#define SG_RESULT     0x41
#define COOLCONF      0x42

#define MSCNT         0x6A
#define MSCURACT      0x6B
#define CHOPCONF      0x6C
#define DRV_STATUS    0x6F
#define PWMCONF       0x70
#define PWMSCALE      0x71
#define PWM_AUTO      0x72

// Register fields in TMC2209

#define I_SCALE_ANALOG_MASK          0x01 // GCONF // I_scale_analog  (Reset default=1)
#define I_SCALE_ANALOG_SHIFT         0 // min.: 0, max.: 1, default: 0
#define I_SCALE_ANALOG_FIELD         ((register_field) { I_SCALE_ANALOG_MASK, I_SCALE_ANALOG_SHIFT, GCONF, false })
#define INTERNAL_RSENSE_MASK         0x02 // GCONF // internal_Rsense (Reset default: OTP)
#define INTERNAL_RSENSE_SHIFT        1 // min.: 0, max.: 1, default: 0
#define INTERNAL_RSENSE_FIELD        ((register_field) { INTERNAL_RSENSE_MASK, INTERNAL_RSENSE_SHIFT, GCONF, false })
#define EN_SPREADCYCLE_MASK          0x04 // GCONF // en_spreadCycle (Reset default: OTP)
#define EN_SPREADCYCLE_SHIFT         2 // min.: 0, max.: 1, default: 0
#define EN_SPREADCYCLE_FIELD         ((register_field) { EN_SPREADCYCLE_MASK, EN_SPREADCYCLE_SHIFT, GCONF, false })
#define SHAFT_MASK                   0x08 // GCONF // controls motor direction
#define SHAFT_SHIFT                  3 // min.: 0, max.: 1, default: 0
#define SHAFT_FIELD                  ((register_field) { SHAFT_MASK, SHAFT_SHIFT, GCONF, false })
#define INDEX_OTPW_MASK              0x10 // GCONF // index_otpw
#define INDEX_OTPW_SHIFT             4 // min.: 0, max.: 1, default: 0
#define INDEX_OTPW_FIELD             ((register_field) { INDEX_OTPW_MASK, INDEX_OTPW_SHIFT, GCONF, false })
#define INDEX_STEP_MASK              0x20 // GCONF // index_step
#define INDEX_STEP_SHIFT             5 // min.: 0, max.: 1, default: 0
#define INDEX_STEP_FIELD             ((register_field) { INDEX_STEP_MASK, INDEX_STEP_SHIFT, GCONF, false })
#define PDN_DISABLE_MASK             0x40 // GCONF // pdn_disable
#define PDN_DISABLE_SHIFT            6 // min.: 0, max.: 1, default: 0
#define PDN_DISABLE_FIELD            ((register_field) { PDN_DISABLE_MASK, PDN_DISABLE_SHIFT, GCONF, false })
#define MSTEP_REG_SELECT_MASK        0x80 // GCONF // mstep_reg_select
#define MSTEP_REG_SELECT_SHIFT       7 // min.: 0, max.: 1, default: 0
#define MSTEP_REG_SELECT_FIELD       ((register_field) { MSTEP_REG_SELECT_MASK, MSTEP_REG_SELECT_SHIFT, GCONF, false })
#define MULTISTEP_FILT_MASK          0x0100 // GCONF // multistep_filt (Reset default=1)
#define MULTISTEP_FILT_SHIFT         8 // min.: 0, max.: 1, default: 0
#define MULTISTEP_FILT_FIELD         ((register_field) { MULTISTEP_FILT_MASK, MULTISTEP_FILT_SHIFT, GCONF, false })
#define TEST_MODE_MASK               0x0200 // GCONF // test_mode 0
#define TEST_MODE_SHIFT              9 // min.: 0, max.: 1, default: 0
#define TEST_MODE_FIELD              ((register_field) { TEST_MODE_MASK, TEST_MODE_SHIFT, GCONF, false })
#define RESET_MASK                   0x01 // GSTAT // reset
#define RESET_SHIFT                  0 // min.: 0, max.: 1, default: 0
#define RESET_FIELD                  ((register_field) { RESET_MASK, RESET_SHIFT, GSTAT, false })
#define DRV_ERR_MASK                 0x02 // GSTAT // drv_err
#define DRV_ERR_SHIFT                1 // min.: 0, max.: 1, default: 0
#define DRV_ERR_FIELD                ((register_field) { DRV_ERR_MASK, DRV_ERR_SHIFT, GSTAT, false })
#define UV_CP_MASK                   0x04 // GSTAT // uv_cp
#define UV_CP_SHIFT                  2 // min.: 0, max.: 1, default: 0
#define UV_CP_FIELD                  ((register_field) { UV_CP_MASK, UV_CP_SHIFT, GSTAT, false })
#define IFCNT_MASK                   0xFF // IFCNT // Interface  transmission  counter.  This  register  becomes incremented  with  each successful UART  interface write access.  Read  out  to  check  the  serial  transmission  for lost  data.  Read  accesses  do  not  change  the  content. The counter wraps around from 255 to 0.
#define IFCNT_SHIFT                  0 // min.: 0, max.: 255, default: 0
#define IFCNT_FIELD                  ((register_field) { IFCNT_MASK, IFCNT_SHIFT, IFCNT, false })
#define SLAVECONF_MASK               0x0F00 // SLAVECONF // SENDDELAY for read access (time until reply is sent): 0, 1:   8 bit times  2, 3:   3*8 bit times  4, 5:   5*8 bit times  6, 7:   7*8 bit times  8, 9:   9*8 bit times  10, 11:  11*8 bit times  12, 13:  13*8 bit times  14, 15:  15*8 bit times
#define SLAVECONF_SHIFT              8 // min.: 0, max.: 15, default: 0
#define SLAVECONF_FIELD              ((register_field) { SLAVECONF_MASK, SLAVECONF_SHIFT, SLAVECONF, false })
#define OTPBIT_MASK                  0x07 // OTP_PROG // Selection of OTP bit to be programmed to the selected byte location (n=0..7: programs bit n to a logic 1)
#define OTPBIT_SHIFT                 0 // min.: 0, max.: 7, default: 0
#define OTPBIT_FIELD                 ((register_field) { OTPBIT_MASK, OTPBIT_SHIFT, OTP_PROG, false })
#define OTPBYTE_MASK                 0x30 // OTP_PROG // Selection of OTP programming location (0, 1 or 2)
#define OTPBYTE_SHIFT                4 // min.: 0, max.: 3, default: 0
#define OTPBYTE_FIELD                ((register_field) { OTPBYTE_MASK, OTPBYTE_SHIFT, OTP_PROG, false })
#define OTPMAGIC_MASK                0xFF00 // OTP_PROG // Set  to  0xBD  to  enable  programming.  A  programming time of  minimum 10ms per bit is  recommended (check by reading OTP_READ).
#define OTPMAGIC_SHIFT               8 // min.: 0, max.: 255, default: 0
#define OTPMAGIC_FIELD               ((register_field) { OTPMAGIC_MASK, OTPMAGIC_SHIFT, OTP_PROG, false })
#define OTP0_BYTE_0_READ_DATA_MASK   0x01 // OTP_READ // to be detailed
#define OTP0_BYTE_0_READ_DATA_SHIFT  0 // min.: 0, max.: 255, default: 0
#define OTP0_BYTE_0_READ_DATA_FIELD  ((register_field) { OTP0_BYTE_0_READ_DATA_MASK, OTP0_BYTE_0_READ_DATA_SHIFT, OTP_READ, false })
#define OTP1_BYTE_1_READ_DATA_MASK   0x02 // OTP_READ // to be detailed
#define OTP1_BYTE_1_READ_DATA_SHIFT  8 // min.: 0, max.: 255, default: 0
#define OTP1_BYTE_1_READ_DATA_FIELD  ((register_field) { OTP1_BYTE_1_READ_DATA_MASK, OTP1_BYTE_1_READ_DATA_SHIFT, OTP_READ, false })
#define OTP2_BYTE_2_READ_DATA_MASK   0x04 // OTP_READ // to be detailed
#define OTP2_BYTE_2_READ_DATA_SHIFT  16 // min.: 0, max.: 255, default: 0
#define OTP2_BYTE_2_READ_DATA_FIELD  ((register_field) { OTP2_BYTE_2_READ_DATA_MASK, OTP2_BYTE_2_READ_DATA_SHIFT, OTP_READ, false })
#define ENN_MASK                     0x01 // IOIN //
#define ENN_SHIFT                    0 // min.: 0, max.: 1, default: 0
#define ENN_FIELD                    ((register_field) { ENN_MASK, ENN_SHIFT, IOIN, false })
#define MS1_MASK                     0x04 // IOIN //
#define MS1_SHIFT                    2 // min.: 0, max.: 1, default: 0
#define MS1_FIELD                    ((register_field) { MS1_MASK, MS1_SHIFT, IOIN, false })
#define MS2_MASK                     0x08 // IOIN //
#define MS2_SHIFT                    3 // min.: 0, max.: 1, default: 0
#define MS2_FIELD                    ((register_field) { MS2_MASK, MS2_SHIFT, IOIN, false })
#define DIAG_MASK                    0x10 // IOIN //
#define DIAG_SHIFT                   4 // min.: 0, max.: 1, default: 0
#define DIAG_FIELD                   ((register_field) { DIAG_MASK, DIAG_SHIFT, IOIN, false })
#define PDN_UART_MASK                0x40 // IOIN //
#define PDN_UART_SHIFT               6 // min.: 0, max.: 1, default: 0
#define PDN_UART_FIELD               ((register_field) { PDN_UART_MASK, PDN_UART_SHIFT, IOIN, false })
#define STEP_MASK                    0x80 // IOIN //
#define STEP_SHIFT                   7 // min.: 0, max.: 1, default: 0
#define STEP_FIELD                   ((register_field) { STEP_MASK, STEP_SHIFT, IOIN, false })
#define SEL_A_MASK                   0x0100 // IOIN // Driver type
#define SEL_A_SHIFT                  8 // min.: 0, max.: 1, default: 0
#define SEL_A_FIELD                  ((register_field) { SEL_A_MASK, SEL_A_SHIFT, IOIN, false })
#define DIR_MASK                     0x0200 // IOIN //
#define DIR_SHIFT                    9 // min.: 0, max.: 1, default: 0
#define DIR_FIELD                    ((register_field) { DIR_MASK, DIR_SHIFT, IOIN, false })
#define VERSION_MASK                 0xFF000000 // IOIN // VERSION: 0x20=first version of the IC Identical numbers mean full digital compatibility.
#define VERSION_SHIFT                24 // min.: 0, max.: 255, default: 0
#define VERSION_FIELD                ((register_field) { VERSION_MASK, VERSION_SHIFT, IOIN, false })
#define FCLKTRIM_MASK                0x1F // FACTORY_CONF // FCLKTRIM (Reset default: OTP)           0-31:  Lowest  to  highest  clock  frequency.  Check  at  charge  pump  output.  The  frequency  span  is  not  guaranteed,  but  it  is  tested,  that  tuning  to  12MHz  internal  clock  is  possible.  The  devices  come  preset  to  12MHz clock frequency by OTP programming.
#define FCLKTRIM_SHIFT               0 // min.: 0, max.: 31, default: 0
#define FCLKTRIM_FIELD               ((register_field) { FCLKTRIM_MASK, FCLKTRIM_SHIFT, FACTORY_CONF, false })
#define OTTRIM_MASK                  0x30 // FACTORY_CONF // OTTRIM (Default: OTP) %00:   OT=143°C, OTPW=120°C %01:  OT=150°C, OTPW=120°C %10:  OT=150°C, OTPW=143°C %11:  OT=157°C, OTPW=143°C
#define OTTRIM_SHIFT                 8 // min.: 0, max.: 3, default: 0
#define OTTRIM_FIELD                 ((register_field) { OTTRIM_MASK, OTTRIM_SHIFT, FACTORY_CONF, false })
#define IHOLD_MASK                   0x1F // IHOLD_IRUN // IHOLD (Reset default: OTP) Standstill current (0=1/32...31=32/32) In  combination  with  stealthChop  mode,  setting  IHOLD=0  allows  to  choose  freewheeling  or  coil  short circuit (passive braking) for motor stand still.
#define IHOLD_SHIFT                  0 // min.: 0, max.: 31, default: 0
#define IHOLD_FIELD                  ((register_field) { IHOLD_MASK, IHOLD_SHIFT, IHOLD_IRUN, false })
#define IRUN_MASK                    0x1F00 // IHOLD_IRUN // IRUN (Reset default=31) Motor run current (0=1/32...31=32/32) Hint:  Choose  sense  resistors  in  a  way,  that  normal  IRUN is 16 to 31 for best microstep performance.
#define IRUN_SHIFT                   8 // min.: 0, max.: 31, default: 0
#define IRUN_FIELD                   ((register_field) { IRUN_MASK, IRUN_SHIFT, IHOLD_IRUN, false })
#define IHOLDDELAY_MASK              0x0F0000 // IHOLD_IRUN // IHOLDDELAY (Reset default: OTP) Controls  the  number  of  clock  cycles  for  motor  power down after standstill is detected (stst=1) and  TPOWERDOWN  has  expired.  The  smooth  transition  avoids a motor jerk upon power down. 0:   instant power down 1..15:   Delay per current reduction step in multiple  of 2^18 clocks
#define IHOLDDELAY_SHIFT             16 // min.: 0, max.: 15, default: 0
#define IHOLDDELAY_FIELD             ((register_field) { IHOLDDELAY_MASK, IHOLDDELAY_SHIFT, IHOLD_IRUN, false })
#define TPOWERDOWN_MASK              0xFF // TPOWERDOWN // (Reset default=20) Sets  the  delay  time  from  stand  still  (stst)  detection  to  motor current power down. Time range is about 0 to 5.6 seconds.  0...((2^8)-1) * 2^18 tclk Attention:  A  minimum  setting  of  2  is  required  to  allow automatic tuning of stealthChop PWM_OFFS_AUTO.
#define TPOWERDOWN_SHIFT             0 // min.: 0, max.: 255, default: 0
#define TPOWERDOWN_FIELD             ((register_field) { TPOWERDOWN_MASK, TPOWERDOWN_SHIFT, TPOWERDOWN, false })
#define TSTEP_MASK                   0x0FFFFF // TSTEP // Actual  measured  time  between  two  1/256  microsteps  derived  from  the  step  input  frequency  in  units  of  1/fCLK.  Measured  value is (2^20)-1 in case of overflow or stand still.  The  TSTEP  related  threshold  uses  a  hysteresis  of  1/16  of  the  compare value to compensate for jitter in the clock or the step  frequency:  (Txxx*15/16)-1  is  the  lower  compare  value  for  each  TSTEP based comparison. This  means,  that  the  lower  switching  velocity  equals  the  calculated setting, but the upper switching velocity is higher as  defined by the hysteresis setting.
#define TSTEP_SHIFT                  0 // min.: 0, max.: 1048575, default: 0
#define TSTEP_FIELD                  ((register_field) { TSTEP_MASK, TSTEP_SHIFT, TSTEP, false })
#define TPWMTHRS_MASK                0x0FFFFF // TPWMTHRS // Sets the upper velocity for stealthChop voltage PWM mode.          For TSTEP = TPWMTHRS, stealthChop PWM mode is enabled, if configured. When  the  velocity  exceeds  the  limit  set  by  TPWMTHRS,  the  driver switches to spreadCycle. 0 = Disabled
#define TPWMTHRS_SHIFT               0 // min.: 0, max.: 1048575, default: 0
#define TPWMTHRS_FIELD               ((register_field) { TPWMTHRS_MASK, TPWMTHRS_SHIFT, TPWMTHRS, false })
#define VACTUAL_MASK                 0xFFFFFF // VACTUAL // VACTUAL allows moving the motor by UART control. It gives the motor velocity in +-(2^23)-1 [µsteps / t] 0: Normal operation. Driver reacts to STEP input. /=0:  Motor  moves  with  the  velocity  given  by  VACTUAL.  Step  pulses  can  be  monitored  via  INDEX  output.  The  motor  direction is controlled by the sign of VACTUAL.
#define VACTUAL_SHIFT                0 // min.: -8388608, max.: 8388607, default: 0
#define VACTUAL_FIELD                ((register_field) { VACTUAL_MASK, VACTUAL_SHIFT, VACTUAL, true })
#define SEMIN_MASK                   0x0000000F
#define SEMIN_SHIFT                  0
#define SEMIN_FIELD                  ((register_field) { SEMIN_MASK, SEMIN_SHIFT, COOLCONF, false })
#define SEUP_MASK                    0x00000060
#define SEUP_SHIFT                   5
#define SEUP_FIELD                   ((register_field) { SEUP_MASK, SEUP_SHIFT, COOLCONF, false })
#define SEMAX_MASK                   0x00000F00
#define SEMAX_SHIFT                  8
#define SEMAX_FIELD                  ((register_field) { SEMAX_MASK, SEMAX_SHIFT, COOLCONF, false })
#define SEDN_MASK                    0x00006000
#define SEDN_SHIFT                   13
#define SEDN_FIELD                   ((register_field) { SEDN_MASK, SEDN_SHIFT, COOLCONF, false })
#define SEIMIN_MASK                  0x00008000
#define SEIMIN_SHIFT                 15
#define SEIMIN_FIELD                 ((register_field) { SEIMIN_MASK, SEIMIN_SHIFT, COOLCONF, false })
#define MSCNT_MASK                   0x03FF // MSCNT // Microstep  counter.  Indicates  actual  position in the microstep table for  CUR_A.  CUR_B  uses an  offset  of  256  into  the  table.  Reading  out MSCNT  allows  determination  of  the  motor position within the electrical wave.
#define MSCNT_SHIFT                  0 // min.: 0, max.: 1023, default: 0
#define MSCNT_FIELD                  ((register_field) { MSCNT_MASK, MSCNT_SHIFT, MSCNT, false })
#define CUR_A_MASK                   0x01FF // MSCURACT // (signed) Actual  microstep current for motor phase  A  as  read  from  the internal  sine  wave  table  (not scaled by current setting)
#define CUR_A_SHIFT                  0 // min.: -255, max.: 255, default: 0
#define CUR_A_FIELD                  ((register_field) { CUR_A_MASK, CUR_A_SHIFT, MSCURACT, true })
#define CUR_B_MASK                   0x01FF0000 // MSCURACT // (signed) Actual  microstep current for motor phase  B  as  read  from  the internal  sine  wave  table  (not scaled by current setting)
#define CUR_B_SHIFT                  16 // min.: -255, max.: 255, default: 0
#define CUR_B_FIELD                  ((register_field) { CUR_B_MASK, CUR_B_SHIFT, MSCURACT, true })
#define TOFF_MASK                    0x0F // CHOPCONF // chopper off time and driver enable, Off time setting controls duration of slow decay phase (Nclk = 12 + 32*Toff),  %0000: Driver disable, all bridges off %0001: 1 - use only with TBL = 2 %0010 ... %1111: 2 - 15 (Default: OTP, resp. 3 in stealthChop mode)
#define TOFF_SHIFT                   0 // min.: 0, max.: 7, default: 0
#define TOFF_FIELD                   ((register_field) { TOFF_MASK, TOFF_SHIFT, CHOPCONF, false })
#define HSTRT_MASK                   0x70 // CHOPCONF // hysteresis start value added to HEND, %000 - %111: Add 1, 2, ..., 8 to hysteresis low value HEND (1/512 of this setting adds to current setting) Attention: Effective HEND+HSTRT <= 16. Hint: Hysteresis decrement is done each 16 clocks. (Default: OTP, resp. 0 in stealthChop mode)
#define HSTRT_SHIFT                  4 // min.: 0, max.: 7, default: 0
#define HSTRT_FIELD                  ((register_field) { HSTRT_MASK, HSTRT_SHIFT, CHOPCONF, false })
#define HEND_MASK                    0x0780 // CHOPCONF // hysteresis low value OFFSET sine wave offset, %0000 - %1111: Hysteresis is -3, -2, -1, 0, 1, ..., 12 (1/512 of this setting adds to current setting) This is the hysteresis value which becomes used for the hysteresis chopper. (Default: OTP, resp. 5 in stealthChop mode)
#define HEND_SHIFT                   7 // min.: 0, max.: 255, default: 0
#define HEND_FIELD                   ((register_field) { HEND_MASK, HEND_SHIFT, CHOPCONF, false })
#define TBL_MASK                     0x018000 // CHOPCONF // blank time select, %00 - %11: Set comparator blank time to 16, 24, 32 or 40 clocks Hint: %00 or %01 is recommended for most applications (Default: OTP)
#define TBL_SHIFT                    15 // min.: 0, max.: 255, default: 0
#define TBL_FIELD                   ((register_field) { TBL_MASK, TBL_SHIFT, CHOPCONF, false })
#define VSENSE_MASK                  0x020000 // CHOPCONF // sense resistor voltage based current scaling
#define VSENSE_SHIFT                 17 // min.: 0, max.: 1, default: 0
#define VSENSE_FIELD                 ((register_field) { VSENSE_MASK, VSENSE_SHIFT, CHOPCONF, false })
#define MRES_MASK                    0x0F000000 // CHOPCONF // MRES micro step resolution,          %0000: Native 256 microstep setting.          %0001 - %1000: 128, 64, 32, 16, 8, 4, 2, FULLSTEP: Reduced microstep resolution.  The  resolution  gives  the  number  of  microstep  entries  per sine quarter wave. When  choosing  a  lower  microstep  resolution,  the  driver automatically  uses  microstep  positions  which  result  in  a symmetrical wave. Number of microsteps per step pulse = 2^MRES (Selection  by  pins  unless  disabled  by  GCONF. mstep_reg_select)
#define MRES_SHIFT                   24 // min.: 0, max.: 255, default: 0
#define MRES_FIELD                   ((register_field) { MRES_MASK, MRES_SHIFT, CHOPCONF, false })
#define INTPOL_MASK                  0x10000000 // CHOPCONF // interpolation to 256 microsteps
#define INTPOL_SHIFT                 28 // min.: 0, max.: 1, default: 0
#define INTPOL_FIELD                 ((register_field) { INTPOL_MASK, INTPOL_SHIFT, CHOPCONF, false })
#define DEDGE_MASK                   0x20000000 // CHOPCONF // enable double edge step pulses
#define DEDGE_SHIFT                  29 // min.: 0, max.: 1, default: 0
#define DEDGE_FIELD                  ((register_field) { DEDGE_MASK, DEDGE_SHIFT, CHOPCONF, false })
#define DISS2G_MASK                  0x40000000 // CHOPCONF // short to GND protection disable
#define DISS2G_SHIFT                 30 // min.: 0, max.: 1, default: 0
#define DISS2G_FIELD                 ((register_field) { DISS2G_MASK, DISS2G_SHIFT, CHOPCONF, false })
#define DISS2VS_MASK                 0x80000000 // CHOPCONF // Low side short protection disable
#define DISS2VS_SHIFT                31 // min.: 0, max.: 1, default: 0
#define DISS2VS_FIELD                ((register_field) { DISS2VS_MASK, DISS2VS_SHIFT, CHOPCONF, false })
#define OTPW_MASK                    0x01 // DRV_STATUS // overtemperature prewarning flag
#define OTPW_SHIFT                   0 // min.: 0, max.: 1, default: 0
#define OTPW_FIELD                   ((register_field) { OTPW_MASK, OTPW_SHIFT, DRV_STATUS, false })
#define OT_MASK                      0x02 // DRV_STATUS // overtemperature flag
#define OT_SHIFT                     1 // min.: 0, max.: 1, default: 0
#define OT_FIELD                     ((register_field) { OT_MASK, OT_SHIFT, DRV_STATUS, false })
#define S2GA_MASK                    0x04 // DRV_STATUS // short to ground indicator phase A
#define S2GA_SHIFT                   2 // min.: 0, max.: 1, default: 0
#define S2GA_FIELD                   ((register_field) { S2GA_MASK, S2GA_SHIFT, DRV_STATUS, false })
#define S2GB_MASK                    0x08 // DRV_STATUS // short to ground indicator phase B
#define S2GB_SHIFT                   3 // min.: 0, max.: 1, default: 0
#define S2GB_FIELD                   ((register_field) { S2GB_MASK, S2GB_SHIFT, DRV_STATUS, false })
#define S2VSA_MASK                   0x10 // DRV_STATUS // low side short indicator phase A
#define S2VSA_SHIFT                  4 // min.: 0, max.: 1, default: 0
#define S2VSA_FIELD                  ((register_field) { S2VSA_MASK, S2VSA_SHIFT, DRV_STATUS, false })
#define S2VSB_MASK                   0x20 // DRV_STATUS // low side short indicator phase B
#define S2VSB_SHIFT                  5 // min.: 0, max.: 1, default: 0
#define S2VSB_FIELD                  ((register_field) { S2VSB_MASK, S2VSB_SHIFT, DRV_STATUS, false })
#define OLA_MASK                     0x40 // DRV_STATUS // open load indicator phase A
#define OLA_SHIFT                    6 // min.: 0, max.: 1, default: 0
#define OLA_FIELD                    ((register_field) { OLA_MASK, OLA_SHIFT, DRV_STATUS, false })
#define OLB_MASK                     0x80 // DRV_STATUS // open load indicator phase B
#define OLB_SHIFT                    7 // min.: 0, max.: 1, default: 0
#define OLB_FIELD                    ((register_field) { OLB_MASK, OLB_SHIFT, DRV_STATUS, false })
#define T120_MASK                    0x0100 // DRV_STATUS // 120°C comparator
#define T120_SHIFT                   8 // min.: 0, max.: 1, default: 0
#define T120_FIELD                   ((register_field) { T120_MASK, T120_SHIFT, DRV_STATUS, false })
#define T143_MASK                    0x0200 // DRV_STATUS // 143°C comparator
#define T143_SHIFT                   9 // min.: 0, max.: 1, default: 0
#define T143_FIELD                   ((register_field) { T143_MASK, T143_SHIFT, DRV_STATUS, false })
#define T150_MASK                    0x0400 // DRV_STATUS // 150°C comparator
#define T150_SHIFT                   10 // min.: 0, max.: 1, default: 0
#define T150_FIELD                   ((register_field) { T150_MASK, T150_SHIFT, DRV_STATUS, false })
#define T157_MASK                    0x0800 // DRV_STATUS // 157°C comparator
#define T157_SHIFT                   11 // min.: 0, max.: 1, default: 0
#define T157_FIELD                   ((register_field) { T157_MASK, T157_SHIFT, DRV_STATUS, false })
#define CS_ACTUAL_MASK               0x1F0000 // DRV_STATUS // actual motor current
#define CS_ACTUAL_SHIFT              16 // min.: 0, max.: 31, default: 0
#define CS_ACTUAL_FIELD              ((register_field) { CS_ACTUAL_MASK, CS_ACTUAL_SHIFT, DRV_STATUS, false })
#define STEALTH_MASK                 0x40000000 // DRV_STATUS // stealthChop indicator
#define STEALTH_SHIFT                30 // min.: 0, max.: 1, default: 0
#define STEALTH_FIELD                ((register_field) { STEALTH_MASK, STEALTH_SHIFT, DRV_STATUS, false })
#define STST_MASK                    0x80000000 // DRV_STATUS // standstill indicator
#define STST_SHIFT                   31 // min.: 0, max.: 1, default: 0
#define STST_FIELD                   ((register_field) { STST_MASK, STST_SHIFT, DRV_STATUS, false })
#define PWM_OFS_MASK                 0xFF // PWMCONF // User defined PWM amplitude offset (0-255) related to full motor current (CS_ACTUAL=31) in stand still. (Reset default=36) When  using  automatic  scaling  (pwm_autoscale=1)  the  value  is  used  for  initialization,  only.  The  autoscale  function  starts  with  PWM_SCALE_AUTO=PWM_OFS  and finds  the  required  offset  to  yield  the  target  current  automatically. PWM_OFS  =  0  will  disable  scaling  down  motor  current below  a  motor  specific  lower  measurement  threshold. This  setting  should  only  be  used  under  certain  conditions, i.e.  when the power supply voltage can vary  up  and  down  by  a  factor  of  two  or  more.  It  prevents the  motor  going  out  of  regulation,  but  it  also  prevents  power down below the regulation limit. PWM_OFS > 0 allows automatic scaling to low PWM duty  cycles  even  below  the  lower  regulation  threshold.  This  allows  low  (standstill)  current  settings  based  on  the  actual (hold) current scale (register IHOLD_IRUN).
#define PWM_OFS_SHIFT                0 // min.: 0, max.: 255, default: 0
#define PWM_OFS_FIELD                ((register_field) { PWM_OFS_MASK, PWM_OFS_SHIFT, PWMCONF, false })
#define PWM_GRAD_MASK                0xFF00 // PWMCONF // Velocity dependent gradient for PWM amplitude:  PWM_GRAD * 256 / TSTEP This  value  is  added  to  PWM_AMPL  to  compensate  for  the velocity-dependent motor back-EMF.  With  automatic  scaling  (pwm_autoscale=1)  the  value  is  used  for  first  initialization,  only.  Set  PWM_GRAD  to  the  application  specific  value  (it  can  be  read  out  from  PWM_GRAD_AUTO)  to  speed  up  the  automatic  tuning  process.  An  approximate  value can be stored to  OTP  by  programming OTP_PWM_GRAD.
#define PWM_GRAD_SHIFT               8 // min.: 0, max.: 255, default: 0
#define PWM_GRAD_FIELD               ((register_field) { PWM_GRAD_MASK, PWM_GRAD_SHIFT, PWMCONF, false })
#define PWM_FREQ_MASK                0x030000 // PWMCONF // %00:   fPWM=2/1024 fCLK          %01:   fPWM=2/683 fCLK          %10:   fPWM=2/512 fCLK          %11:   fPWM=2/410 fCLK
#define PWM_FREQ_SHIFT               16 // min.: 0, max.: 3, default: 0
#define PWM_FREQ_FIELD               ((register_field) { PWM_FREQ_MASK, PWM_FREQ_SHIFT, PWMCONF, false })
#define PWM_AUTOSCALE_MASK           0x040000 // PWMCONF //
#define PWM_AUTOSCALE_SHIFT          18 // min.: 0, max.: 1, default: 0
#define PWM_AUTOSCALE_FIELD          ((register_field) { PWM_AUTOSCALE_MASK, PWM_AUTOSCALE_SHIFT, PWMCONF, false })
#define PWM_AUTOGRAD_MASK            0x080000 // PWMCONF //
#define PWM_AUTOGRAD_SHIFT           19 // min.: 0, max.: 1, default: 0
#define PWM_AUTOGRAD_FIELD           ((register_field) { PWM_AUTOGRAD_MASK, PWM_AUTOGRAD_SHIFT, PWMCONF, false })
#define FREEWHEEL_MASK               0x300000 // PWMCONF // Stand still option when motor current setting is zero (I_HOLD=0).  %00:   Normal operation %01:   Freewheeling %10:   Coil shorted using LS drivers %11:   Coil shorted using HS drivers
#define FREEWHEEL_SHIFT              20 // min.: 0, max.: 3, default: 0
#define FREEWHEEL_FIELD              ((register_field) { FREEWHEEL_MASK, FREEWHEEL_SHIFT, PWMCONF, false })
#define PWM_REG_MASK                 0x0F000000 // PWMCONF // User defined  maximum  PWM amplitude  change per  half  wave when using pwm_autoscale=1. (1...15): 1: 0.5 increments (slowest regulation) 2: 1 increment (default with OTP2.1=1) 3: 1.5 increments 4: 2 increments ... 8: 4 increments (default with OTP2.1=0) ...  15: 7.5 increments (fastest regulation)
#define PWM_REG_SHIFT                24 // min.: 0, max.: 25, default: 0
#define PWM_REG_FIELD                ((register_field) { PWM_REG_MASK, PWM_REG_SHIFT, PWMCONF, false })
#define PWM_LIM_MASK                 0xF0000000 // PWMCONF // Limit  for  PWM_SCALE_AUTO  when  switching  back  from  spreadCycle to stealthChop. This value defines  the upper  limit  for  bits  7  to  4  of  the  automatic  current  control  when switching back. It can be set to reduce the current  jerk during mode change back to stealthChop. It does not limit PWM_GRAD or PWM_GRAD_AUTO offset. (Default = 12)
#define PWM_LIM_SHIFT                28 // min.: 0, max.: 15, default: 0
#define PWM_LIM_FIELD                ((register_field) { PWM_LIM_MASK, PWM_LIM_SHIFT, PWMCONF, false })
#define PWM_SCALE_SUM_MASK           0xFF // PWM_SCALE // Actual  PWM  duty  cycle.  This value  is  used  for  scaling  the values  CUR_A  and  CUR_B  read from the sine wave table.
#define PWM_SCALE_SUM_SHIFT          0 // min.: 0, max.: 255, default: 0
#define PWM_SCALE_SUM_FIELD          ((register_field) { PWM_SCALE_SUM_MASK, PWM_SCALE_SUM_SHIFT, PWM_SCALE, false })
#define PWM_SCALE_AUTO_MASK          0x01FF0000 // PWM_SCALE // 9 Bit signed offset added to the calculated  PWM  duty  cycle.  This is  the  result  of  the  automatic amplitude  regulation  based  on current measurement.
#define PWM_SCALE_AUTO_SHIFT         16 // min.: -255, max.: 255, default: 0
#define PWM_SCALE_AUTO_FIELD         ((register_field) { PWM_SCALE_AUTO_MASK, PWM_SCALE_AUTO_SHIFT, PWM_SCALE, true })
#define PWM_OFS_AUTO_MASK            0xFF // PWM_AUTO // Automatically  determined  offset value
#define PWM_OFS_AUTO_SHIFT           0 // min.: 0, max.: 255, default: 0
#define PWM_OFS_AUTO_FIELD           ((register_field) { PWM_OFS_AUTO_MASK, PWM_OFS_AUTO_SHIFT, PWM_AUTO, false })
#define PWM_GRAD_AUTO_MASK           0xFF0000 // PWM_AUTO // Automatically  determined gradient value
#define PWM_GRAD_AUTO_SHIFT          16 // min.: 0, max.: 255, default: 0
#define PWM_GRAD_AUTO_FIELD          ((register_field) { PWM_GRAD_AUTO_MASK, PWM_GRAD_AUTO_SHIFT, PWM_AUTO, false })

}  // namespace tmc2209
}  // namespace esphome
