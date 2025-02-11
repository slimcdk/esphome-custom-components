#pragma once

namespace esphome {
namespace tmc2208 {

// clang-format off

// Constants
#define MAX_VELOCITY     (int32_t)  2147483647
#define MAX_ACCELERATION (uint32_t) 4294967295uL


// Registers
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
#define VACTUAL       0x22
#define MSCNT         0x6A
#define MSCURACT      0x6B
#define CHOPCONF      0x6C
#define DRV_STATUS    0x6F
#define PWM_CONF      0x70
#define PWM_SCALE     0x71
#define PWM_AUTO      0x72


// Register fields
#define I_SCALE_ANALOG_MASK          0x00000001
#define I_SCALE_ANALOG_SHIFT         0
#define I_SCALE_ANALOG_FIELD         ((RegisterField) {I_SCALE_ANALOG_MASK, I_SCALE_ANALOG_SHIFT, GCONF, false})
#define INTERNAL_RSENSE_MASK         0x00000002
#define INTERNAL_RSENSE_SHIFT        1
#define INTERNAL_RSENSE_FIELD        ((RegisterField) {INTERNAL_RSENSE_MASK, INTERNAL_RSENSE_SHIFT, GCONF, false})
#define EN_SPREADCYCLE_MASK          0x00000004
#define EN_SPREADCYCLE_SHIFT         2
#define EN_SPREADCYCLE_FIELD         ((RegisterField) {EN_SPREADCYCLE_MASK, EN_SPREADCYCLE_SHIFT, GCONF, false})
#define SHAFT_MASK                   0x00000008
#define SHAFT_SHIFT                  3
#define SHAFT_FIELD                  ((RegisterField) {SHAFT_MASK, SHAFT_SHIFT, GCONF, false})
#define INDEX_OTPW_MASK              0x00000010
#define INDEX_OTPW_SHIFT             4
#define INDEX_OTPW_FIELD             ((RegisterField) {INDEX_OTPW_MASK, INDEX_OTPW_SHIFT, GCONF, false})
#define INDEX_STEP_MASK              0x00000020
#define INDEX_STEP_SHIFT             5
#define INDEX_STEP_FIELD             ((RegisterField) {INDEX_STEP_MASK, INDEX_STEP_SHIFT, GCONF, false})
#define PDN_DISABLE_MASK             0x00000040
#define PDN_DISABLE_SHIFT            6
#define PDN_DISABLE_FIELD            ((RegisterField) {PDN_DISABLE_MASK, PDN_DISABLE_SHIFT, GCONF, false})
#define MSTEP_REG_SELECT_MASK        0x00000080
#define MSTEP_REG_SELECT_SHIFT       7
#define MSTEP_REG_SELECT_FIELD       ((RegisterField) {MSTEP_REG_SELECT_MASK, MSTEP_REG_SELECT_SHIFT, GCONF, false})
#define MULTISTEP_FILT_MASK          0x00000100
#define MULTISTEP_FILT_SHIFT         8
#define MULTISTEP_FILT_FIELD         ((RegisterField) {MULTISTEP_FILT_MASK, MULTISTEP_FILT_SHIFT, GCONF, false})
#define TEST_MODE_MASK               0x00000200
#define TEST_MODE_SHIFT              9
#define TEST_MODE_FIELD              ((RegisterField) {TEST_MODE_MASK, TEST_MODE_SHIFT, GCONF, false})
#define RESET_MASK                   0x00000001
#define RESET_SHIFT                  0
#define RESET_FIELD                  ((RegisterField) {RESET_MASK, RESET_SHIFT, GSTAT, false})
#define DRV_ERR_MASK                 0x00000002
#define DRV_ERR_SHIFT                1
#define DRV_ERR_FIELD                ((RegisterField) {DRV_ERR_MASK, DRV_ERR_SHIFT, GSTAT, false})
#define UV_CP_MASK                   0x00000004
#define UV_CP_SHIFT                  2
#define UV_CP_FIELD                  ((RegisterField) {UV_CP_MASK, UV_CP_SHIFT, GSTAT, false})
#define IFCNT_MASK                   0x000000FF
#define IFCNT_SHIFT                  0
#define IFCNT_FIELD                  ((RegisterField) {IFCNT_MASK, IFCNT_SHIFT, IFCNT, false})
#define SLAVECONF_MASK               0x00000F00
#define SLAVECONF_SHIFT              8
#define SLAVECONF_FIELD              ((RegisterField) {SLAVECONF_MASK, SLAVECONF_SHIFT, SLAVECONF, false})
#define OTPBIT_MASK                  0x00000007
#define OTPBIT_SHIFT                 0
#define OTPBIT_FIELD                 ((RegisterField) {OTPBIT_MASK, OTPBIT_SHIFT, OTP_PROG, false})
#define OTPBYTE_MASK                 0x00000030
#define OTPBYTE_SHIFT                4
#define OTPBYTE_FIELD                ((RegisterField) {OTPBYTE_MASK, OTPBYTE_SHIFT, OTP_PROG, false})
#define OTPMAGIC_MASK                0x0000FF00
#define OTPMAGIC_SHIFT               8
#define OTPMAGIC_FIELD               ((RegisterField) {OTPMAGIC_MASK, OTPMAGIC_SHIFT, OTP_PROG, false})
#define OTP0_BYTE_0_READ_DATA_MASK   0x000000FF
#define OTP0_BYTE_0_READ_DATA_SHIFT  0
#define OTP0_BYTE_0_READ_DATA_FIELD  ((RegisterField) {OTP0_BYTE_0_READ_DATA_MASK, OTP0_BYTE_0_READ_DATA_SHIFT, OTP_READ, false})
#define OTP1_BYTE_1_READ_DATA_MASK   0x0000FF00
#define OTP1_BYTE_1_READ_DATA_SHIFT  8
#define OTP1_BYTE_1_READ_DATA_FIELD  ((RegisterField) {OTP1_BYTE_1_READ_DATA_MASK, OTP1_BYTE_1_READ_DATA_SHIFT, OTP_READ, false})
#define OTP2_BYTE_2_READ_DATA_MASK   0x00FF0000
#define OTP2_BYTE_2_READ_DATA_SHIFT  16
#define OTP2_BYTE_2_READ_DATA_FIELD  ((RegisterField) {OTP2_BYTE_2_READ_DATA_MASK, OTP2_BYTE_2_READ_DATA_SHIFT, OTP_READ, false})
#define ENN_MASK                     0x00000001
#define ENN_SHIFT                    0
#define ENN_FIELD                    ((RegisterField) {ENN_MASK, ENN_SHIFT, IOIN, false})
#define MS1_MASK                     0x00000004
#define MS1_SHIFT                    2
#define MS1_FIELD                    ((RegisterField) {MS1_MASK, MS1_SHIFT, IOIN, false})
#define MS2_MASK                     0x00000008
#define MS2_SHIFT                    3
#define MS2_FIELD                    ((RegisterField) {MS2_MASK, MS2_SHIFT, IOIN, false})
#define DIAG_MASK                    0x00000010
#define DIAG_SHIFT                   4
#define DIAG_FIELD                   ((RegisterField) {DIAG_MASK, DIAG_SHIFT, IOIN, false})
#define PDN_UART_MASK                0x00000040
#define PDN_UART_SHIFT               6
#define PDN_UART_FIELD               ((RegisterField) {PDN_UART_MASK, PDN_UART_SHIFT, IOIN, false})
#define STEP_MASK                    0x00000080
#define STEP_SHIFT                   7
#define STEP_FIELD                   ((RegisterField) {STEP_MASK, STEP_SHIFT, IOIN, false})
#define SEL_A_MASK                   0x00000100
#define SEL_A_SHIFT                  8
#define SEL_A_FIELD                  ((RegisterField) {SEL_A_MASK, SEL_A_SHIFT, IOIN, false})
#define DIR_MASK                     0x00000200
#define DIR_SHIFT                    9
#define DIR_FIELD                    ((RegisterField) {DIR_MASK, DIR_SHIFT, IOIN, false})
#define VERSION_MASK                 0xFF000000
#define VERSION_SHIFT                24
#define VERSION_FIELD                ((RegisterField) {VERSION_MASK, VERSION_SHIFT, IOIN, false})
#define FCLKTRIM_MASK                0x0000001F
#define FCLKTRIM_SHIFT               0
#define FCLKTRIM_FIELD               ((RegisterField) {FCLKTRIM_MASK, FCLKTRIM_SHIFT, FACTORY_CONF, false})
#define OTTRIM_MASK                  0x00000300
#define OTTRIM_SHIFT                 8
#define OTTRIM_FIELD                 ((RegisterField) {OTTRIM_MASK, OTTRIM_SHIFT, FACTORY_CONF, false})
#define IHOLD_MASK                   0x0000001F
#define IHOLD_SHIFT                  0
#define IHOLD_FIELD                  ((RegisterField) {IHOLD_MASK, IHOLD_SHIFT, IHOLD_IRUN, false})
#define IRUN_MASK                    0x00001F00
#define IRUN_SHIFT                   8
#define IRUN_FIELD                   ((RegisterField) {IRUN_MASK, IRUN_SHIFT, IHOLD_IRUN, false})
#define IHOLDDELAY_MASK              0x000F0000
#define IHOLDDELAY_SHIFT             16
#define IHOLDDELAY_FIELD             ((RegisterField) {IHOLDDELAY_MASK, IHOLDDELAY_SHIFT, IHOLD_IRUN, false})
#define TPOWERDOWN_MASK              0x000000FF
#define TPOWERDOWN_SHIFT             0
#define TPOWERDOWN_FIELD             ((RegisterField) {TPOWERDOWN_MASK, TPOWERDOWN_SHIFT, TPOWERDOWN, false})
#define TSTEP_MASK                   0x000FFFFF
#define TSTEP_SHIFT                  0
#define TSTEP_FIELD                  ((RegisterField) {TSTEP_MASK, TSTEP_SHIFT, TSTEP, false})
#define TPWMTHRS_MASK                0x000FFFFF
#define TPWMTHRS_SHIFT               0
#define TPWMTHRS_FIELD               ((RegisterField) {TPWMTHRS_MASK, TPWMTHRS_SHIFT, TPWMTHRS, false})
#define VACTUAL_MASK                 0x00FFFFFF
#define VACTUAL_SHIFT                0
#define VACTUAL_FIELD                ((RegisterField) {VACTUAL_MASK, VACTUAL_SHIFT, VACTUAL, true})
#define MSCNT_MASK                   0x000003FF
#define MSCNT_SHIFT                  0
#define MSCNT_FIELD                  ((RegisterField) {MSCNT_MASK, MSCNT_SHIFT, MSCNT, false})
#define CUR_A_MASK                   0x000001FF
#define CUR_A_SHIFT                  0
#define CUR_A_FIELD                  ((RegisterField) {CUR_A_MASK, CUR_A_SHIFT, MSCURACT, true})
#define CUR_B_MASK                   0x01FF0000
#define CUR_B_SHIFT                  16
#define CUR_B_FIELD                  ((RegisterField) {CUR_B_MASK, CUR_B_SHIFT, MSCURACT, true})
#define TOFF_MASK                    0x0000000F
#define TOFF_SHIFT                   0
#define TOFF_FIELD                   ((RegisterField) {TOFF_MASK, TOFF_SHIFT, CHOPCONF, false})
#define HSTRT_MASK                   0x00000070
#define HSTRT_SHIFT                  4
#define HSTRT_FIELD                  ((RegisterField) {HSTRT_MASK, HSTRT_SHIFT, CHOPCONF, false})
#define HEND_MASK                    0x00000780
#define HEND_SHIFT                   7
#define HEND_FIELD                   ((RegisterField) {HEND_MASK, HEND_SHIFT, CHOPCONF, false})
#define TBL_MASK                     0x00018000
#define TBL_SHIFT                    15
#define TBL_FIELD                    ((RegisterField) {TBL_MASK, TBL_SHIFT, CHOPCONF, false})
#define VSENSE_MASK                  0x00020000
#define VSENSE_SHIFT                 17
#define VSENSE_FIELD                 ((RegisterField) {VSENSE_MASK, VSENSE_SHIFT, CHOPCONF, false})
#define MRES_MASK                    0x0F000000
#define MRES_SHIFT                   24
#define MRES_FIELD                   ((RegisterField) {MRES_MASK, MRES_SHIFT, CHOPCONF, false})
#define INTPOL_MASK                  0x10000000
#define INTPOL_SHIFT                 28
#define INTPOL_FIELD                 ((RegisterField) {INTPOL_MASK, INTPOL_SHIFT, CHOPCONF, false})
#define DEDGE_MASK                   0x20000000
#define DEDGE_SHIFT                  29
#define DEDGE_FIELD                  ((RegisterField) {DEDGE_MASK, DEDGE_SHIFT, CHOPCONF, false})
#define DISS2G_MASK                  0x40000000
#define DISS2G_SHIFT                 30
#define DISS2G_FIELD                 ((RegisterField) {DISS2G_MASK, DISS2G_SHIFT, CHOPCONF, false})
#define DISS2VS_MASK                 0x80000000
#define DISS2VS_SHIFT                31
#define DISS2VS_FIELD                ((RegisterField) {DISS2VS_MASK, DISS2VS_SHIFT, CHOPCONF, false})
#define OTPW_MASK                    0x00000001
#define OTPW_SHIFT                   0
#define OTPW_FIELD                   ((RegisterField) {OTPW_MASK, OTPW_SHIFT, DRV_STATUS, false})
#define OT_MASK                      0x00000002
#define OT_SHIFT                     1
#define OT_FIELD                     ((RegisterField) {OT_MASK, OT_SHIFT, DRV_STATUS, false})
#define S2GA_MASK                    0x00000004
#define S2GA_SHIFT                   2
#define S2GA_FIELD                   ((RegisterField) {S2GA_MASK, S2GA_SHIFT, DRV_STATUS, false})
#define S2GB_MASK                    0x00000008
#define S2GB_SHIFT                   3
#define S2GB_FIELD                   ((RegisterField) {S2GB_MASK, S2GB_SHIFT, DRV_STATUS, false})
#define S2VSA_MASK                   0x00000010
#define S2VSA_SHIFT                  4
#define S2VSA_FIELD                  ((RegisterField) {S2VSA_MASK, S2VSA_SHIFT, DRV_STATUS, false})
#define S2VSB_MASK                   0x00000020
#define S2VSB_SHIFT                  5
#define S2VSB_FIELD                  ((RegisterField) {S2VSB_MASK, S2VSB_SHIFT, DRV_STATUS, false})
#define OLA_MASK                     0x00000040
#define OLA_SHIFT                    6
#define OLA_FIELD                    ((RegisterField) {OLA_MASK, OLA_SHIFT, DRV_STATUS, false})
#define OLB_MASK                     0x00000080
#define OLB_SHIFT                    7
#define OLB_FIELD                    ((RegisterField) {OLB_MASK, OLB_SHIFT, DRV_STATUS, false})
#define T120_MASK                    0x00000100
#define T120_SHIFT                   8
#define T120_FIELD                   ((RegisterField) {T120_MASK, T120_SHIFT, DRV_STATUS, false})
#define T143_MASK                    0x00000200
#define T143_SHIFT                   9
#define T143_FIELD                   ((RegisterField) {T143_MASK, T143_SHIFT, DRV_STATUS, false})
#define T150_MASK                    0x00000400
#define T150_SHIFT                   10
#define T150_FIELD                   ((RegisterField) {T150_MASK, T150_SHIFT, DRV_STATUS, false})
#define T157_MASK                    0x00000800
#define T157_SHIFT                   11
#define T157_FIELD                   ((RegisterField) {T157_MASK, T157_SHIFT, DRV_STATUS, false})
#define CS_ACTUAL_MASK               0x001F0000
#define CS_ACTUAL_SHIFT              16
#define CS_ACTUAL_FIELD              ((RegisterField) {CS_ACTUAL_MASK, CS_ACTUAL_SHIFT, DRV_STATUS, false})
#define STEALTH_MASK                 0x40000000
#define STEALTH_SHIFT                30
#define STEALTH_FIELD                ((RegisterField) {STEALTH_MASK, STEALTH_SHIFT, DRV_STATUS, false})
#define STST_MASK                    0x80000000
#define STST_SHIFT                   31
#define STST_FIELD                   ((RegisterField) {STST_MASK, STST_SHIFT, DRV_STATUS, false})
#define PWM_OFS_MASK                 0x000000FF
#define PWM_OFS_SHIFT                0
#define PWM_OFS_FIELD                ((RegisterField) {PWM_OFS_MASK, PWM_OFS_SHIFT, PWM_CONF, false})
#define PWM_GRAD_MASK                0x0000FF00
#define PWM_GRAD_SHIFT               8
#define PWM_GRAD_FIELD               ((RegisterField) {PWM_GRAD_MASK, PWM_GRAD_SHIFT, PWM_CONF, false})
#define PWM_FREQ_MASK                0x00030000
#define PWM_FREQ_SHIFT               16
#define PWM_FREQ_FIELD               ((RegisterField) {PWM_FREQ_MASK, PWM_FREQ_SHIFT, PWM_CONF, false})
#define PWM_AUTOSCALE_MASK           0x00040000
#define PWM_AUTOSCALE_SHIFT          18
#define PWM_AUTOSCALE_FIELD          ((RegisterField) {PWM_AUTOSCALE_MASK, PWM_AUTOSCALE_SHIFT, PWM_CONF, false})
#define PWM_AUTOGRAD_MASK            0x00080000
#define PWM_AUTOGRAD_SHIFT           19
#define PWM_AUTOGRAD_FIELD           ((RegisterField) {PWM_AUTOGRAD_MASK, PWM_AUTOGRAD_SHIFT, PWM_CONF, false})
#define FREEWHEEL_MASK               0x00300000
#define FREEWHEEL_SHIFT              20
#define FREEWHEEL_FIELD              ((RegisterField) {FREEWHEEL_MASK, FREEWHEEL_SHIFT, PWM_CONF, false})
#define PWM_REG_MASK                 0x0F000000
#define PWM_REG_SHIFT                24
#define PWM_REG_FIELD                ((RegisterField) {PWM_REG_MASK, PWM_REG_SHIFT, PWM_CONF, false})
#define PWM_LIM_MASK                 0xF0000000
#define PWM_LIM_SHIFT                28
#define PWM_LIM_FIELD                ((RegisterField) {PWM_LIM_MASK, PWM_LIM_SHIFT, PWM_CONF, false})
#define PWM_SCALE_SUM_MASK           0x000000FF
#define PWM_SCALE_SUM_SHIFT          0
#define PWM_SCALE_SUM_FIELD          ((RegisterField) {PWM_SCALE_SUM_MASK, PWM_SCALE_SUM_SHIFT, PWM_SCALE, false})
#define PWM_SCALE_AUTO_MASK          0x01FF0000
#define PWM_SCALE_AUTO_SHIFT         16
#define PWM_SCALE_AUTO_FIELD         ((RegisterField) {PWM_SCALE_AUTO_MASK, PWM_SCALE_AUTO_SHIFT, PWM_SCALE, true})
#define PWM_OFS_AUTO_MASK            0x000000FF
#define PWM_OFS_AUTO_SHIFT           0
#define PWM_OFS_AUTO_FIELD           ((RegisterField) {PWM_OFS_AUTO_MASK, PWM_OFS_AUTO_SHIFT, PWM_AUTO, false})
#define PWM_GRAD_AUTO_MASK           0x00FF0000
#define PWM_GRAD_AUTO_SHIFT          16
#define PWM_GRAD_AUTO_FIELD          ((RegisterField) {PWM_GRAD_AUTO_MASK, PWM_GRAD_AUTO_SHIFT, PWM_AUTO, false})

// clang-format on

}  // namespace tmc2208
}  // namespace esphome
