// Constants
#define REGISTER_COUNT   128
#define WRITE_BIT        0x80
#define ADDRESS_MASK     0x7F
#define MAX_VELOCITY     (int32_t) 2147483647
#define MAX_ACCELERATION (uint32_t) 4294967295uL

// Registers
#define GCONF         0x00
#define GSTAT         0x01
#define IFCNT         0x02
#define SLAVECONF     0x03
#define IOIN          0x06

#define IHOLD_IRUN    0x10
#define TPOWERDOWN    0x11
#define TSTEP         0x12

#define TCOOLTHRS     0x14

#define VACTUAL       0x22
#define XDIRECT       0x22

#define SGTHRS        0x40
#define SG_VALUE      0x41
#define COOLCONF      0x42

#define MSCNT         0x6A
#define CHOPCONF      0x6C
#define DRVSTATUS     0x6F
#define PWMCONF       0x70
#define PWMSCALE      0x71
#define PWMAUTO       0x72


// Register fields (manually add)
#define EN_SPREADCYCLE_MASK 0x04 // GCONF // par_mode (Reset default=0)
#define EN_SPREADCYCLE_SHIFT 2 // par_mode (Reset default=0)
#define EN_SPREADCYCLE_FIELD   ((RegisterField) {EN_SPREADCYCLE_MASK, EN_SPREADCYCLE_SHIFT, GCONF, false})

// Register fields
#define _MASK                    0x00000001
#define _SHIFT                   0
#define _FIELD                   ((RegisterField) {_MASK, _SHIFT, GCONF, false})
#define EXTCAP_MASK              0x00000002
#define EXTCAP_SHIFT             1
#define EXTCAP_FIELD             ((RegisterField) {EXTCAP_MASK, EXTCAP_SHIFT, GCONF, false})
#define SHAFT_MASK               0x00000008
#define SHAFT_SHIFT              3
#define SHAFT_FIELD              ((RegisterField) {SHAFT_MASK, SHAFT_SHIFT, GCONF, false})
#define DIAG_INDEX_MASK          0x00000010
#define DIAG_INDEX_SHIFT         4
#define DIAG_INDEX_FIELD         ((RegisterField) {DIAG_INDEX_MASK, DIAG_INDEX_SHIFT, GCONF, false})
#define DIAG_STEP_MASK           0x00000020
#define DIAG_STEP_SHIFT          5
#define DIAG_STEP_FIELD          ((RegisterField) {DIAG_STEP_MASK, DIAG_STEP_SHIFT, GCONF, false})
#define MULTISTEP_FILT_MASK      0x00000040
#define MULTISTEP_FILT_SHIFT     6
#define MULTISTEP_FILT_FIELD     ((RegisterField) {MULTISTEP_FILT_MASK, MULTISTEP_FILT_SHIFT, GCONF, false})
#define TEST_MODE_MASK           0x00000080
#define TEST_MODE_SHIFT          7
#define TEST_MODE_FIELD          ((RegisterField) {TEST_MODE_MASK, TEST_MODE_SHIFT, GCONF, false})
#define RESET_MASK               0x00000001
#define RESET_SHIFT              0
#define RESET_FIELD              ((RegisterField) {RESET_MASK, RESET_SHIFT, GSTAT, false})
#define DRV_ERR_MASK             0x00000002
#define DRV_ERR_SHIFT            1
#define DRV_ERR_FIELD            ((RegisterField) {DRV_ERR_MASK, DRV_ERR_SHIFT, GSTAT, false})
#define U3V5_MASK                0x00000004
#define U3V5_SHIFT               2
#define U3V5_FIELD               ((RegisterField) {U3V5_MASK, U3V5_SHIFT, GSTAT, false})
#define IFCNT_MASK               0x000000FF
#define IFCNT_SHIFT              0
#define IFCNT_FIELD              ((RegisterField) {IFCNT_MASK, IFCNT_SHIFT, IFCNT, false})
#define SLAVECONF_MASK           0x00000F00
#define SLAVECONF_SHIFT          8
#define SLAVECONF_FIELD          ((RegisterField) {SLAVECONF_MASK, SLAVECONF_SHIFT, SLAVECONF, false})
#define EN_MASK                  0x00000001
#define EN_SHIFT                 0
#define EN_FIELD                 ((RegisterField) {EN_MASK, EN_SHIFT, IOIN, false})
#define NSTDBY_MASK              0x00000002
#define NSTDBY_SHIFT             1
#define NSTDBY_FIELD             ((RegisterField) {NSTDBY_MASK, NSTDBY_SHIFT, IOIN, false})
#define MS1_MASK                 0x00000004
#define MS1_SHIFT                2
#define MS1_FIELD                ((RegisterField) {MS1_MASK, MS1_SHIFT, IOIN, false})
#define MS2_MASK                 0x00000008
#define MS2_SHIFT                3
#define MS2_FIELD                ((RegisterField) {MS2_MASK, MS2_SHIFT, IOIN, false})
#define DIAG_MASK                0x00000010
#define DIAG_SHIFT               4
#define DIAG_FIELD               ((RegisterField) {DIAG_MASK, DIAG_SHIFT, IOIN, false})
#define STEPPER_CLK_INPUT_MASK   0x00000020
#define STEPPER_CLK_INPUT_SHIFT  5
#define STEPPER_CLK_INPUT_FIELD  ((RegisterField) {STEPPER_CLK_INPUT_MASK, STEPPER_CLK_INPUT_SHIFT, IOIN, false})
#define PDN_UART_MASK            0x00000040
#define PDN_UART_SHIFT           6
#define PDN_UART_FIELD           ((RegisterField) {PDN_UART_MASK, PDN_UART_SHIFT, IOIN, false})
#define MODE_INPUT_MASK          0x00000080
#define MODE_INPUT_SHIFT         7
#define MODE_INPUT_FIELD         ((RegisterField) {MODE_INPUT_MASK, MODE_INPUT_SHIFT, IOIN, false})
#define STEP_MASK                0x00000100
#define STEP_SHIFT               8
#define STEP_FIELD               ((RegisterField) {STEP_MASK, STEP_SHIFT, IOIN, false})
#define DIR_MASK                 0x00000200
#define DIR_SHIFT                9
#define DIR_FIELD                ((RegisterField) {DIR_MASK, DIR_SHIFT, IOIN, false})
#define COMP_A1A2_MASK           0x00000400
#define COMP_A1A2_SHIFT          10
#define COMP_A1A2_FIELD          ((RegisterField) {COMP_A1A2_MASK, COMP_A1A2_SHIFT, IOIN, false})
#define COMP_B1B2_MASK           0x00000800
#define COMP_B1B2_SHIFT          11
#define COMP_B1B2_FIELD          ((RegisterField) {COMP_B1B2_MASK, COMP_B1B2_SHIFT, IOIN, false})
#define VERSION_MASK             0xFF000000
#define VERSION_SHIFT            24
#define VERSION_FIELD            ((RegisterField) {VERSION_MASK, VERSION_SHIFT, IOIN, false})
#define IHOLD_MASK               0x0000001F
#define IHOLD_SHIFT              0
#define IHOLD_FIELD              ((RegisterField) {IHOLD_MASK, IHOLD_SHIFT, IHOLD_IRUN, false})
#define IRUN_MASK                0x00001F00
#define IRUN_SHIFT               8
#define IRUN_FIELD               ((RegisterField) {IRUN_MASK, IRUN_SHIFT, IHOLD_IRUN, false})
#define IHOLDDELAY_MASK          0x000F0000
#define IHOLDDELAY_SHIFT         16
#define IHOLDDELAY_FIELD         ((RegisterField) {IHOLDDELAY_MASK, IHOLDDELAY_SHIFT, IHOLD_IRUN, false})
#define TPOWERDOWN_MASK          0x000000FF
#define TPOWERDOWN_SHIFT         0
#define TPOWERDOWN_FIELD         ((RegisterField) {TPOWERDOWN_MASK, TPOWERDOWN_SHIFT, TPOWERDOWN, false})
#define TSTEP_MASK               0x000FFFFF
#define TSTEP_SHIFT              0
#define TSTEP_FIELD              ((RegisterField) {TSTEP_MASK, TSTEP_SHIFT, TSTEP, false})
#define TCOOLTHRS_MASK           0xFFFFFFFF
#define TCOOLTHRS_SHIFT          0
#define TCOOLTHRS_FIELD          ((RegisterField) {TCOOLTHRS_MASK, TCOOLTHRS_SHIFT, TCOOLTHRS, true})
#define VACTUAL_MASK             0x00FFFFFF
#define VACTUAL_SHIFT            0
#define VACTUAL_FIELD            ((RegisterField) {VACTUAL_MASK, VACTUAL_SHIFT, VACTUAL, true})
#define SGTHRS_MASK              0x000000FF
#define SGTHRS_SHIFT             0
#define SGTHRS_FIELD             ((RegisterField) {SGTHRS_MASK, SGTHRS_SHIFT, SGTHRS, false})
#define SG_VALUE_MASK            0x000003FF
#define SG_VALUE_SHIFT           0
#define SG_VALUE_FIELD           ((RegisterField) {SG_VALUE_MASK, SG_VALUE_SHIFT, SG_VALUE, false})
#define SEMIN_MASK               0x0000000F
#define SEMIN_SHIFT              0
#define SEMIN_FIELD              ((RegisterField) {SEMIN_MASK, SEMIN_SHIFT, COOLCONF, false})
#define SEUP_MASK                0x00000060
#define SEUP_SHIFT               5
#define SEUP_FIELD               ((RegisterField) {SEUP_MASK, SEUP_SHIFT, COOLCONF, false})
#define SEMAX_MASK               0x00000F00
#define SEMAX_SHIFT              8
#define SEMAX_FIELD              ((RegisterField) {SEMAX_MASK, SEMAX_SHIFT, COOLCONF, false})
#define SEDN_MASK                0x00006000
#define SEDN_SHIFT               13
#define SEDN_FIELD               ((RegisterField) {SEDN_MASK, SEDN_SHIFT, COOLCONF, false})
#define SEIMIN_MASK              0x00008000
#define SEIMIN_SHIFT             15
#define SEIMIN_FIELD             ((RegisterField) {SEIMIN_MASK, SEIMIN_SHIFT, COOLCONF, false})
#define MSCNT_MASK               0x000003FF
#define MSCNT_SHIFT              0
#define MSCNT_FIELD              ((RegisterField) {MSCNT_MASK, MSCNT_SHIFT, MSCNT, false})
#define CUR_A_MASK               0x000001FF
#define CUR_A_SHIFT              0
#define CUR_A_FIELD              ((RegisterField) {CUR_A_MASK, CUR_A_SHIFT, MSCURACT, true})
#define CUR_B_MASK               0x01FF0000
#define CUR_B_SHIFT              16
#define CUR_B_FIELD              ((RegisterField) {CUR_B_MASK, CUR_B_SHIFT, MSCURACT, true})
#define ENABLEDRV_MASK           0x00000001
#define ENABLEDRV_SHIFT          0
#define ENABLEDRV_FIELD          ((RegisterField) {ENABLEDRV_MASK, ENABLEDRV_SHIFT, CHOPCONF, false})
#define TBL_MASK                 0x00018000
#define TBL_SHIFT                15
#define TBL_FIELD                ((RegisterField) {TBL_MASK, TBL_SHIFT, CHOPCONF, false})
#define MRES_MASK                0x0F000000
#define MRES_SHIFT               24
#define MRES_FIELD               ((RegisterField) {MRES_MASK, MRES_SHIFT, CHOPCONF, false})
#define INTPOL_MASK              0x10000000
#define INTPOL_SHIFT             28
#define INTPOL_FIELD             ((RegisterField) {INTPOL_MASK, INTPOL_SHIFT, CHOPCONF, false})
#define DEDGE_MASK               0x20000000
#define DEDGE_SHIFT              29
#define DEDGE_FIELD              ((RegisterField) {DEDGE_MASK, DEDGE_SHIFT, CHOPCONF, false})
#define DISS2G_MASK              0x40000000
#define DISS2G_SHIFT             30
#define DISS2G_FIELD             ((RegisterField) {DISS2G_MASK, DISS2G_SHIFT, CHOPCONF, false})
#define DISS2VS_MASK             0x80000000
#define DISS2VS_SHIFT            31
#define DISS2VS_FIELD            ((RegisterField) {DISS2VS_MASK, DISS2VS_SHIFT, CHOPCONF, false})
#define OTPW_MASK                0x00000001
#define OTPW_SHIFT               0
#define OTPW_FIELD               ((RegisterField) {OTPW_MASK, OTPW_SHIFT, DRVSTATUS, false})
#define OT_MASK                  0x00000002
#define OT_SHIFT                 1
#define OT_FIELD                 ((RegisterField) {OT_MASK, OT_SHIFT, DRVSTATUS, false})
#define S2GA_MASK                0x00000004
#define S2GA_SHIFT               2
#define S2GA_FIELD               ((RegisterField) {S2GA_MASK, S2GA_SHIFT, DRVSTATUS, false})
#define S2GB_MASK                0x00000008
#define S2GB_SHIFT               3
#define S2GB_FIELD               ((RegisterField) {S2GB_MASK, S2GB_SHIFT, DRVSTATUS, false})
#define S2VSA_MASK               0x00000010
#define S2VSA_SHIFT              4
#define S2VSA_FIELD              ((RegisterField) {S2VSA_MASK, S2VSA_SHIFT, DRVSTATUS, false})
#define S2VSB_MASK               0x00000020
#define S2VSB_SHIFT              5
#define S2VSB_FIELD              ((RegisterField) {S2VSB_MASK, S2VSB_SHIFT, DRVSTATUS, false})
#define OLA_MASK                 0x00000040
#define OLA_SHIFT                6
#define OLA_FIELD                ((RegisterField) {OLA_MASK, OLA_SHIFT, DRVSTATUS, false})
#define OLB_MASK                 0x00000080
#define OLB_SHIFT                7
#define OLB_FIELD                ((RegisterField) {OLB_MASK, OLB_SHIFT, DRVSTATUS, false})
#define T120_MASK                0x00000100
#define T120_SHIFT               8
#define T120_FIELD               ((RegisterField) {T120_MASK, T120_SHIFT, DRVSTATUS, false})
#define T150_MASK                0x00000200
#define T150_SHIFT               9
#define T150_FIELD               ((RegisterField) {T150_MASK, T150_SHIFT, DRVSTATUS, false})
#define CS_ACTUAL_MASK           0x001F0000
#define CS_ACTUAL_SHIFT          16
#define CS_ACTUAL_FIELD          ((RegisterField) {CS_ACTUAL_MASK, CS_ACTUAL_SHIFT, DRVSTATUS, false})
#define STST_MASK                0x80000000
#define STST_SHIFT               31
#define STST_FIELD               ((RegisterField) {STST_MASK, STST_SHIFT, DRVSTATUS, false})
#define PWM_OFS_MASK             0x000000FF
#define PWM_OFS_SHIFT            0
#define PWM_OFS_FIELD            ((RegisterField) {PWM_OFS_MASK, PWM_OFS_SHIFT, PWMCONF, false})
#define PWM_GRAD_MASK            0x0000FF00
#define PWM_GRAD_SHIFT           8
#define PWM_GRAD_FIELD           ((RegisterField) {PWM_GRAD_MASK, PWM_GRAD_SHIFT, PWMCONF, false})
#define PWM_FREQ_MASK            0x00030000
#define PWM_FREQ_SHIFT           16
#define PWM_FREQ_FIELD           ((RegisterField) {PWM_FREQ_MASK, PWM_FREQ_SHIFT, PWMCONF, false})
#define PWM_AUTOSCALE_MASK       0x00040000
#define PWM_AUTOSCALE_SHIFT      18
#define PWM_AUTOSCALE_FIELD      ((RegisterField) {PWM_AUTOSCALE_MASK, PWM_AUTOSCALE_SHIFT, PWMCONF, false})
#define PWM_AUTOGRAD_MASK        0x00080000
#define PWM_AUTOGRAD_SHIFT       19
#define PWM_AUTOGRAD_FIELD       ((RegisterField) {PWM_AUTOGRAD_MASK, PWM_AUTOGRAD_SHIFT, PWMCONF, false})
#define FREEWHEEL_MASK           0x00300000
#define FREEWHEEL_SHIFT          20
#define FREEWHEEL_FIELD          ((RegisterField) {FREEWHEEL_MASK, FREEWHEEL_SHIFT, PWMCONF, false})
#define PWM_REG_MASK             0x0F000000
#define PWM_REG_SHIFT            24
#define PWM_REG_FIELD            ((RegisterField) {PWM_REG_MASK, PWM_REG_SHIFT, PWMCONF, false})
#define PWM_LIM_MASK             0xF0000000
#define PWM_LIM_SHIFT            28
#define PWM_LIM_FIELD            ((RegisterField) {PWM_LIM_MASK, PWM_LIM_SHIFT, PWMCONF, false})
#define PWM_SCALE_SUM_MASK       0x000000FF
#define PWM_SCALE_SUM_SHIFT      0
#define PWM_SCALE_SUM_FIELD      ((RegisterField) {PWM_SCALE_SUM_MASK, PWM_SCALE_SUM_SHIFT, PWMSCALE, false})
#define PWM_SCALE_AUTO_MASK      0xFFFF0000
#define PWM_SCALE_AUTO_SHIFT     16
#define PWM_SCALE_AUTO_FIELD     ((RegisterField) {PWM_SCALE_AUTO_MASK, PWM_SCALE_AUTO_SHIFT, PWMSCALE, true})
#define PWM_OFS_AUTO_MASK        0x000000FF
#define PWM_OFS_AUTO_SHIFT       0
#define PWM_OFS_AUTO_FIELD       ((RegisterField) {PWM_OFS_AUTO_MASK, PWM_OFS_AUTO_SHIFT, PWMAUTO, false})
#define PWM_GRAD_AUTO_MASK       0x00FF0000
#define PWM_GRAD_AUTO_SHIFT      16
#define PWM_GRAD_AUTO_FIELD      ((RegisterField) {PWM_GRAD_AUTO_MASK, PWM_GRAD_AUTO_SHIFT, PWMAUTO, false})
