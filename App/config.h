#ifndef YFOC_CONFIG_H
#define YFOC_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#define CFG_NODE_ID                 (1U)
#define CFG_POLE_PAIRS              (10U)

#define CFG_SYSCLK_HZ               (170000000UL)
#define CFG_PWM_HZ                  (20000UL)
#define CFG_CTRL_HZ                 (4000UL)
#define CFG_PWM_DT_S                (1.0f / (float)CFG_PWM_HZ)
#define CFG_CTRL_DT_S               (1.0f / (float)CFG_CTRL_HZ)

#define CFG_TIM1_ARR                ((uint32_t)((CFG_SYSCLK_HZ / (2UL * CFG_PWM_HZ)) - 1UL))
#define CFG_TIM1_DEADTIME_DTG       (17U)   /* ~100 ns at 170 MHz, CKD=1 */

#define CFG_V_LIMIT                 (0.4f)     /* pu; 1.0 = PWM full scale */
#define CFG_V_SLEW_PU_S             (100.0f)   /* max |d(Vd,Vq)/dt|, pu/s */
#define CFG_VEL_PLL_HZ              (50.0f)   /* Type-2 PLL bandwidth; 40-120 typical */
#define CFG_VEL_PLL_ZETA            (0.707f)   /* zeta = 0.707 for critically damped response */

#define CFG_CAN_MOTION_BASE         (0x100U)
#define CFG_CAN_VEL_BASE            (0x140U)
#define CFG_CAN_GAINS_BASE          (0x180U)
#define CFG_CAN_POS_BASE            (0x1C0U)
#define CFG_CAN_MGMT_BASE           (0x200U)
#define CFG_CAN_ACK_BASE            (0x280U)
#define CFG_CAN_FB_BASE             (0x300U)
#define CFG_CAN_STATUS_BASE         (0x380U)
#define CFG_CAN_CALI_RPT_BASE       (0x3C0U)
#define CFG_CAN_RX_SLOTS            (16U)

#define CFG_POS_LSB                 (0.0001f)
#define CFG_VEL_LSB                 (0.1f)
#define CFG_KP_LSB                  (0.01f)
#define CFG_KD_LSB                  (0.001f)
#define CFG_KP_VEL_LSB              (0.001f)
#define CFG_KI_VEL_LSB              (0.001f)
#define CFG_KP_POS_LSB              (0.01f)
#define CFG_KI_POS_LSB              (0.001f)
#define CFG_KD_POS_LSB              (0.001f)
#define CFG_TORQUE_LSB              (0.001f)

#define CFG_VEL_CMD_MIN             (-100.0f)
#define CFG_VEL_CMD_MAX             (100.0f)
#define CFG_POS_VMAX_DEFAULT        (100.0f)
#define CFG_POS_ACC_DEFAULT         (80.0f)    /* rad/s^2; v_lim = min(vmax, sqrt(2 a |ep|)) */
#define CFG_POS_SETTLE_RAD          (1.0f)     /* blend to PID inside this error */
#define CFG_VEL_KP_DEFAULT          (0.01f)    /* pu/(rad/s) */
#define CFG_VEL_KI_DEFAULT          (0.3f)    /* pu/rad */
#define CFG_POS_KP_DEFAULT          (3.0f)    /* 1/s */
#define CFG_POS_KI_DEFAULT          (0.1f)     /* 1/s^2 */
#define CFG_POS_KD_DEFAULT          (0.0f)
#define CFG_KP_MIN                  (0.0f)
#define CFG_KP_MAX                  (5.0f)
#define CFG_KD_MIN                  (0.0f)
#define CFG_KD_MAX                  (1.0f)
#define CFG_KP_VEL_MAX              (1.0f)
#define CFG_KI_VEL_MAX              (10.0f)
#define CFG_KP_POS_MAX              (50.0f)
#define CFG_KI_POS_MAX              (5.0f)
#define CFG_KD_POS_MAX              (5.0f)

#define CFG_CALI_LOCK_V             (0.15f)
#define CFG_CALI_LOCK_MS            (500U)
#define CFG_CALI_ROTATE_ELEC_RAD_S  (6.283185307179586f)    /* 1 electrical rev / s */
#define CFG_CALI_PP_ELEC_REVS       (4U)
#define CFG_CALI_ROTATE_MS          (1000U * CFG_CALI_PP_ELEC_REVS)
#define CFG_CALI_ROTATE_SAMPLE_MS   (10U)
#define CFG_CALI_MIN_MECH_DELTA     (0.05f)
#define CFG_CALI_PP_MIN             (2U)
#define CFG_CALI_PP_MAX             (30U)
#define CFG_CALI_PP_MAX_RESIDUAL    (0.25f)
#define CFG_CALI_PROBE_VQ           (0.12f)
#define CFG_CALI_PROBE_MS           (400U)
#define CFG_CALI_MIN_VEL            (0.3f)
#define CFG_CALI_REPORT_MS          (50U)

#define CFG_DEBUG_PERIOD_MS         (200U)

#define CFG_NVIC_TIM1               (1U)
#define CFG_NVIC_TIM6               (3U)
#define CFG_NVIC_CAN                (5U)

#ifdef __cplusplus
}
#endif

#endif /* YFOC_CONFIG_H */
