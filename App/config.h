#ifndef YFOC_CONFIG_H
#define YFOC_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#define CFG_NODE_ID                 (1U)
#define CFG_POLE_PAIRS              (14U)

#define CFG_ENCODER_MT6701          (0U)
#define CFG_ENCODER_KTH7812         (1U)
#define CFG_ENCODER_TYPE            CFG_ENCODER_MT6701

#define CFG_SYSCLK_HZ               (170000000UL)
#define CFG_PWM_HZ                  (20000UL)
#define CFG_CTRL_HZ                 (4000UL)
#define CFG_PWM_DT_S                (1.0f / (float)CFG_PWM_HZ)
#define CFG_CTRL_DT_S               (1.0f / (float)CFG_CTRL_HZ)

#define CFG_TIM1_ARR                ((uint32_t)((CFG_SYSCLK_HZ / (2UL * CFG_PWM_HZ)) - 1UL))
#define CFG_TIM1_DEADTIME_DTG       (17U)   /* ~100 ns at 170 MHz, CKD=1 */

#define CFG_V_LIMIT                 (0.50f)    /* pu; also clamps cali Vd/Vq */
#define CFG_V_SLEW_PU_S             (100.0f)   /* max |d(Vd,Vq)/dt|, pu/s */
#define CFG_VEL_LPF_HZ              (500.0f)  /* 2nd-order Butterworth on d(theta_e)/dt */

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

#define CFG_VEL_CMD_MIN             (-3276.8f) /* int16 * 0.1 rad/s */
#define CFG_VEL_CMD_MAX             (3276.7f)
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

#define CFG_CALI_LOCK_V             (0.05f)
#define CFG_CALI_LOCK_MS            (300U)
#define CFG_CALI_ROTATE_ELEC_RAD_S  (3.141592653589793f) /* 0.5 elec rev/s */
#define CFG_CALI_PP_ELEC_REVS       (4U)   /* per direction; both ways still 8 elec revs */
#define CFG_CALI_ROTATE_MS          (2000U * CFG_CALI_PP_ELEC_REVS) /* 2π/rate = 2 s/rev */
#define CFG_CALI_ROTATE_SAMPLE_MS   (10U)
#define CFG_CALI_OFFSET_SKIP_MS     (250U)
#define CFG_CALI_SETTLE_MS          (40U)
#define CFG_CALI_MIN_MECH_DELTA     (0.05f)
#define CFG_CALI_PP_MIN             (2U)
#define CFG_CALI_PP_MAX             (30U)
#define CFG_CALI_PP_MAX_RESIDUAL    (0.40f)
#define CFG_CALI_PROBE_VQ           (0.05f)
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
