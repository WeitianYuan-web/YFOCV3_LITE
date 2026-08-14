#ifndef FOC_SVPWM_H
#define FOC_SVPWM_H

#ifdef __cplusplus
extern "C" {
#endif

void Foc_SvpwmOffsetOptimized(float u_alpha,
                              float u_beta,
                              float *duty_a,
                              float *duty_b,
                              float *duty_c);

#ifdef __cplusplus
}
#endif

#endif /* FOC_SVPWM_H */
