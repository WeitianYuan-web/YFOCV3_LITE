#ifndef PWM_H
#define PWM_H

#ifdef __cplusplus
extern "C" {
#endif

void Pwm_StartTimer(void);
void Pwm_EnableOutputs(void);
void Pwm_DisableOutputs(void);
void Pwm_ApplyDuty(float duty_a, float duty_b, float duty_c);
void CtrlTimer_Start(void);

#ifdef __cplusplus
}
#endif

#endif /* PWM_H */
