#ifndef MOTOR_H
#define MOTOR_H

#define Motor_MAX 5000
#define Motor_MIN -5000

#define LMOTOR_DIR P1_0
#define LMOTOR_PWM PWMA_CH2N_P13 

#define RMOTOR_DIR P2_4
#define RMOTOR_PWM PWMA_CH4P_P26

#define LMOTOR_FORWARD  P10=1
#define LMOTOR_RETREAT  P10=0
#define RMOTOR_FORWARD  P24=0
#define RMOTOR_RETREAT  P24=1

#define L_ENCONDER_FORWARD P32==0
#define R_ENCONDER_FORWARD P06==1
//MINI                                                                                                                                                                                                                                                                                                                        
// #define RMOTOR_FORWARD  P24=0;
// #define RMOTOR_RETREAT  P24=1;

// C                                                                                                                                                                                                                                                                                                                          


void Motor_Init();
void START_CAR();
void Motor_Forward(int lv,int rv);
void Get_Speed(void);
void LMotor_PI(int16 target,int16 eyaw);
void RMotor_PI(int16 target,int16 eyaw);
void mt_akermann();
void Speed_Judge();
#endif