#ifndef SERVO_H
#define SERVO_H

#define SERVO_PWM PWMB_CH3_P33

// #define DTY_MID 4000
// #define DTY_MAX DTY_MID+700
// #define DTY_MIN DTY_MID-700

typedef struct {
float v1; 
float v2; 
float z1; 
float z2; 
float z3; 
int u;  
float h;  
float r;  
float wc; 
float w0; 
float b0; 
} LADRC_NUM;


extern void Servo_PID(int err);
extern void Servo_init();
void Ladrc_Init(LADRC_NUM *LADRC_);
void Ladrc_Reset(LADRC_NUM *LADRC_);
#endif