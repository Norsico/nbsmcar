#include "SystemConfig.h"

extern ele flag;
LADRC_NUM LADRC;
float r_LADRC   = 100,//fast_track
      h_LADRC   = 0.005;//filiter factor
float b0_LADRC  = 250,//system coefficient
      wc_LADRC  = 100,//delta为fal（e，alpha，delta）函数的线性区间宽度
      w0_LADRC  = 200;//esp feed back 1
void Servo_init()
{
    pwm_init(SERVO_PWM,333,Servo_Mid);
}

void Ladrc_Init(LADRC_NUM *LADRC_)
{
    LADRC_->h = h_LADRC;//step
    LADRC_->r = r_LADRC;//track_vel
    LADRC_->b0 = b0_LADRC;//system coefficent
    LADRC_->wc = wc_LADRC;//eso bandwidth
    LADRC_->w0 = w0_LADRC;//statu feedback
}

void Ladrc_Reset(LADRC_NUM *LADRC_)
{
    LADRC_->z1=0;//step time
    LADRC_->z2=0;//track_vel
    LADRC_->z3=0;//eso bendwidth
}

void Ladrc_Td(LADRC_NUM *LADRC_,float expect)
{
    float fh = -LADRC_->r*LADRC_->r*(LADRC_->v1-expect)-2*LADRC_->r*LADRC_->v2;
    LADRC_->v1+=LADRC_->v2*LADRC_->h;
    LADRC_->v2+=fh*LADRC_->h;
}

void Ladrc_Eso(LADRC_NUM *LADRC_,float feedback)
{
    float B1=3*LADRC_->w0;
    float B2=3*LADRC_->w0*LADRC_->w0;
    float B3=LADRC_->w0*LADRC_->w0*LADRC_->w0;
    float e= LADRC_->z1-feedback;
    LADRC_->z1+= (LADRC_->z2 - B1*e)*LADRC_->h;
    LADRC_->z2+= (LADRC_->z3 - B2*e + LADRC_->b0*LADRC_->u)*LADRC_->h;
    LADRC_->z3+=-B3*e*LADRC_->h;    
}

int decelerate_max = 8000;
int increase_max = 4000;

void  Ladrc_Lf(LADRC_NUM *LADRC_)
{
    float Kp=LADRC_->wc*LADRC_->wc;
    float Kd=2*LADRC_->wc;//kd倍率可以为3

    float e1=LADRC_->v1-LADRC_->z1;
    float e2=LADRC_->v2-LADRC_->z2;
    float u0=Kp*e1+Kd*e2;
    LADRC_->u=(u0-LADRC_->z3)/LADRC_->b0;
    if(LADRC_->u>increase_max)
        LADRC_->u=increase_max;
    else if(LADRC_->u<-decelerate_max)
        LADRC_->u=-decelerate_max;
}

void Ladrc_Process(LADRC_NUM *LADRC_,float Expect,float RealTimeOut)
{
    float  Expect_Value = Expect;
    float  Measure = RealTimeOut;
    Ladrc_Td(LADRC_,Expect_Value);
    Ladrc_Eso(LADRC_,Measure);
    Ladrc_Lf(LADRC_);  
}

float Fuzzy_P(float E, float EC)
{ 
    float EFF[7] =  {-18, -11, 4, 0, 4, 11, 18};
    float DFF[7] = {-5, - 4, -3, 0, 3, 4, 5};

    int rule[7][7] = {
        {6, 5, 4, 3, 2, 1, 0},                          
        {5, 4, 3, 2, 1, 0, 1},
        {4, 3, 2, 1, 0, 1, 2},
        {3, 2, 1, 0, 1, 2, 3},
        {2, 1, 0, 1, 2, 3, 4},
        {1, 0, 1, 2, 3, 4, 5},
        {0, 1, 2, 3, 4, 5, 6},
    };
    float U = 0;                                 
    float PF[2] = {0}, DF[2] = {0}, UF[4] = {0}; 

    int Pn = 0, Dn = 0, Un[4] = {0};
    float t1 = 0, t2 = 0, t3 = 0, t4 = 0, temp1 = 0, temp2 = 0;



    if (E > EFF[0] && E < EFF[6])
    {
        if (E <= EFF[1])
        {
            Pn = -2;
            PF[0] = (EFF[1] - E) / (EFF[1] - EFF[0]);
        }
        else if (E <= EFF[2])
        {
            Pn = -1;
            PF[0] = (EFF[2] - E) / (EFF[2] - EFF[1]);
        }
        else if (E <= EFF[3])
        {
            Pn = 0;
            PF[0] = (EFF[3] - E) / (EFF[3] - EFF[2]);
        }
        else if (E <= EFF[4])
        {
            Pn = 1;
            PF[0] = (EFF[4] - E) / (EFF[4] - EFF[3]);
        }
        else if (E <= EFF[5])
        {
            Pn = 2;
            PF[0] = (EFF[5] - E) / (EFF[5] - EFF[4]);
        }
        else if (E <= EFF[6])
        {
            Pn = 3;
            PF[0] = (EFF[6] - E) / (EFF[6] - EFF[5]);
        }
    }

    else if (E <= EFF[0])
    {
        Pn = -2;
        PF[0] = 1;
    }
    else if (E >= EFF[6])
    {
        Pn = 3;
        PF[0] = 0;
    }

    PF[1] = 1 - PF[0];

    if (EC > DFF[0] && EC < DFF[6])
    {
        if (EC <= DFF[1])
        {
            Dn = -2;
            DF[0] = (DFF[1] - EC) / (DFF[1] - DFF[0]);
        }
        else if (EC <= DFF[2])
        {
            Dn = -1;
            DF[0] = (DFF[2] - EC) / (DFF[2] - DFF[1]);
        }
        else if (EC <= DFF[3])
        {
            Dn = 0;
            DF[0] = (DFF[3] - EC) / (DFF[3] - DFF[2]);
        }
        else if (EC <= DFF[4])
        {
            Dn = 1;
            DF[0] = (DFF[4] - EC) / (DFF[4] - DFF[3]);
        }
        else if (EC <= DFF[5])
        {
            Dn = 2;
            DF[0] = (DFF[5] - EC) / (DFF[5] - DFF[4]);
        }
        else if (EC <= DFF[6])
        {
            Dn = 3;
            DF[0] = (DFF[6] - EC) / (DFF[6] - DFF[5]);
        }
    }
    else if (EC <= DFF[0])
    {
        Dn = -2;
        DF[0] = 1;
    }
    else if (EC >= DFF[6])
    {
        Dn = 3;
        DF[0] = 0;
    }

    DF[1] = 1 - DF[0];


    Un[0] = rule[Pn + 2][Dn + 2];
    Un[1] = rule[Pn + 3][Dn + 2];
    Un[2] = rule[Pn + 2][Dn + 3];
    Un[3] = rule[Pn + 3][Dn + 3];

    if (PF[0] <= DF[0]) 
        UF[0] = PF[0];
    else
        UF[0] = DF[0];
    if (PF[1] <= DF[0])
        UF[1] = PF[1];
    else
        UF[1] = DF[0];
    if (PF[0] <= DF[1])
        UF[2] = PF[0];
    else
        UF[2] = DF[1];
    if (PF[1] <= DF[1])
        UF[3] = PF[1];
    else
        UF[3] = DF[1];

    if (Un[0] == Un[1])
    {
        if (UF[0] > UF[1])
            UF[1] = 0;
        else
            UF[0] = 0;
    }
    if (Un[0] == Un[2])
    {
        if (UF[0] > UF[2])
            UF[2] = 0;
        else
            UF[0] = 0;
    }
    if (Un[0] == Un[3])
    {
        if (UF[0] > UF[3])
            UF[3] = 0;
        else
            UF[0] = 0;
    }
    if (Un[1] == Un[2])
    {
        if (UF[1] > UF[2])
            UF[2] = 0;
        else
            UF[1] = 0;
    }
    if (Un[1] == Un[3])
    {
        if (UF[1] > UF[3])
            UF[3] = 0;
        else
            UF[1] = 0;
    }
    if (Un[2] == Un[3])
    {
        if (UF[2] > UF[3])
            UF[3] = 0;
        else
            UF[2] = 0;
    }
    t1 = UF[0] * UFF[Un[0]];
    t2 = UF[1] * UFF[Un[1]];
    t3 = UF[2] * UFF[Un[2]];
    t4 = UF[3] * UFF[Un[3]];
    temp1 = t1 + t2 + t3 + t4;
    temp2 = UF[0] + UF[1] + UF[2] + UF[3];
    U = temp1 / temp2;
    return U;
}


void Servo_PID(int err)
{
    static int lasterror=0;
    static int llasterror=0;
    float Kp=0;

    switch (Spid_Sel)
    {
        case 0:
            Servo_Out=err*ServoP+(err-lasterror)*ServoD;
            break;
        case 1:
            Kp =  (err * err)*0.1 / (P_Rate) + (ServoP);if(Kp>MaxP)Kp=MaxP;
            Servo_Out=err*Kp+(err-lasterror)*ServoD;
            break;
        case 2:
            Servo_Out=err*Fuzzy_P(err,err-lasterror)+(err-lasterror)*ServoD;
            break;
    }
    // Servo_Out=(int)Kp_Increase(err-lasterror,2.72)*(err*0.7+lasterror*0.2+llasterror*0.1)
                    // ;//+(err-lasterror)*Steer_d+0.1*((lasterror-llasterror)*0.3+(err-lasterror)*0.7)
    // Ladrc_Process(&LADRC,err,lasterror);
    // Servo_Out=-LADRC.u;
    lasterror=err;
    llasterror=lasterror;
    Servo_Out=Servo_Mid+Servo_Out;

    if (Servo_Out>Servo_Mid+Servo_pc){Servo_Out=Servo_Mid+Servo_pc;}
    if (Servo_Out<Servo_Mid-Servo_pc){Servo_Out=Servo_Mid-Servo_pc;} 
    
    pwm_duty(SERVO_PWM,Servo_Out);

}

