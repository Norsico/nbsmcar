#include "SystemConfig.h"

#define Z  icm20602_gyro_z
#define Y  icm20602_gyro_y
#define PI 3.1415926
extern ele flag;
float Get_ICM(int16 icm_data,float icmerr)
{
    icm20602_get_gyro();
    icm20602_get_acc();
    
    return icm_data+icmerr; 
}

float mid_fliter(float icm_data)
{
    float i,j,k,tmp;//sample 3 times
    
    i = icm_data;//  select mid value
    j = icm_data;
    k = icm_data;
    
    if(i > j)       {tmp = i;i = j;j = tmp;}
    if(k > j)       {tmp = j;}
    else if(k > i)  {tmp = k;}
    else            {tmp = i;}
    
    return tmp;
}

float avg_fliter(float icm_data)
{
#define avg_num 3
    uint8 i=0;
    int16 sum=0;
    for ( i = 0; i < avg_num; i++)
    {
        sum+=icm_data;
    }
    sum=sum/avg_num;
    
    return sum;
}

void get_angle()
{
    icm20602_get_gyro();
    yaw     =   (icm20602_gyro_transition(icm20602_gyro_z)+0.08)/200;
    pitch   =   (icm20602_gyro_transition(icm20602_gyro_y)-1.1)/200;
    pitch_err+=-pitch;
}