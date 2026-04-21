/*
 * ackerman.c - 阿克曼几何学运动学算法实现
 *
 * 提供纯运动学解算，不包含PID闭环控制
 * 参考 SmartCCar_CCD_open1 的差速计算实现
 *
 * [2026-04-20] 浮点转整型版本
 *   角度单位: 0.01度 (int16, 1 = 0.01°)
 *   速度单位: 0.01m/s (int16, 1 = 0.01m/s)
 */

#include "ackerman.h"

/************ tan(δ) 查表 (0° ~ 30°, 0.5度分辨率) ************/
/* 范围 0 ~ 3000 (对应 0° ~ 30°)，步进值50 */
/* 值: tan(angle°) * 100 */
/* int16 范围 -32768~32767 */
/* 负角度使用奇函数特性: tan(-x) = -tan(x) */
static const int16 tan_table[61] = {
    0,10,20,30,30,40,50,60,70,80,90, // 0 ~ 500
    100,110,110,120,130,140,150,160,170,180, // 550 ~ 1000
    190,190,200,210,220,230,240,250,260,270, // 1050 ~ 1500
    280,290,300,310,320,320,330,340,350,360, // 1550 ~ 2000
    370,380,390,400,410,420,440,450,460,470, // 2050 ~ 2500
    480,490,500,510,520,530,540,550,570,580 // 2550 ~ 3000
};

/************ 全局状态 ************/
static ackerman_kinematic_t ackerman_kinematic = {0};

/**
 * @brief 初始化阿克曼运动学状态
 */
void ackerman_init(void)
{
    ackerman_kinematic.steer_angle = 0;
    ackerman_kinematic.speed = 0;
    ackerman_kinematic.left_wheel_speed = 0;
    ackerman_kinematic.right_wheel_speed = 0;
}

/**
 * @brief 获取tan值 查表+线性运算
 * @param angle 角度 (0.01度)
 * @note 逻辑演示：
    假设你要查 10.23°的 tan 值：
    查表：找到表中10.0° (0.18) 和 10.5°(0.19)。
    计算：10.23 位于 10.0 和 10.5 之间，大约 46% 的位置。
    插值：
    0.18+(0.19−0.18)×0.46=0.1846
    结果：保留两位小数为 0.18。
    真实值 tan(10.23∘)≈0.1805结果一致。
 * @return tan(angle) * 1000
 */
static int16 ackerman_tan(int16 angle)
{
    uint8 idx;
    int16 tan_res; // 最大不会超过580

    // 限制范围 -3000 ~ +3000 (即 -30° ~ +30°)
    if (angle >= 3000) {
        return 580;
    } else if (angle <= -3000) {
        return -580;
    }

    tan_res = 1;
    if(angle==0) return 0;
    else if(angle<0){
        angle = -angle;
        tan_res = -1;
    } 
    
    // 循环找位置
    for(idx = 0;idx<60;idx++){
        if(angle<idx*50) break; // 3.2° 320 < 50*7 3.5°
        // idx 不会为0
    }
    //  (angle-(idx-1)*50) / 50 代表比例  
    tan_res = tan_res * (tan_table[idx-1] + (angle-(idx-1)*50) * (tan_table[idx]-tan_table[idx-1]) / 50);

    return  tan_res;

}

/**
 * @brief 设置转向角
 */
void ackerman_set_steer_angle(int16 steer_angle)
{
    // 限制转向角范围
    if (steer_angle > CAR_MAX_STEER_ANGLE) {
        steer_angle = CAR_MAX_STEER_ANGLE;
    } else if (steer_angle < -CAR_MAX_STEER_ANGLE) {
        steer_angle = -CAR_MAX_STEER_ANGLE;
    }

    ackerman_kinematic.steer_angle = steer_angle;
}

/**
 * @brief 计算左右轮速度
 */
void ackerman_calc_wheel_speeds(int16 speed, int16 steer_angle)
{
    int16 v_dif;
    int16 tan_val;

    ackerman_set_steer_angle(steer_angle);
    steer_angle = ackerman_kinematic.steer_angle;

    // 保存当前速度
    ackerman_kinematic.speed = speed;

    // 阿克曼差速因子: v_dif = (tread_width / wheelbase) * tan(δ)
    // tan_val = tan(δ) * 1000
    // v_dif = 809 * tan_val / 1000 (809 = 0.8088 * 1000)
    // 结果 v_dif 为速度缩放值 (无单位)
    tan_val = ackerman_tan(steer_angle);

    // 使 v_dif必须使用int32暂存
    // v_dif = ACKerman_K * tan_val  max ： 809 * 580 = 469220 
    // 预期 v_dif在 int16内，4位有效数字 除100 max ： 469220/100 = 4692
    v_dif = ((int32)ACKerman_K * tan_val) / 100;  

    /* 右转时左轮走外侧，左转时右轮走外侧。 */
    /* 外侧轮保持基础速度，内侧轮按国一口径减速。 */
    if (steer_angle > 0)
    {
        ackerman_kinematic.left_wheel_speed = speed;
        // right = speed * (1 - v_dif/10000) = speed - speed*v_dif/10000
        ackerman_kinematic.right_wheel_speed = speed - (int16)(((int32)speed * v_dif) / 10000);
    }
    else if (steer_angle < 0)
    {
        ackerman_kinematic.right_wheel_speed = speed;
        // left = speed * (1 + v_dif/10000) = speed + speed*v_dif/10000
        ackerman_kinematic.left_wheel_speed = speed + (int16)(((int32)speed * v_dif) / 10000);
    }
    else
    {
        ackerman_kinematic.left_wheel_speed = speed;
        ackerman_kinematic.right_wheel_speed = speed;
    }
}

/**
 * @brief 获取左轮速度
 */
int16 ackerman_get_left_speed(void)
{
    return ackerman_kinematic.left_wheel_speed;
}

/**
 * @brief 获取右轮速度
 */
int16 ackerman_get_right_speed(void)
{
    return ackerman_kinematic.right_wheel_speed;
}

/**
 * @brief 获取当前转向角
 */
int16 ackerman_get_steer_angle(void)
{
    return ackerman_kinematic.steer_angle;
}

/**
 * @brief 获取当前车速
 */
int16 ackerman_get_speed(void)
{
    return ackerman_kinematic.speed;
}

/**
 * @brief 获取阿克曼运动学状态
 */
const ackerman_kinematic_t* ackerman_get_state(void)
{
    return &ackerman_kinematic;
}
