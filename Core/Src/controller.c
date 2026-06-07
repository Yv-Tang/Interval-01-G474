/*
 * controller.c
 *
 *  Created on: 2026-06-07 17:05:05
 *  Author: TangYv
 *  Version: 1.0.0
 *  Description: 控制器源文件
 */

#include "controller.h"

/* 限幅宏 */
#define CONSTRAIN(val, min, max)  ((val) < (min) ? (min) : ((val) > (max) ? (max) : (val)))

/**
 * @brief  PID初始化函数
 * @brief  连续传递函数：G(s) = Kp + Ki/s + Kd·s
 * @brief  简化说明：去掉Ti、Td，直接使用Kp Ki Kd，参数更直观
 */
void PID_Init(PID_Struct *pid, float kp, float ki, float kd,
              float sample_time, float out_max, float out_min, float delta_max)
{
    // 核心控制参数
    pid->Kp = kp;    // 比例系数
    pid->Ki = ki;    // 积分系数
    pid->Kd = kd;    // 微分系数

    pid->T  = sample_time;  // 采样周期

    // 限幅保护参数
    pid->OutMax   = out_max;
    pid->OutMin   = out_min;
    pid->DeltaMax = delta_max;

    // 历史值初始化
    pid->Ek = 0;
    pid->Ek_1 = 0;
    pid->Ek_2 = 0;
    pid->Pout = 0;
    pid->Iout = 0;
    pid->Dout = 0;
    pid->OUT_Single = 0;
    pid->Out = 0;
}

/**
 * @brief  增量式PID计算
 * @brief  离散数学式：
 *         Δu(k) = Kp·(e(k)-e(k-1)) + Ki·T·e(k) + Kd/T·(e(k)-2e(k-1)+e(k-2))
 *         u(k) = u(k-1) + Δu(k)
 */
float PID_Compute(PID_Struct *pid, float setpoint, float process_value)
{
    float Sv = setpoint;   // 设定值
    float Pv = process_value; // 反馈值

    // 计算当前误差 e(k)
    pid->Ek = Sv - Pv;

    // 1. 比例项：ΔP = Kp*(e(k) - e(k-1))
    pid->Pout = pid->Kp * (pid->Ek - pid->Ek_1);

    // 2. 积分项：ΔI = Ki * T * e(k)
    pid->Iout = pid->Ki * pid->Ek * pid->T;

    // 3. 微分项：ΔD = Kd/T*(e(k) - 2e(k-1) + e(k-2))
    pid->Dout = pid->Kd * (pid->Ek - 2 * pid->Ek_1 + pid->Ek_2) / pid->T;

    // 总增量 Δu(k)
    pid->OUT_Single = pid->Pout + pid->Iout + pid->Dout;

    // 增量限幅
    pid->OUT_Single = CONSTRAIN(pid->OUT_Single, -pid->DeltaMax, pid->DeltaMax);

    // 累加得到本次输出 u(k) = u(k-1) + Δu(k)
    pid->Out += pid->OUT_Single;

    // 总输出限幅
    pid->Out = CONSTRAIN(pid->Out, pid->OutMin, pid->OutMax);

    // 更新误差历史
    pid->Ek_2 = pid->Ek_1;
    pid->Ek_1 = pid->Ek;

    return pid->Out;
}

/**
 * @brief  准PR控制器初始化
 * @brief  连续传递函数：G(s) = Kp + (2·Kr·wc·s) / (s² + 2·wc·s + wo²)
 */
void PR_Init(PR_Struct *pr, float kp, float kr, float wc, float wo, float sample_time,
             float out_max, float out_min)
{
    // PR控制参数
    pr->Kp = kp;    // 比例增益：基础响应速度
    pr->Kr = kr;    // 谐振增益：谐振点放大倍数，越大跟踪交流越强
    pr->wc = wc;    // 阻尼系数：谐振带宽，越大越稳定
    pr->wo = wo;    // 谐振角频率：跟踪目标频率 50Hz→314rad/s
    pr->T  = sample_time; // 采样周期

    // 限幅
    pr->OutMax = out_max;
    pr->OutMin = out_min;

    // 双线性变换离散系数计算
    float T = pr->T;
    float den = T*T*wo*wo + 4*wc*T + 4;

    pr->a1 = (2*T*T*wo*wo - 8) / den;
    pr->a2 = (T*T*wo*wo - 4*wc*T + 4) / den;
    pr->b0 = (kp*den + 2*kr*wc*T) / den;
    pr->b1 = (2*kp*(T*T*wo*wo - 4)) / den;
    pr->b2 = (kp*den - 2*kr*wc*T) / den;

    // 历史值清零
    pr->Ek = 0;
    pr->Ek_1 = 0;
    pr->Ek_2 = 0;
    pr->Uk = 0;
    pr->Uk_1 = 0;
    pr->Uk_2 = 0;
}

/**
 * @brief  准PR控制器计算
 * @brief  离散递推公式：
 *    u(k) = (2-a1)u(k-1) - (1+a2)u(k-2) + b0e(k)+b1e(k-1)+b2e(k-2)
 */
float PR_Compute(PR_Struct *pr, float setpoint, float feedback)
{
    // 误差 e(k)
    pr->Ek = setpoint - feedback;

    // PR离散递推计算
    pr->Uk = 2*pr->Uk_1 - pr->Uk_2
            + pr->b0 * pr->Ek
            + pr->b1 * pr->Ek_1
            + pr->b2 * pr->Ek_2
            - pr->a1 * pr->Uk_1
            - pr->a2 * pr->Uk_2;

    // 输出限幅
    pr->Uk = CONSTRAIN(pr->Uk, pr->OutMin, pr->OutMax);

    // 更新历史值
    pr->Ek_2 = pr->Ek_1;
    pr->Ek_1 = pr->Ek;
    pr->Uk_2 = pr->Uk_1;
    pr->Uk_1 = pr->Uk;

    return pr->Uk;
}
