/*
 * controller.h
 *
 *  Created on: 2026-06-07 17:04:34
 *  Author: TangYv
 *  Version: 1.0.0
 *  Description: 控制器
 */

#ifndef INC_controller_H_
#define INC_controller_H_

typedef struct
{
    // PID 核心参数
    float Kp;     // 比例系数：加快响应速度，减小稳态误差
    float Ki;     // 积分系数：消除静差，提高系统无差度
    float Kd;     // 微分系数：抑制振荡，超前调节，提高稳定性
    float T;      // 采样周期（单位：秒）

    // 历史误差缓存
    float Ek;     // 当前误差 e(k)
    float Ek_1;   // 上一时刻误差 e(k-1)
    float Ek_2;   // 上两时刻误差 e(k-2)

    // 分项输出
    float Pout;   // 比例项输出
    float Iout;   // 积分项输出
    float Dout;   // 微分项输出

    // 输出控制
    float OUT_Single; // 本次输出增量
    float Out;        // 总输出

    // 限幅保护
    float OutMax;   // 输出上限
    float OutMin;   // 输出下限
    float DeltaMax; // 单次增量限幅（防抖动）
} PID_Struct;

typedef struct
{
    // 准PR 核心参数
    float Kp;    // 比例增益：全频段基础增益
    float Kr;    // 谐振增益：谐振频率点放大倍数（跟踪交流信号核心）
    float wc;    // 阻尼带宽：决定谐振峰宽度，越大越稳定
    float wo;    // 谐振角频率：要跟踪的信号频率（50Hz=314rad/s）
    float T;     // 采样周期（单位：秒）

    // 离散化系数（自动计算，无需修改）
    float a1, a2;
    float b0, b1, b2;

    // 误差与输出历史
    float Ek;     // 当前误差 e(k)
    float Ek_1;   // 上一时刻误差 e(k-1)
    float Ek_2;   // 上两时刻误差 e(k-2)
    float Uk;     // 当前输出 u(k)
    float Uk_1;   // 上一时刻输出 u(k-1)
    float Uk_2;   // 上两时刻输出 u(k-2)

    // 输出限幅
    float OutMax;
    float OutMin;
} PR_Struct;

// PID 函数
void PID_Init(PID_Struct *pid, float kp, float ki, float kd,
              float sample_time, float out_max, float out_min, float delta_max);

float PID_Compute(PID_Struct *pid, float setpoint, float process_value);

// PR 函数
void PR_Init(PR_Struct *pr, float kp, float kr, float wc, float wo, float sample_time,
             float out_max, float out_min);

float PR_Compute(PR_Struct *pr, float setpoint, float feedback);

#endif /* INC_controller_H_ */
