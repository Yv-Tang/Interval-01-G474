/*
 * inverter.h
 *
 *  Created on: 2026-06-07 17:09:30
 *  Author: TangYv
 *  Version: 1.0.0
 *  Description: 逆变器头文件
 */

#ifndef INC_inverter_H_
#define INC_inverter_H_

#include "controller.h"

/* 逆变器参数 */
#define VDC_NOMINAL     48.0f    	// 额定输入直流电压 48V
#define VDC_MIN         42.0f    	// 最小输入直流电压 42V
#define VAC_RMS_MAX     28.0f    	// 最大输出交流电压 28V
#define VAC_RMS_MIN     20.0f    	// 最小输出交流电压 20V
#define FREQ_OUTPUT     50.0f    	// 输出频率 50Hz
#define FS_PWM          10000.0f	// PWM频率 10kHz
#define FS_CONTROL      10000.0f	// 控制频率 10kHz
#define PWM_PERIOD		8499		// 重装值8499

/* 控制器结构体 */
extern PID_Struct ac_U_PI;      // 电压外环 PI
extern PR_Struct  ac_I_PR;      // 电流内环 PR
extern PID_Struct ac_V_PI;      // 单电压环 PI

/* 函数声明 */
void Inverter_Init(void);
void Inverter_SetVoltage(float vac_rms);

// 双环控制：电压PI + 电流PR
void Inverter_Loop_With_ALL(float vac, float iL, float vdc);

// 单环控制：电压单PI
void Inverter_Loop_With_Voltage(float vac);

// 开环控制
void Inverter_Loop_With_None(void);

float Inverter_GetDutyA(void);
float Inverter_GetDutyB(void);
float Inverter_GetPhase(void);

#endif /* INC_inverter_H_ */
