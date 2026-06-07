/*
 * inverter.c
 *
 *  Created on: 2026-06-07 17:09:02
 *  Author: TangYv
 *  Version: 1.0.0
 *  Description: 逆变器源文件
 */

#include "inverter.h"
#include "arm_math.h"
#include "tim.h"
#include "stdio.h"

#define SQRT2 1.41421356237f

float vac_p2p_ref;				// 交流电压峰峰值参考值
float vac_rms_ref;				// 交流电压有效值参考值
float iL_ref;				// 电流内环参考值

float vac_meas;				// 交流电压测量值
float iL_meas;				// 电感电流测量值
float vdc_meas;				// 直流电压测量值

float phase;				// 输出相位 (0~2π)
float phase_step;			// 相位步进值
float modulation_index;		// 调制比
float duty_A;				// A相占空比
float duty_B;				// B相占空比

PID_Struct ac_U_PI;      // 电压外环 PI
PR_Struct  ac_I_PR;      // 电流内环 PR
PID_Struct ac_V_PI;      // 单电压环 PI

/**
 * @brief  初始化逆变器控制器
 */
void Inverter_Init(void)
{
	// 电压外环 PI
	PID_Init(&ac_U_PI,
			 0.08f, 0.0025f, 0.0f,
			 0.0001f,
			 3.0f, -3.0f, 0.15f);

	// 电流内环 PR
	PR_Init(&ac_I_PR,
			0.25f,
			80.0f,
			5.0f,
			314.16f,
			0.0001f,
			60.0f, -60.0f);

	// 单电压环 PI
	PID_Init(&ac_V_PI,
			 0.35f, 0.4f, 0.0f,
			 0.0001f,
			 0.9f, -0.9f, 0.15f);

	// 初始化变量
	vac_rms_ref = 8.0f;
	vac_meas = 0.0f;
	iL_meas = 0.0f;
	iL_ref = 0.0f;
	phase = 0.0f;
	modulation_index = 0.5f;
	duty_A = 0.5f;
	duty_B = 0.5f;

	// 相位步进
	phase_step = 2.0f * PI * FREQ_OUTPUT / FS_CONTROL;
}

/**
 * @brief  设置输出电压RMS值
 */
void Inverter_SetVoltage(float vac_rms)
{
	if (vac_rms > VAC_RMS_MAX) vac_rms = VAC_RMS_MAX;
	if (vac_rms < VAC_RMS_MIN) vac_rms = VAC_RMS_MIN;

	vac_rms_ref = vac_rms;
	vac_p2p_ref = vac_rms * SQRT2;
}

/**
 * @brief  双环控制：电压PI + 电流PR
 */
void Inverter_Loop_With_ALL(float vac, float iL, float vdc)
{
	vac_meas = vac;
	iL_meas = iL;
	vdc_meas = vdc;

	phase += phase_step;
	if (phase > 2.0f * PI) {
		phase -= 2.0f * PI;
	}

	vac_p2p_ref = vac_rms_ref * SQRT2;

	// 电压外环 PI
	iL_ref = PID_Compute(&ac_U_PI, vac_p2p_ref * arm_sin_f32(phase), vac_meas);

	// 电流内环 PR
	float vac_ref = PR_Compute(&ac_I_PR, iL_ref, iL_meas);

	// 占空比计算
	float duty = vac_ref / 12.0f;

	duty_A = 0.5f * (1.0f + duty);
	duty_B = 0.5f * (1.0f - duty);

	// 限幅
	if(duty_A < 0.05f) duty_A = 0.05f;
	if(duty_A > 0.95f) duty_A = 0.95f;
	if(duty_B < 0.05f) duty_B = 0.05f;
	if(duty_B > 0.95f) duty_B = 0.95f;

	// PWM 更新
	uint16_t ccrA = (uint16_t)(duty_A * PWM_PERIOD);
	uint16_t ccrB = (uint16_t)(duty_B * PWM_PERIOD);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccrA);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccrB);
}

/**
 * @brief  单环控制：电压单PI
 */
void Inverter_Loop_With_Voltage(float vac)
{
	vac_meas = vac;

	phase += phase_step;
	if (phase > 2.0f * PI) {
		phase -= 2.0f * PI;
	}

	vac_p2p_ref = vac_rms_ref * SQRT2;

	// 单电压环 PI
	float duty = PID_Compute(&ac_V_PI, vac_p2p_ref * arm_sin_f32(phase), vac_meas);

	duty_A = 0.5f * (1.0f + duty);
	duty_B = 0.5f * (1.0f - duty);

	// 限幅
	if(duty_A < 0.05f) duty_A = 0.05f;
	if(duty_A > 0.95f) duty_A = 0.95f;
	if(duty_B < 0.05f) duty_B = 0.05f;
	if(duty_B > 0.95f) duty_B = 0.95f;

	// PWM 更新
	uint16_t ccrA = (uint16_t)(duty_A * PWM_PERIOD);
	uint16_t ccrB = (uint16_t)(duty_B * PWM_PERIOD);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccrA);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccrB);
}

/**
 * @brief  开环控制
 */
void Inverter_Loop_With_None(void)
{
	phase += 0.0314159282f;
	if (phase > 2.0f * PI) {
		phase -= 2.0f * PI;
	}

	float sin_val = arm_sin_f32(phase) * modulation_index;

	duty_A = 0.5f * (1.0f + sin_val);
	duty_B = 0.5f * (1.0f - sin_val);

	uint16_t ccrA = (uint16_t)(duty_A * PWM_PERIOD);
	uint16_t ccrB = (uint16_t)(duty_B * PWM_PERIOD);

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccrA);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccrB);
}

float Inverter_GetDutyA(void)
{
	return duty_A;
}

float Inverter_GetDutyB(void)
{
	return duty_B;
}

float Inverter_GetPhase(void)
{
	return phase;
}
