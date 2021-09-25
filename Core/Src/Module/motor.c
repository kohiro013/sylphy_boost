
#include "defines.h"
#include "global.h"

#define PCLK				(48000000)
#define AUTORELOAD 		 	(LL_TIM_GetAutoReload(TIM4) + 1)

const uint16_t MOT_DUTY_MIN = 10;		// モータの最低Duty
const uint16_t MOT_DUTY_MAX = 950;		// モータの最大Duty

// モータの向き設定
#define MOT_SET_COMPARE_L_FORWARD(x)	LL_TIM_OC_SetCompareCH4(TIM4, x)
#define MOT_SET_COMPARE_L_REVERSE(x)	LL_TIM_OC_SetCompareCH3(TIM4, x)
#define MOT_SET_COMPARE_R_FORWARD(x)	LL_TIM_OC_SetCompareCH2(TIM4, x)
#define MOT_SET_COMPARE_R_REVERSE(x)	LL_TIM_OC_SetCompareCH1(TIM4, x)


/* ---------------------------------------------------------------
	モータ用のタイマーを開始する関数
--------------------------------------------------------------- */
void Motor_Initialize( void )
{
	LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH1);
	LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH2);
	LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH3);
	LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH4);
	LL_TIM_EnableCounter(TIM4);
}

/* ---------------------------------------------------------------
	モータの動作周波数を設定する関数
--------------------------------------------------------------- */
void Motor_SetFrequency( uint16_t khz )
{
	Motor_StopPWM();
	LL_TIM_SetAutoReload(TIM4, (PCLK / (khz * 1000)) - 1);
}

/* ---------------------------------------------------------------
	モータのの回転を止める関数
--------------------------------------------------------------- */
void Motor_StopPWM( void )
{
	MOT_SET_COMPARE_R_FORWARD( 0xffff );
	MOT_SET_COMPARE_R_REVERSE( 0xffff );
	MOT_SET_COMPARE_L_FORWARD( 0xffff );
	MOT_SET_COMPARE_L_REVERSE( 0xffff );
}

/* ---------------------------------------------------------------
	右モータを指定のDuty（0～1000）で回転させる関数
--------------------------------------------------------------- */
void Motor_SetDuty_Right( int16_t duty_r )
{
	uint32_t	pulse_r;

	if( ABS(duty_r) > MOT_DUTY_MAX ) {
		pulse_r = (uint32_t)(AUTORELOAD * MOT_DUTY_MAX / 1000) - 1;
	} else if( ABS(duty_r) < MOT_DUTY_MIN ) {
		pulse_r = (uint32_t)(AUTORELOAD * MOT_DUTY_MIN / 1000) - 1;
	} else {
		pulse_r = (uint32_t)(AUTORELOAD * ABS(duty_r) / 1000) - 1;
	}

	if( duty_r > 0 ) {
		MOT_SET_COMPARE_R_FORWARD( pulse_r );
		MOT_SET_COMPARE_R_REVERSE( 0 );
	} else if( duty_r < 0 ) {
		MOT_SET_COMPARE_R_FORWARD( 0 );
		MOT_SET_COMPARE_R_REVERSE( pulse_r );
	} else {
		MOT_SET_COMPARE_R_FORWARD( 0 );
		MOT_SET_COMPARE_R_REVERSE( 0 );
	}
}

/* ---------------------------------------------------------------
	左モータを指定のDuty（0～1000）で回転させる関数
--------------------------------------------------------------- */
void Motor_SetDuty_Left( int16_t duty_l )
{
	uint32_t	pulse_l;

	if( ABS(duty_l) > MOT_DUTY_MAX ) {
		pulse_l = (uint32_t)(AUTORELOAD * MOT_DUTY_MAX / 1000) - 1;
	} else if( ABS(duty_l) < MOT_DUTY_MIN ) {
		pulse_l = (uint32_t)(AUTORELOAD * MOT_DUTY_MIN / 1000) - 1;
	} else {
		pulse_l = (uint32_t)(AUTORELOAD * ABS(duty_l) / 1000) - 1;
	}

	if( duty_l > 0 ) {
		MOT_SET_COMPARE_L_FORWARD( pulse_l );
		MOT_SET_COMPARE_L_REVERSE( 0 );
	} else if( duty_l < 0 ) {
		MOT_SET_COMPARE_L_FORWARD( 0 );
		MOT_SET_COMPARE_L_REVERSE( pulse_l );
	} else {
		MOT_SET_COMPARE_L_FORWARD( 0 );
		MOT_SET_COMPARE_L_REVERSE( 0 );
	}
}
