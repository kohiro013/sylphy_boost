
#include "defines.h"
#include "global.h"

#define AUTORELOAD 	(LL_TIM_GetAutoReload(TIM2) + 1)
#define FAN_DUTY	(0.5f)						// 吸引モータのDuty

/* ---------------------------------------------------------------
	吸引ファン用の動作周波数とDuty比を設定する関数
--------------------------------------------------------------- */
void SuctionFan_Initialize( void )
{
	LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH3);
	LL_TIM_EnableCounter(TIM2);
}

/* ---------------------------------------------------------------
	吸引ファンを回転させる関数
--------------------------------------------------------------- */
void SuctionFan_Start( void )
{
	LL_TIM_OC_SetCompareCH3(TIM2, (uint32_t)(AUTORELOAD * FAN_DUTY) - 1);
}

/* ---------------------------------------------------------------
	吸引ファンの回転を停止する関数
--------------------------------------------------------------- */
void SuctionFan_Stop( void )
{
	LL_TIM_OC_SetCompareCH3(TIM2, 0);
}
