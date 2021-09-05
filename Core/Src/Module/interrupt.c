
#include "defines.h"
#include "global.h"

#define PCLK			(48000000)
#define TIMER_COUNT		(LL_TIM_GetCounter(TIM5))
#define TIMER_LOAD		(LL_TIM_GetAutoReload(TIM5) + 1)
#define TIMER_PSC		(LL_TIM_GetPrescaler(TIM5) + 1)

static volatile uint32_t	global_timer = 0;
static volatile uint32_t	interrupt_count_now;
static volatile int32_t		interrupt_duty;
static volatile int32_t		interrupt_duty_max = 0;
static volatile float		boot_time = 0.f;


/* ---------------------------------------------------------------
	1ms周期で割り込み処理関数
--------------------------------------------------------------- */
void Interrupt_Main( void )
{
	// ジャイロと加速度の値の更新
	IMU_Update();

	// エンコーダの値の更新
	Encoder_Update();

	// 壁センサ情報の更新
	Wall_Update();

	if( Control_GetMode() > NONE ) {
		// 車両運動計算
		Vehicle_UpdateDynamics();

		// ログ
		Log_WriteRecodeData();
	} else;

	// スイッチ反応中の時間の更新
	Switch_UpdateTimer();

	// LED点灯の時間の更新
	LED_UpdateTimer();
}

/* ---------------------------------------------------------------
	絶対時間を取得する関数
--------------------------------------------------------------- */
uint32_t Interrupt_GetGlobalTime( void )
{
	return global_timer;
}

/* ---------------------------------------------------------------
	指定ms待機する関数
--------------------------------------------------------------- */
void Interrupt_Wait1ms( uint32_t ms )
{
	volatile uint32_t tmp_timer;

	tmp_timer = global_timer;
	while(global_timer - tmp_timer < ms);
}

/* ---------------------------------------------------------------
	メイン割り込みの初期設定関数
--------------------------------------------------------------- */
void Interrupt_Initialize( void )
{
	LL_TIM_EnableIT_UPDATE(TIM5);
	LL_TIM_EnableCounter(TIM5);
}

/* ---------------------------------------------------------------
	割り込み前処理関数
--------------------------------------------------------------- */
void Interrupt_PreProcess( void )
{
	global_timer ++;
	interrupt_count_now = TIMER_COUNT;
	boot_time = (float)global_timer * SYSTEM_PERIOD;
}

/* ---------------------------------------------------------------
	割り込み後処理関数
--------------------------------------------------------------- */
void Interrupt_PostProcess( void )
{
	interrupt_duty = (uint16_t)(MIN(TIMER_COUNT - interrupt_count_now,
									TIMER_COUNT - interrupt_count_now + TIMER_LOAD)) * 1000 / TIMER_LOAD;
	interrupt_duty_max = MAX( interrupt_duty_max, interrupt_duty );
}

/* ---------------------------------------------------------------
	割り込み周期に占める呼び出し位置までの処理時間の割合を取得する関数
--------------------------------------------------------------- */
int32_t Interrupt_GetDuty( void )
{
	return interrupt_duty;
}

/* ---------------------------------------------------------------
	上記割合の最大値を取得する関数
--------------------------------------------------------------- */
int32_t Interrupt_GetDuty_Max( void )
{
	return interrupt_duty_max;
}

/* ---------------------------------------------------------------
	マイコン起動時からの経過時間を取得する関数
--------------------------------------------------------------- */
float Interrupt_GetBootTime( void )
{
	return boot_time;
}
