
#include "defines.h"
#include "global.h"

// 探索時における壁切れ距離
#define OFFSET_EDGE_LEFT	(3.0f)		// 小さくするほど前に進む
#define OFFSET_EDGE_RIGHT	(4.0f)

volatile static float 	cycle_const;
volatile static float 	cycle_accel, amplitude_accel;
volatile static float 	cycle_slow, amplitude_slow;

/* ---------------------------------------------------------------
	直線走行用パラメータ設定関数
--------------------------------------------------------------- */
void Motion_SetStraightParameters( float acceleration, float deceleration, float max_v, float terminal_v, float distance )
{
	float initial_v = ABS(Vehicle_GetVelocity());
	float distance_accel;
	float distance_slow;

	// 設定距離が0以下の場合の処理
	if( distance < 0 ) {
		cycle_accel = cycle_const = cycle_slow = 0.f;
		amplitude_accel = amplitude_slow = 0.f;
		return;
	} else;

	// 計算時間中に走行した分の距離を引く
	distance -= ABS(Vehicle_GetDistance());

	// 加速・減速距離の計算
	if( initial_v > max_v ) {
		max_v = initial_v;
	} else;
	distance_accel = (((max_v * max_v) - (initial_v * initial_v)) / (2 * acceleration));
	distance_slow  = (((max_v * max_v) - (terminal_v * terminal_v)) / (2 * deceleration));

	// 加速・減速時間の計算
	if( distance < ( distance_accel + distance_slow ) ) {
		cycle_const = 0.0f;
		arm_sqrt_f32(((acceleration * terminal_v * terminal_v) + (deceleration * initial_v * initial_v)
				+ (2 * acceleration * deceleration * distance)) / (acceleration + deceleration), &max_v);
		distance_accel = (((max_v * max_v) - (initial_v * initial_v)) / (2 * acceleration));
		distance_slow = (((max_v * max_v) - (terminal_v * terminal_v)) / (2 * deceleration));
	} else {
		cycle_const	= (distance - distance_accel - distance_slow) / max_v;
	}

	// 加減速用ネイピア関数の周期と振幅の計算
	cycle_accel	= ( max_v - initial_v ) / acceleration;
	if( cycle_accel <= 0 ) amplitude_accel = 0.0f;
	else amplitude_accel = ( max_v - initial_v ) / ( NAPEIR_INTGRAL * cycle_accel );

	cycle_slow	= ( max_v - terminal_v ) / deceleration;
	if( cycle_slow <= 0 ) amplitude_slow = 0.0f;
	else amplitude_slow = ( max_v - terminal_v ) / ( NAPEIR_INTGRAL * cycle_slow );
}

/* ---------------------------------------------------------------
	直線走行の開始関数
--------------------------------------------------------------- */
void Motion_StartStraight( float acceleration, float deceleration, float max_v, float terminal_v, float distance )
{
	// 直線走行パラメータの設定
	Motion_SetStraightParameters( acceleration, deceleration, max_v, terminal_v, distance );

	// タイマーのリセット
	Vehicle_ResetTimer();
}

/* ---------------------------------------------------------------
	直線走行時間の取得関数
--------------------------------------------------------------- */
float Motion_GetStraightTime( float acceleration, float deceleration, float max_v, float terminal_v, float distance )
{
	Motion_SetStraightParameters( acceleration, deceleration, max_v, terminal_v, distance );
	return cycle_accel + cycle_const + cycle_slow;
}

/* ---------------------------------------------------------------
	直線の加速度出力関数
--------------------------------------------------------------- */
float Motion_SetStraightAcceleration( float t )
{
	volatile float a = 0.f;

	// 加速時
	if( t < cycle_accel ) {
		a = amplitude_accel * mynapier(t / cycle_accel);
	// 最高速度時
	} else if( t < cycle_accel + cycle_const ) {
		// 壁切れ
		if( Control_GetMode() == SEARCH ) {
			if( Wall_GetEdge( RIGHT ) == true ) {
				cycle_const -= (((45.f + OFFSET_EDGE_RIGHT) - Vehicle_GetDistance()) / Motion_GetSlalomVelocity(turn_90, 0));
				LED_ToggleLightBinary(0x01);
			} else if( Wall_GetEdge( LEFT ) == true ) {
				cycle_const -= (((45.f + OFFSET_EDGE_LEFT) - Vehicle_GetDistance()) / Motion_GetSlalomVelocity(turn_90, 0));
				LED_ToggleLightBinary(0x08);
			} else;
		} else;

		a = 0.f;
	// 減速時
	} else if( t < cycle_accel + cycle_const + cycle_slow ) {
		a = -amplitude_slow * mynapier((t - cycle_accel - cycle_const) / cycle_slow);
	// スリップ調整区間
	} else {
		a = 0.f;
	}
	return a;
}

/* ---------------------------------------------------------------
	直線走行中の待機関数
--------------------------------------------------------------- */
void Motion_WaitStraight( void )
{
	volatile float t = 0.f;

	while( (Control_GetMode() != FAULT) && (t < cycle_accel + cycle_const + cycle_slow) ) {
		t = Vehicle_GetTimer();
	}
	cycle_accel = cycle_const = cycle_slow = 0.f;
	amplitude_accel = amplitude_slow = 0.f;
	Vehicle_SetAcceleration( 0.0f );
	Vehicle_ResetDistance();
	Control_ResetFilterDistance();
	Wall_ResetEdgeMinDistance();
}

