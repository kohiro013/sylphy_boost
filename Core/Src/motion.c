
#include "defines.h"
#include "global.h"

// 探索時における壁切れ距離
#define OFFSET_EDGE_LEFT	(3.0f)		// 小さくするほど前に進む
#define OFFSET_EDGE_RIGHT	(3.0f)

#define SLIP_RATE			(0.95f)		// タイヤのスリップ率

volatile static float	v_adjust;
volatile static float 	cycle_const, cycle_slip;
volatile static float 	cycle_accel, amplitude_accel;
volatile static float 	cycle_slow, amplitude_slow;

/* ---------------------------------------------------------------
	直線走行用パラメータ設定関数
--------------------------------------------------------------- */
void Motion_SetStraightParameters( float acceleration, float deceleration, float max_v, float terminal_v, float distance )
{
	volatile float initial_v = ABS(Vehicle_GetVelocity());
	volatile float distance_accel;
	volatile float distance_slow;

	// 設定距離が0以下の場合の処理
	if( distance < 0 ) {
		cycle_accel = cycle_const = cycle_slow = cycle_slip = 0.f;
		amplitude_accel = amplitude_slow = 0.f;
		return;
	} else;

	// 加速・減速距離の計算
	if(initial_v > max_v) {
		max_v = initial_v;
	} else;
	distance_accel 	= (((max_v * max_v) - (initial_v * initial_v)) / (2 * acceleration));
	distance_slow	= (((max_v * max_v) - (terminal_v * terminal_v)) / (2 * deceleration));

	// 計算時間中に走行した分の距離を引く
	distance -= ABS(Vehicle_GetDistance());

	// 加減速後のスリップを吸収する区間
	v_adjust = terminal_v;
	if( ROUND(terminal_v) > 0.f ) {
		distance *= SLIP_RATE;
		cycle_slip = distance * (1.f - SLIP_RATE) / terminal_v;
	} else {
		cycle_slip = 0.f;
	}

	// 加速・減速距離の計算
	if( distance * SLIP_RATE < ( distance_accel + distance_slow ) ) {
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

	// 壁切れ
	if( Control_GetMode() == SEARCH ) {
		if( Wall_GetEdge( RIGHT ) == true ) {
			cycle_slip -= (((45.f + OFFSET_EDGE_RIGHT) - Vehicle_GetDistance()) / v_adjust);
			LED_ToggleLightBinary(0x01);
		} else if( Wall_GetEdge( LEFT ) == true ) {
			cycle_slip -= (((45.f + OFFSET_EDGE_LEFT) - Vehicle_GetDistance()) / v_adjust);
			LED_ToggleLightBinary(0x08);
		} else;
	}

	// 加速時
	if( t < cycle_accel ) {
		a = amplitude_accel * mynapier(t / cycle_accel);
	// 最高速度時
	} else if( t < cycle_accel + cycle_const ) {
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

	while( (Control_GetMode() != FAULT) && (t < cycle_accel + cycle_const + cycle_slow + cycle_slip) ) {
		t = Vehicle_GetTimer();
	}
	cycle_accel = cycle_const = cycle_slow = cycle_slip = 0.f;
	amplitude_accel = amplitude_slow = 0.f;
	Vehicle_SetAcceleration( 0.0f );
	Vehicle_ResetDistance();
	Control_ResetFilterDistance();
	Wall_ResetEdgeMinDistance();
//	Wall_ResetEdgeDistance();
}

/* ---------------------------------------------------------------
	超信地旋回の開始関数
--------------------------------------------------------------- */
void Motion_StartRotate( float degree, int8_t direction )
{
	volatile float t			= 0.0f;
	volatile float alpha;
	volatile float angle		= DEG2RAD( degree );
	volatile float amplitude	= 12.0f;
	volatile float cycle 		= ( amplitude * NAPEIR_INTGRAL ) / angle;

	// フェールセーフ
	if( Control_GetMode() > NONE ) {
		Control_SetMode( ROTATE );
	} else {
		return;
	}

	// 各パラメータのリセット
	Control_ResetFilterDistance();
	IMU_ResetGyroAngle_Z();
	Vehicle_ResetStraight();
	Vehicle_ResetTurning();
	Vehicle_ResetIntegral();
	LL_mDelay( 200 );
	Control_ResetEncoderDeviation();
	Control_ResetGyroDeviation();
	Control_ResetAngleDeviation();
	Control_ResetSensorDeviation();
	Control_ResetFrontSensorDeviation();

	// 超信地旋回
	Vehicle_ResetTimer();
	while( ( Control_GetMode() > NONE ) && ( t < 1.0f / cycle ) ) {
		// 現在時間の取得
		t = Vehicle_GetTimer();

		// 角加速度の計算
		if( t <= 0.0f )	{
			alpha = 0.0f;
		} else {
			alpha = amplitude * ( mynapier( cycle * t ) - mynapier( cycle * ( t - SYSTEM_PERIOD ) ) ) / SYSTEM_PERIOD;
		}

		// 回転方向の決定
		if( direction == LEFT ) {
			Vehicle_SetAngularAcceleration( alpha );
		} else {
			Vehicle_SetAngularAcceleration( -alpha );
		}
	}

	// 各パラメータのリセット
	LL_mDelay( 200 );
	Vehicle_ResetStraight();
	Vehicle_ResetTurning();
	Vehicle_ResetIntegral();
	Control_ResetFilterDistance();
	IMU_ResetGyroAngle_Z();
	Control_ResetEncoderDeviation();
	Control_ResetSensorDeviation();
	Control_ResetFrontSensorDeviation();
}

