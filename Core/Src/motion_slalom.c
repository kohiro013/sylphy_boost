
#include "defines.h"
#include "global.h"
#include "slalom.h"

#define EDGE_WALL_SL	( 1.0f)	// 直線時における左の壁切れ距離
#define EDGE_WALL_SR	(-0.0f)	// 直線時における右の壁切れ距離
#define EDGE_PILLAR_SL	(-3.0f)	// 直線時における左の壁切れ距離
#define EDGE_PILLAR_SR	(-4.0f)	// 直線時における右の壁切れ距離

#define EDGE_WALL_DL	(3.0f)	// 斜め時における左の壁切れ距離
#define EDGE_WALL_DR	(4.0f)	// 斜め時における右の壁切れ距離
#define EDGE_PILLAR_DL	(3.0f)	// 斜め時における左の壁切れ距離
#define EDGE_PILLAR_DR	(4.0f)	// 斜め時における右の壁切れ距離

#define TH_PILLAR_STRAIGHT	(50.f)	// 壁切れ距離の柱判定
#define TH_PILLAR_DIAGONAL	(35.f)	// 壁切れ距離の柱判定
#define REF_WALL_FRONT		(87.f)	// 前センサの距離リファレンス

volatile static float		fastest_a;
volatile static float		turn_v;
volatile static float 		cycle_slalom;
volatile static float		amplitude_slalom;
volatile static float		before_distance;
volatile static float		after_distance;


/* ---------------------------------------------------------------
	スラローム走行の開始関数
--------------------------------------------------------------- */
void Motion_StartSlalom( int8_t type, int8_t direction, int8_t param )
{
	volatile float	angle_slalom;
	volatile float 	radius;

	if( type == turn_0 || type == turn_90 ) param = 0;

	angle_slalom			= DEG2RAD( init_slalom[type-1][param].degree );
	turn_v 					= init_slalom[type-1][param].velocity;
	radius					= init_slalom[type-1][param].radius;

	if( direction == RIGHT ) {
		amplitude_slalom 	= -turn_v / radius;
		before_distance		= init_slalom[type-1][param].before;
		after_distance		= init_slalom[type-1][param].after;
	} else {
		amplitude_slalom 	= turn_v / radius;
		before_distance		= init_slalom[type-1][param].before;
		after_distance		= init_slalom[type-1][param].after;
	}
	cycle_slalom = ( ABS(amplitude_slalom) * NAPEIR_INTGRAL ) / angle_slalom;

	if( type == turn_0 || type == turn_90 ) {
		Vehicle_ResetTurning();
		Vehicle_ResetIntegral();
		Control_ResetFilterDistance();
		Control_ResetSensorDeviation();
		IMU_ResetGyroAngle_Z();
		Control_ResetAngleDeviation();
		//Control_ResetGyroDeviation();
	} else;
	Vehicle_ResetTimer();
}

/* ---------------------------------------------------------------
	スラロームの角加速度出力関数
--------------------------------------------------------------- */
float Motion_SetSlalomAngularAcceleration( float t )
{
	volatile float alpha = 0.f;

	if( t <= before_distance/turn_v ) {
		alpha = 0.f;
	} else if( t < before_distance/turn_v + 1.f/cycle_slalom ) {
		if( t - before_distance/turn_v <= SYSTEM_PERIOD ) {
			alpha = 0.f;
		} else {
			alpha = amplitude_slalom * ( mynapier( cycle_slalom * (t - before_distance/turn_v) )
					- mynapier( cycle_slalom * ( (t - before_distance/turn_v) - SYSTEM_PERIOD ) ) ) / SYSTEM_PERIOD;
		}
	} else {
		alpha = 0.0f;
	}
	return alpha;
}

/* ---------------------------------------------------------------
	スラロームの加速度
--------------------------------------------------------------- */
void Motion_SetSlalomAcceleration( float a )
{
	fastest_a = a;
}

void Motion_SlalomAcceleration( float a )
{
	float acceleration = MIN(a, ABS(Vehicle_GetVelocity() - turn_v) / SYSTEM_PERIOD);

	if( Vehicle_GetVelocity() < turn_v ) {
		Vehicle_SetAcceleration( acceleration );
	} else if( Vehicle_GetVelocity() > turn_v ) {
		Vehicle_SetAcceleration( -acceleration );
	} else {
		Vehicle_SetAcceleration( 0.f );
	}
}

/* ---------------------------------------------------------------
	スラローム前の壁切れ補正
--------------------------------------------------------------- */
void Motion_CorrectWallEdge( int8_t type, int8_t direction )
{
	float distance_left  = Wall_GetDistance(LEFT);
	float distance_right = Wall_GetDistance(RIGHT);

	while( Control_GetMode() != FAULT ) {
		if( direction == RIGHT && Wall_GetEdge(RIGHT) == true ) {
			// 直進の場合
			if( type <= turn_135in ) {
				if( distance_right < TH_PILLAR_STRAIGHT ) {
					LED_ToggleLightBinary(0x01);
					before_distance += EDGE_WALL_SR;
				} else {
					LED_ToggleLightBinary(0x02);
					before_distance += EDGE_PILLAR_SR;
				}
			// 斜めの場合
			} else {
				if( distance_right < TH_PILLAR_DIAGONAL ) {
					LED_ToggleLightBinary(0x01);
					before_distance += EDGE_WALL_DR;
				} else {
					LED_ToggleLightBinary(0x02);
					before_distance += EDGE_PILLAR_DR;
				}
			}
			break;
		} else if( direction == RIGHT && Wall_GetEdge(LEFT) == true ) {
			// 直進の場合
			if( type <= turn_135in ) {
				if( distance_left < TH_PILLAR_STRAIGHT ) {
					LED_ToggleLightBinary(0x08);
					before_distance += EDGE_WALL_SL;
				} else {
					LED_ToggleLightBinary(0x04);
					before_distance += EDGE_PILLAR_SL;
				}
			} else;
			break;

		} else if( direction == LEFT && Wall_GetEdge(LEFT) == true ) {
			// 直進の場合
			if( type <= turn_135in ) {
				if( distance_left < TH_PILLAR_STRAIGHT ) {
					LED_ToggleLightBinary(0x08);
					before_distance += EDGE_WALL_SL;
				} else {
					LED_ToggleLightBinary(0x04);
					before_distance += EDGE_PILLAR_SL;
				}
			// 斜めの場合
			} else {
				if( distance_left < TH_PILLAR_DIAGONAL ) {
					LED_ToggleLightBinary(0x08);
					before_distance += EDGE_WALL_DL;
				} else {
					LED_ToggleLightBinary(0x04);
					before_distance += EDGE_PILLAR_DL;
				}
			}
			break;
		} else if( direction == LEFT && Wall_GetEdge(RIGHT) == true ) {
			// 直進の場合
			if( type <= turn_135in ) {
				if( distance_right < TH_PILLAR_STRAIGHT ) {
					LED_ToggleLightBinary(0x01);
					before_distance += EDGE_WALL_SR;
				} else {
					LED_ToggleLightBinary(0x02);
					before_distance += EDGE_PILLAR_SR;
				}
			} else;
			break;

		} else {
			Vehicle_ResetTimer();
			Motion_SlalomAcceleration(fastest_a);
		}
	}
	before_distance = MAX(0.f, before_distance);	// 前距離が0以下にならないようにする
}

/* ---------------------------------------------------------------
	スラローム走行中の待機関数
--------------------------------------------------------------- */
void Motion_WaitSlalom( int8_t type, int8_t direction, int8_t param )
{
	volatile float 	t = 0.f;
	// 計算時間中に走行した分の距離を引く
	volatile float 	offset_distance = ABS(Vehicle_GetDistance());
	volatile float	acceleration = fastest_a;

	// 探索調整用スラローム
	if( type == turn_0 ) {
		Control_SetMode( TURN );
		Vehicle_SetAcceleration( 0.0f );
		Vehicle_SetVelocity( turn_v );
		Vehicle_ResetTimer();
		while( (Control_GetMode() != FAULT) && (t < (before_distance + after_distance - offset_distance)/turn_v + 1.f/cycle_slalom) ) {
			t = Vehicle_GetTimer();
		}

	// 探索用スラローム
	} else if( type == turn_90 ) {
		Vehicle_SetAcceleration( 0.0f );
		Vehicle_SetVelocity( turn_v );

		// 1区画前なるまで前進
		if( Wall_GetIsMaze(FRONT + RIGHT) == true && Wall_GetIsMaze(FRONT + LEFT) == true ) {
			while( Control_GetMode() != FAULT && (Wall_GetDistance(FRONT + LEFT) > REF_WALL_FRONT || Wall_GetDistance(FRONT + RIGHT) > REF_WALL_FRONT) ) {
				Vehicle_ResetTimer();
				offset_distance = 0.f;
			}
		} else;

		// スラローム前の左右壁で後距離を補正する
		if( direction == RIGHT && Wall_GetIsMaze(LEFT) == true ) {
//			after_distance -= (Wall_GetDistance(LEFT) - 42.f);
		} else if( direction == LEFT && Wall_GetIsMaze(RIGHT) == true ) {
//			after_distance -= (Wall_GetDistance(RIGHT) - 42.f);
		} else;
		
		// スラローム
		Control_SetMode( ROTATE );
		Vehicle_SetTimer( MIN(offset_distance, before_distance)/turn_v );
		while( (Control_GetMode() != FAULT) && (t < (before_distance + after_distance)/turn_v + 1.f/cycle_slalom) ) {
			t = Vehicle_GetTimer();
			if( t < before_distance/turn_v ) {
				// 前距離で進みすぎるのを防止する
				if( Wall_GetDistance(FRONT + LEFT) < REF_WALL_FRONT - before_distance && Wall_GetDistance(FRONT + RIGHT) < REF_WALL_FRONT - before_distance ) {
					Vehicle_SetTimer( before_distance/turn_v );
				} else;

				// 1区画前なるまで前進
				if( Wall_GetIsMaze(FRONT + RIGHT) == true && Wall_GetIsMaze(FRONT + LEFT) == true ) {
					while( Control_GetMode() != FAULT
							&& (Wall_GetDistance(FRONT + LEFT) > REF_WALL_FRONT - before_distance
							|| Wall_GetDistance(FRONT + RIGHT) > REF_WALL_FRONT - before_distance) ) {
						Vehicle_SetTimer( before_distance/turn_v - SYSTEM_PERIOD );
					}
				} else;
			} else if( t > before_distance/turn_v + 1.f/cycle_slalom ) {
				// 後距離で進みすぎるのを防止する
				if( Wall_GetDistance(FRONT + LEFT) < REF_WALL_FRONT && Wall_GetDistance(FRONT + RIGHT) < REF_WALL_FRONT ) {
					break;
				} else;
			} else;
		}

		// 1区画前なるまで前進
		if( Wall_GetIsMaze(FRONT + RIGHT) == true && Wall_GetIsMaze(FRONT + LEFT) == true ) {
			while( Control_GetMode() != FAULT && (Wall_GetDistance(FRONT + LEFT) >= REF_WALL_FRONT || Wall_GetDistance(FRONT + RIGHT) >= REF_WALL_FRONT) ) {}
		} else;

		// 各パラメータのリセット
		Vehicle_ResetAngle();
		IMU_ResetGyroAngle_Z();
		Control_ResetAngleDeviation();
		Control_ResetSensorDeviation();

	// 通常時のターン
	} else {
		// 斜め脱出時に壁制御で姿勢が崩れないように斜めの壁制御を切る
//		if( type >= turn_90v && type <= turn_135out ) {
			Control_SetMode(TURN);
//		} else;

		// 壁切れまで直進
		Wall_ResetEdgeMinDistance();
		Motion_CorrectWallEdge(type, direction);

		// 最短走行で設定した加速度で加速しきれない場合に加速度を増加させる
		float before_velocity = Vehicle_GetVelocity();
		if( fastest_a > ABS(turn_v * turn_v - before_velocity * before_velocity) / (2*before_distance) ) {
			acceleration = fastest_a;
		} else {
			acceleration = ABS(turn_v * turn_v - before_velocity * before_velocity) / (2*before_distance);
		}
		
		// 各パラメータのリセット
		Vehicle_ResetTurning();
		Vehicle_ResetDistance();
		Control_ResetFilterDistance();
		Control_ResetSensorDeviation();
		Control_SetMode( TURN );

		// 前距離
		while( Control_GetMode() != FAULT && Vehicle_GetDistance() < before_distance ) {
			Vehicle_ResetTimer();
			Motion_SlalomAcceleration(acceleration);
			Wall_ResetEdgeMinDistance();
		}
		Vehicle_SetAcceleration( 0.f );
		Vehicle_SetVelocity( turn_v );
		Vehicle_ResetDistance();
		Control_ResetFilterDistance();

		// スラローム
		Vehicle_SetTimer(before_distance/turn_v);
		while( Control_GetMode() != FAULT && t < before_distance/turn_v + 1.f/cycle_slalom ) {
			t = Vehicle_GetTimer();
			Wall_ResetEdgeMinDistance();
		}
	}

	// 各パラメータのリセット
	Vehicle_ResetTurning();
	Vehicle_ResetDistance();
	Control_ResetFilterDistance();
	cycle_slalom = amplitude_slalom = 0.f;
	before_distance = after_distance = 0.f;
	Wall_ResetEdgeDistance();
	Wall_ResetEdgeMinDistance();
}

/* ----------------------------------------------------------------------------------
	スラロームの時間
-----------------------------------------------------------------------------------*/
float Motion_GetSlalomTime( int8_t type, int8_t param )
{
	if( type == turn_0 || type == goal ) {
		return 0.0f;
	} else {
		return ( init_slalom[type-1][param].time );
	}
}

/* ----------------------------------------------------------------------------------
	スラロームの速度
-----------------------------------------------------------------------------------*/
float Motion_GetSlalomVelocity( int8_t type, int8_t param )
{
	if( type == turn_0 || type == goal ) {
		return( 0 );
	} else {
		return( init_slalom[type-1][param].velocity );
	}
}

/* ----------------------------------------------------------------------------------
	スラロームの前距離
-----------------------------------------------------------------------------------*/
float Motion_GetSlalomBeforeDistance( int8_t type, int8_t direction, int8_t param )
{
	if( type == turn_0 || type == goal ) {
		return( 0 );
	} else {
		return( init_slalom[type-1][param].before );
	}
}

/* ----------------------------------------------------------------------------------
	スラロームの後距離
-----------------------------------------------------------------------------------*/
float Motion_GetSlalomAfterDistance( int8_t type, int8_t direction, int8_t param )
{
	if( type == turn_0 || type == goal ) {
		return( 0 );
	} else {
		return( init_slalom[type-1][param].after );
	}
}

/* ---------------------------------------------------------------
	超信地旋回の開始関数
--------------------------------------------------------------- */
void Motion_StartRotate( float degree, int8_t direction )
{
	volatile float t		= 0.0f;
	volatile float angle	= DEG2RAD(degree);

	amplitude_slalom		= (direction == RIGHT ? -1 : 1) * 12.0f;
	cycle_slalom 			= ABS(amplitude_slalom) * NAPEIR_INTGRAL / angle;
	before_distance = after_distance = 0.f;

	// フェールセーフ
	if( Control_GetMode() != FAULT ) {
		Control_SetMode(ROTATE);
	} else {
		return;
	}

	// 各パラメータのリセット
	Vehicle_ResetStraight();
	Vehicle_ResetTurning();
	Vehicle_ResetIntegral();
	Control_ResetFilterDistance();
	IMU_ResetGyroAngle_Z();
	LL_mDelay(200);
	Control_ResetEncoderDeviation();
	Control_ResetGyroDeviation();
	Control_ResetAngleDeviation();
	Control_ResetSensorDeviation();
	Control_ResetFrontSensorDeviation();

	// 超信地旋回
	Vehicle_ResetTimer();
	while( (Control_GetMode() != FAULT) && (t < 1.0f/cycle_slalom) ) {
		// 現在時間の取得
		t = Vehicle_GetTimer();
	}

	// 各パラメータのリセット
	cycle_slalom = amplitude_slalom = 0.f;
	LL_mDelay(200);
	Vehicle_ResetStraight();
	Vehicle_ResetTurning();
	Vehicle_ResetIntegral();
	Control_ResetFilterDistance();
	IMU_ResetGyroAngle_Z();
	Control_ResetEncoderDeviation();
	Control_ResetSensorDeviation();
	Control_ResetFrontSensorDeviation();
	Wall_ResetEdgeDistance();
	Wall_ResetEdgeMinDistance();
}

