
#include "defines.h"
#include "global.h"

#define SECTION_WALL_EDGE	(15.f)			// 壁切れ検出区間
#define SECTION_COOLDOWN	(20.f)			// ターン直後の安定化区間

const t_init_straight param_straight[] = {
	// 加速度	減速度	  最高速度
	{ 6000.0f,  6000.0f,  1500.0f	},	// 0
	{ 7000.0f,  6000.0f,  1500.0f	},	// 1
	{ 7000.0f,  7000.0f,  2000.0f	},	// 2
	{ 8000.0f,  7000.0f,  2000.0f	},	// 3
	{ 8000.0f,  8000.0f,  2500.0f	},	// 4
	{ 9000.0f,  8000.0f,  2500.0f	},	// 5
	{ 9000.0f,  9000.0f,  3000.0f	},	// 6
};

const t_init_straight param_diagonal[] = {
	// 加速度	減速度	  最高速度
	{ 5000.0f,  5000.0f,  1200.0f	},	// 0
	{ 6000.0f,  5000.0f,  1500.0f	},	// 1
	{ 6000.0f,  6000.0f,  1500.0f	},	// 2
	{ 7000.0f,  6000.0f,  2000.0f	},	// 3
	{ 7000.0f,  6500.0f,  2000.0f	},	// 4
	{ 8000.0f,  7000.0f,  2500.0f	},	// 5
	{ 8000.0f,  8000.0f,  2500.0f	},	// 6
};

/* ----------------------------------------------------------------------------------
	最短走行用パスの生成
-----------------------------------------------------------------------------------*/
void Fastest_SetPath( int8_t next_direction )
{
	switch( next_direction ) {
		case FRONT:	Path_SetStraightSection( 2 );			break;
		case RIGHT:	Path_SetTurnSection( turn_90, RIGHT );	break;
		case LEFT:	Path_SetTurnSection( turn_90, LEFT );	break;
		case REAR:											break;
	}
}

/* ----------------------------------------------------------------------------------
	最短走行モーション
-----------------------------------------------------------------------------------*/
void Fastest_StartPathSequence( int8_t param, int8_t is_return )
{
	uint8_t		num;
	t_path		path;
	t_path		path_next, path_old;

	uint8_t		turn_param = param;
	float		turn_velocity;
	float		after_distance 	= 0.0f;

	if( param > sizeof(param_straight)/sizeof(param_straight[0]) ) {
		param = sizeof(param_straight)/sizeof(param_straight[0]);
	} else if( param < 0 ) {
		param = 0;
	} else;

	// 最短走行時の走行パラメータ変数群
	const float	max_straight = param_straight[param].max_velocity;
	const float	acc_straight = param_straight[param].acceleration;
	const float	dec_straight = param_straight[param].deceleration;
	const float	max_diagonal = param_diagonal[param].max_velocity;
	const float	acc_diagonal = param_diagonal[param].acceleration;
	const float	dec_diagonal = param_diagonal[param].deceleration;

	// エラーかゴールパスに辿り着くまでパスにしたがって走行
	for( num = 0; num < 255; num++ ) {
		if( Control_GetMode() == FAULT ) { break; }
		// パスの読み込み
		if( is_return == false ) {
			path = Path_GetSequence( num );
			path_next = Path_GetSequence( num+1 );
		} else {
			path = Path_GetReturnSequence( num );
			path_next = Path_GetReturnSequence( num+1 );
		}

		// スラローム速度の読み込み
		turn_velocity = Motion_GetSlalomVelocity( path.type, turn_param );

		// スタートからの直線走行
		if( num == 0 ) {
			if( path.straight == 0 ) {
				Control_SetMode(ADJUST);
			} else {
				Control_SetMode(FASTEST);
				//Control_SetMode(ADJUST);
			}
			if( is_return == false ) {
				Motion_StartStraight( acc_straight, dec_straight, max_straight, turn_velocity, 45.f*path.straight + START_OFFSET - SECTION_WALL_EDGE );
			} else {
				Motion_StartStraight( acc_straight, dec_straight, max_straight, turn_velocity, 45.f*path.straight - SECTION_WALL_EDGE );
			}
		// ゴールまでの直線走行
		} else if( path.type == goal ) {
			Motion_StartStraight( acc_straight, dec_straight, max_straight, 0.f, 45.f*path.straight + after_distance - SECTION_COOLDOWN );
		// 連続ターン間の直線走行
		} else if( path.straight == 0 ) {
			Control_SetMode(TURN);
			Motion_StartStraight( acc_straight, dec_straight, turn_velocity, turn_velocity, after_distance - SECTION_WALL_EDGE - 10.f );
		// 斜め走行
		} else if( path.type >= turn_90v && path.type <= turn_135out ) {
			if( path.straight <= 1 ) {
				Control_SetMode(TURN);
				Motion_StartStraight( acc_diagonal, dec_diagonal, turn_velocity, turn_velocity, 45.f*SQRT2*path.straight + after_distance - SECTION_WALL_EDGE - SECTION_COOLDOWN );
			} else {
				Motion_StartStraight( acc_diagonal, dec_diagonal, max_diagonal, turn_velocity, 45.f*SQRT2*(path.straight) + after_distance - SECTION_COOLDOWN - SECTION_WALL_EDGE );
				Motion_WaitStraight();
				// 最後の1区画で斜め制御をなくすため
				Control_SetMode(TURN);
				//Motion_StartStraight( acc_diagonal, dec_diagonal, turn_velocity, turn_velocity, 45.f*SQRT2 - SECTION_WALL_EDGE );
			}
		// 直線走行
		} else {
			Motion_StartStraight( acc_straight, dec_straight, max_straight, turn_velocity, 45.f*path.straight + after_distance - SECTION_WALL_EDGE - SECTION_COOLDOWN );
		}
		// 直線走行の待機時間
		Motion_WaitStraight();
		// 各パラメータのリセット
		if( path.straight > 1 ) {
			Vehicle_ResetTurning();
			Vehicle_ResetIntegral();
			Control_ResetFilterDistance();
			IMU_ResetGyroAngle_Z();
			Control_ResetAngleDeviation();
			Control_ResetSensorDeviation();
			Motion_SetSlalomAcceleration( 0.f );
		} else {
			if( turn_large <= path.type && path.type <= turn_135in ) {
				if( Vehicle_GetVelocity() < turn_velocity )
					Motion_SetSlalomAcceleration( acc_straight );
				else
					Motion_SetSlalomAcceleration( dec_straight );
			} else {
				if( Vehicle_GetVelocity() < turn_velocity )
					Motion_SetSlalomAcceleration( acc_diagonal );
				else
					Motion_SetSlalomAcceleration( dec_diagonal );
			}
		}

		// スラローム
		if( path.type == turn_0 || path.type == goal ) {
			break;
		} else {
			Motion_StartSlalom( path.type, path.direction, turn_param );
		}
		// スラロームの待機時間
		Motion_WaitSlalom( path.type, path.direction, turn_param );

		// スラロームの後距離
		after_distance = Motion_GetSlalomAfterDistance( path.type, path.direction, turn_param );

		// ターン後の直線時における制御方法の切り替え
		if( path_next.straight == 0 ) {
			if( path_next.type == turn_45out || path_next.type == turn_135out || path_next.type == turn_90v ) {
				Control_SetMode(ADJUST);
			} else {
				Control_SetMode(FASTEST);
			}
		} else {
			// 各パラメータのリセット
			IMU_OffsetGyroAngle_Z();
			Vehicle_ResetTurning();
			Vehicle_ResetIntegral();
			Control_ResetFilterDistance();

			// ターン直後の安定化区間
			Motion_StartStraight( acc_straight, dec_straight, turn_velocity, turn_velocity, SECTION_COOLDOWN );
			Motion_WaitStraight();

			// 制御モードの切替
			if( path.type == turn_90 ) {
				Control_SetMode( SEARCH );
			} else if( path.type == turn_large || path.type == turn_180 ) {
				Control_SetMode( FASTEST );
			} else if( path.type == turn_45in || path.type == turn_135in ) {
				Control_SetMode( DIAGONAL );
			} else if( path.type == turn_45out || path.type == turn_135out ) {
				Control_SetMode( FASTEST );
			} else if( path.type == turn_90v ) {
				Control_SetMode( DIAGONAL );
			} else;
		}

		path_old = path;
	}
}

/* ----------------------------------------------------------------------------------
	最短走行
-----------------------------------------------------------------------------------*/
void Fastest_RunSimple( int8_t param, int8_t is_return )
{
//	int8_t			next_direction = -1;
	t_position		my;

	Maze_LoadFlash();
	Maze_Reset( FASTEST );
	Maze_InsertDeadEnd();
	Position_Reset();

	// 最短経路生成
	Path_Reset();
	my = Dijkstra_ConvertPath( GOAL_X, GOAL_Y );

	// パスに沿って走行開始
	Fastest_StartPathSequence( param, false );

	// 制御のリセット
	if( Control_GetMode() != FAULT ) {
		LL_mDelay( 500 );
		Motor_StopPWM();
		LED_LightBinary( 0x01 ); 	LL_mDelay( 100 );
		LED_LightBinary( 0x02 );  	LL_mDelay( 100 );
		LED_LightBinary( 0x04 );  	LL_mDelay( 100 );
		LED_LightBinary( 0x02 );  	LL_mDelay( 100 );
		LED_LightBinary( 0x01 );  	LL_mDelay( 100 );
		LED_LightBinary( 0x00 );
	} else;

	if( Control_GetMode() != FAULT && is_return == true ) {
		// 各パラメータのリセット
		Control_ResetFilterDistance();
		IMU_ResetGyroAngle_Z();
		Vehicle_ResetStraight();
		Vehicle_ResetTurning();
		Vehicle_ResetIntegral();
		Control_ResetEncoderDeviation();
		Control_ResetGyroDeviation();
		Control_ResetAngleDeviation();
		Control_ResetSensorDeviation();
		Control_ResetFrontSensorDeviation();

		// 前壁補正(前壁があるとき)
		if( Maze_IsLocal(my.x, my.y, my.dir, FRONT) == true ) {
			Control_SetMode( FWALL );
			LL_mDelay( 500 );
		} else;

		// 反転して帰還走行開始
		if( Maze_IsLocal(my.x, my.y, my.dir, RIGHT) == true ) {
			Motion_StartRotate( 90.f, RIGHT );
			Control_SetMode( FWALL );
			LL_mDelay( 500 );
			Motion_StartRotate( 90.f, RIGHT );
		} else if( Maze_IsLocal(my.x, my.y, my.dir, LEFT) == true ) {
			Motion_StartRotate( 90.f, LEFT );
			Control_SetMode( FWALL );
			LL_mDelay( 500 );
			Motion_StartRotate( 90.f, LEFT );
		} else {
			Motion_StartRotate( 180.f, RIGHT );
		}
		Control_ResetGyroDeviation();
		Control_ResetAngleDeviation();

		// パスに沿って走行開始
		//Fastest_StartPathSequence( param, true );
		Fastest_StartPathSequence( 0, true );

		// 制御のリセット
		if( Control_GetMode() != FAULT ) {
			LL_mDelay( 500 );
			Motor_StopPWM();
			LED_LightBinary( 0x01 ); 	LL_mDelay( 100 );
			LED_LightBinary( 0x02 );  	LL_mDelay( 100 );
			LED_LightBinary( 0x04 );  	LL_mDelay( 100 );
			LED_LightBinary( 0x02 );  	LL_mDelay( 100 );
			LED_LightBinary( 0x01 );  	LL_mDelay( 100 );
			LED_LightBinary( 0x00 );
		} else;
	}
}

