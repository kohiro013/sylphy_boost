
#include "defines.h"
#include "global.h"

#define SECTION_WALL_EDGE	(15.f)			// 壁切れ検出区間
#define SECTION_COOLDOWN	(20.f)			// ターン直後の安定化区間


// 直線パラメータ
const static t_init_straight param_straight[] = {
	// 加速度		減速度	  最高速度
	{ 4000.0f,  4000.0f,   700.0f	},	// 0(探索時のみ)
	{ 6000.0f,  6000.0f,  1500.0f	},	// 1
	{ 7000.0f,  6000.0f,  1500.0f	},	// 2
	{ 7000.0f,  7000.0f,  2000.0f	},	// 3
	{ 8000.0f,  7000.0f,  2000.0f	},	// 4
	{ 8000.0f,  8000.0f,  2500.0f	},	// 5
	{ 9000.0f,  8000.0f,  2500.0f	},	// 6
	{ 9000.0f,  9000.0f,  3000.0f	},	// 7
};

// 斜めパラメータ
const static t_init_straight param_diagonal[] = {
	// 加速度		減速度	  最高速度
	{ 4000.0f,  4000.0f,   700.0f	},	// 0(探索時のみ)
	{ 5000.0f,  5000.0f,  1200.0f	},	// 1
	{ 6000.0f,  5000.0f,  1500.0f	},	// 2
	{ 6000.0f,  6000.0f,  1500.0f	},	// 3
	{ 7000.0f,  6000.0f,  2000.0f	},	// 4
	{ 7000.0f,  6500.0f,  2000.0f	},	// 5
	{ 8000.0f,  7000.0f,  2500.0f	},	// 6
	{ 8000.0f,  8000.0f,  2500.0f	},	// 7
};

/* ----------------------------------------------------------------------------------
	パラメータ取得
-----------------------------------------------------------------------------------*/
t_init_straight Route_GetParameters( uint8_t mode, uint8_t param )
{
	if( mode == SEARCH ) {
		return param_straight[0];
	} else if( mode == FASTEST ) {
		if( sizeof(param_straight) / sizeof(param_straight[0]) < param ) {
			return param_straight[sizeof(param_straight) / sizeof(param_straight[0]) - 1];
		} else {
			return param_straight[param];
		}
	} else if( mode == DIAGONAL ) {
		if( sizeof(param_diagonal) / sizeof(param_diagonal[0]) < param ) {
			return param_diagonal[sizeof(param_diagonal) / sizeof(param_diagonal[0]) - 1];
		} else {
			return param_diagonal[param];
		}
	} else;

	return param_straight[0];
}

/* ----------------------------------------------------------------------------------
	最短走行モーション
-----------------------------------------------------------------------------------*/
t_path Route_StartPathSequence( int8_t param, int8_t is_return )
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

		// 未探索区間
		if( path.straight == 2 && path.type == goal && path.direction == -1 ) {
			Control_SetMode(SEARCH);
			turn_velocity = Motion_GetSlalomVelocity(turn_90, 0);
			Motion_StartStraight( acc_straight, dec_straight, turn_velocity, turn_velocity, 90.f );
		// 探索済み区間から未探索区間
		} else if( path.type == turn_90 ) {
			if( path.straight != 0 ) {
				 if( path.straight <= 2 ) {
					 Control_SetMode(FASTEST);
					Motion_StartStraight( acc_straight, dec_straight, turn_velocity, turn_velocity, after_distance );
					Motion_WaitStraight();
					Control_SetMode(SEARCH);
					Motion_StartStraight( acc_straight, dec_straight, turn_velocity, turn_velocity, 45.f*path.straight );
				} else {
					Control_SetMode(FASTEST);
					Motion_StartStraight( acc_straight, dec_straight, max_straight, turn_velocity, 45.f*(path.straight - 2) + after_distance );
					Motion_WaitStraight();
					Control_SetMode(SEARCH);
					Motion_StartStraight( acc_straight, dec_straight, turn_velocity, turn_velocity, 90.f );
				}
				Motion_WaitStraight();
			} else;
		// スタートからの直線走行
		} else if( num == 0 ) {
			if( path.straight == 0 ) {
				Control_SetMode(ADJUST);
			} else {
				Control_SetMode(FASTEST);
			}
			if( is_return == false ) {
				Motion_StartStraight( acc_straight, dec_straight, max_straight, turn_velocity, 45.f*path.straight + START_OFFSET - SECTION_WALL_EDGE );
			} else {
				Motion_StartStraight( acc_straight, dec_straight, max_straight, turn_velocity, 45.f*path.straight - SECTION_WALL_EDGE );
			}
			Motion_WaitStraight();
		// ゴールまでの直線走行
		} else if( path.type == goal ) {
			if( (Position_GetIsGoal(0, 0) == false) && (Position_GetIsGoal(GOAL_X, GOAL_Y) == false) ) {
				Control_SetMode(FASTEST);
				turn_velocity = Motion_GetSlalomVelocity(turn_90, 0);
				Motion_StartStraight( acc_straight, dec_straight, max_straight, turn_velocity, 45.f*path.straight + after_distance );
			} else if( num == 0 || path_old.type == turn_90 ) {
				Control_SetMode(FASTEST);
				turn_velocity = Motion_GetSlalomVelocity(turn_90, 0);
				Motion_StartStraight( acc_straight, dec_straight, max_straight, turn_velocity, 45.f*path.straight + after_distance );
			} else {
				Motion_StartStraight( acc_straight, dec_straight, max_straight, 0.f, 45.f*path.straight + 11.f + after_distance - SECTION_COOLDOWN );
			}
			Motion_WaitStraight();
		// 連続ターン間の直線走行
		} else if( path.straight == 0 ) {
			Control_SetMode(TURN);
			Motion_StartStraight( acc_straight, dec_straight, turn_velocity, turn_velocity, after_distance - SECTION_WALL_EDGE - 10.f );
			Motion_WaitStraight();
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
			}
			Motion_WaitStraight();
		// 直線走行
		} else {
			Motion_StartStraight( acc_straight, dec_straight, max_straight, turn_velocity, 45.f*path.straight + after_distance - SECTION_WALL_EDGE - SECTION_COOLDOWN );
			Motion_WaitStraight();
		}

		if( path.type == turn_0 || path.type == goal ) {
			return path;
		} else {
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
				// 連続ターン時の加減速
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
			Motion_StartSlalom( path.type, path.direction, turn_param );

			// 探索の場合
			if( path.type == turn_90 ) {
				after_distance = 0.f;
				// 未探索区間
				if( num == 0 && path.straight == 0 && path_next.straight == 0 && path_next.type == goal ) {
					path.direction = -1;
					return path;
				// 既知区間
				} else {
					// スラロームの待機時間
					Motion_WaitSlalom( path.type, path.direction, 0 );
				}
			// 最短の場合
			} else {
				// スラロームの後距離
				after_distance = Motion_GetSlalomAfterDistance( path.type, path.direction, turn_param );

				// スラロームの待機時間
				Motion_WaitSlalom( path.type, path.direction, turn_param );

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
					if( path.type == turn_large || path.type == turn_180 ) {
						Control_SetMode( FASTEST );
					} else if( path.type == turn_45in || path.type == turn_135in ) {
						Control_SetMode( DIAGONAL );
					} else if( path.type == turn_45out || path.type == turn_135out ) {
						Control_SetMode( FASTEST );
					} else if( path.type == turn_90v ) {
						Control_SetMode( DIAGONAL );
					} else;
				}
			}
		}
		path_old = path;
	}
	path.type = turn_0;
	return path;
}

/* ----------------------------------------------------------------------------------
	進む方向の決定（足立法）
-----------------------------------------------------------------------------------*/
int8_t Route_GetNextDirection_Adachi( t_position my )
{
	static int8_t 	old_direction 	= -1;
	int8_t 			next_direction	= REAR;
	uint16_t 		min_potential 	= Potential_GetAroundSection( &my, -1 );
	t_maze 			local_maze 		= Maze_GetLocal( &my );

	if( local_maze.bit.east == false ) {
		next_direction = RIGHT;
	} else;

	if( local_maze.bit.west == false ) {
		next_direction = LEFT;
	} else;

	if( local_maze.bit.north == false ) {
		next_direction = FRONT;
	} else;

	// 斜め優先（前回の方向が左だった場合右を優先）
	if( (old_direction == LEFT) && (local_maze.bit.east == false) ) {
		if( min_potential > Potential_GetAroundSection( &my, RIGHT ) ) {
			min_potential = Potential_GetAroundSection( &my, RIGHT );
			next_direction = RIGHT;
		} else;
	} else;

	// 斜め優先（前回の方向が右だった場合左を優先）
	if( (old_direction == RIGHT) && (local_maze.bit.west == false) ) {
		if( min_potential > Potential_GetAroundSection( &my, LEFT ) ) {
			min_potential = Potential_GetAroundSection( &my, LEFT );
			next_direction = LEFT;
		} else;
	} else;

	if( local_maze.bit.north == false ) {
		if( min_potential > Potential_GetAroundSection( &my, FRONT ) ) {
			min_potential = Potential_GetAroundSection( &my, FRONT );
			next_direction = FRONT;
		} else;
	} else;

	if( local_maze.bit.east == false ) {
		if( min_potential > Potential_GetAroundSection( &my, RIGHT ) ) {
			min_potential = Potential_GetAroundSection( &my, RIGHT );
			next_direction = RIGHT;
		} else;
	} else;

	if( local_maze.bit.west == false ) {
		if( min_potential > Potential_GetAroundSection( &my, LEFT ) ) {
			min_potential = Potential_GetAroundSection( &my, LEFT );
			next_direction = LEFT;
		} else;
	} else;

	if( min_potential > Potential_GetAroundSection( &my, REAR ) ) {
		min_potential = Potential_GetAroundSection( &my, REAR );
		next_direction = REAR;
	} else;

	old_direction = next_direction;
	return next_direction;
}

/* ----------------------------------------------------------------------------------
	進む方向の決定（拡張足立法）
-----------------------------------------------------------------------------------*/
int8_t Route_GetNextDirection_PrioritizeUnknown( t_position my )
{
	static int8_t 	old_direction 	= -1;
	int8_t 			next_direction 	= Route_GetNextDirection_Adachi( my );
	t_maze 			local_maze 		= Maze_GetLocal( &my );


	if( (local_maze.bit.west == false) && (Maze_GetIsUnknown(&my, LEFT) == false) ) {
		next_direction = LEFT;
	} else;

	if( (local_maze.bit.east == false) && (Maze_GetIsUnknown(&my, RIGHT) == false) ) {
		next_direction = RIGHT;
	} else;

	// 斜め優先（前回の方向が左だった場合右を優先）
	if( (old_direction == LEFT) && (local_maze.bit.east == false) && (Maze_GetIsUnknown(&my, RIGHT) == false) ) {
		next_direction = RIGHT;
	} else;

	// 斜め優先（前回の方向が右だった場合左を優先）
	if( (old_direction == RIGHT) && (local_maze.bit.west == false) && (Maze_GetIsUnknown(&my, LEFT) == false) ) {
		next_direction = LEFT;
	} else;

	// 直進優先
	if( (local_maze.bit.north == false) && (Maze_GetIsUnknown(&my, FRONT) == false) ) {
		next_direction = FRONT;
	} else;

	old_direction = next_direction;
	return next_direction;
}
