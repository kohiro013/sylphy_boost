
#include "defines.h"
#include "global.h"

#define SLIP_RATE			(0.02f)			// タイヤのスリップ率
#define SECTION_WALL_EDGE	(15.f)			// 壁切れ検出区間

static volatile int8_t turn_param[256];

// 直線パラメータ
const static t_init_straight param_straight[] = {
	// 加速度		減速度	  最高速度
	{ 4000.0f,  4000.0f,   700.0f	},	// 0(探索時のみ)
	{ 6000.0f,  6000.0f,  1500.0f	},	// 1
	{ 8000.0f,  7000.0f,  2000.0f	},	// 2
	{10000.0f,  9000.0f,  2500.0f	},	// 3
	{11000.0f, 10000.0f,  2500.0f	},	// 4
	{12000.0f, 11000.0f,  3000.0f	},	// 5
	{13000.0f, 12000.0f,  3000.0f	},	// 6
	{14000.0f, 13000.0f,  3500.0f	},	// 7
	{15000.0f, 14000.0f,  4000.0f	},	// 8
};

// 斜めパラメータ
const static t_init_straight param_diagonal[] = {
	// 加速度		減速度	  最高速度
	{ 4000.0f,  4000.0f,   700.0f	},	// 0(探索時のみ)
	{ 6000.0f,  5000.0f,  1200.0f	},	// 1
	{ 6000.0f,  5000.0f,  1500.0f	},	// 2
	{ 8000.0f,  7000.0f,  2000.0f	},	// 3
	{ 9000.0f,  8000.0f,  2000.0f	},	// 4
	{10000.0f,  9000.0f,  2500.0f	},	// 5
	{11000.0f, 10000.0f,  2500.0f	},	// 6
	{12000.0f, 11000.0f,  3000.0f	},	// 7
	{13000.0f, 12000.0f,  3500.0f	},	// 8
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
	ターンの速度パラメータ計算
-----------------------------------------------------------------------------------*/
void Route_AdjustTurnParameter( int8_t param, int8_t is_return )
{
	t_path		path;
	float		turn_v;
	float		before_v;
	float		acceleration;
	float		before_distance;
	float		after_distance;
	float		distance;
	int8_t		is_adjust = false;

	// ターンパラメータの初期化
	for( uint16_t i = 0; i < 255; i++ ) {
		turn_param[i] = param;
	}

	// ターンパラメータの調整が終わるまでループ
	do {
		is_adjust = false;					// ループフラグのリセット
		before_v = after_distance = 0.f;	// 前回ループの値をリセット
		for( uint16_t num = 0; num < 255; num++ ) {
			// パスの取得
			if( is_return == false ) {
				path = Path_GetSequence(num);
			} else {
				path = Path_GetReturnSequence(num);
			}

			// ゴールまたは異常時にループ終了
			if( path.type == goal || path.type == turn_0 ) {
				break;
			} else;

			// ターンパラメータの取得
			turn_v = Motion_GetSlalomVelocity(path.type, turn_param[num]);
			before_distance = Motion_GetSlalomBeforeDistance(path.type, path.direction, turn_param[num]);
			if( num == 0 ) {
				acceleration = param_straight[param].acceleration;
				if( is_return == false ) {
					distance = 45.f * path.straight + before_distance + START_OFFSET - 5.f;
				} else {
					distance = 45.f * path.straight + before_distance - START_OFFSET - 5.f;
				}
			} else {
				// 直線走行
				if( turn_large <= path.type && path.type <= turn_135in ) {
					if( turn_v > before_v ) acceleration = param_straight[param].acceleration;
					else					acceleration = param_straight[param].deceleration;
					distance = 45.f * path.straight + before_distance + after_distance;
				// 斜め走行
				} else {
					if( turn_v > before_v ) acceleration = param_diagonal[param].acceleration;
					else					acceleration = param_diagonal[param].deceleration;
					distance = 45.f*SQRT2 * path.straight + before_distance + after_distance;
				}
				// スリップ区間の確保
				distance -= MAX(distance * SLIP_RATE, SECTION_WALL_EDGE);
			}

			// デバッグ用
/*			Path_Display(path);
			if( path.type == turn_180 || path.type == turn_45in || path.type == turn_90v
					|| ((path.type == turn_large || path.type == turn_135in || path.type == turn_45out) && path.direction == LEFT) ) {
				printf("\t");
			} else;
			printf("\t%4.0f, %4.0f, %5.0f, %4.1f, %4.1f, %5.1f, %4.1f, %1d\r\n", before_v, turn_v, acceleration,
					before_distance, after_distance, distance, (turn_v - before_v) * (turn_v - before_v) / (2*acceleration), turn_param[num]);
*/
			// ターン速度の調整
			if( (turn_v - before_v) * (turn_v - before_v) / (2*acceleration) > distance ) {
				if( turn_v < before_v ) {	// 減速距離が足りない場合一つ前のターンを遅くする
					if( turn_param[num-1] > 0 ) {
						turn_param[num-1] --;
						is_adjust = true;
					} else;
				} else {					// 加速距離が足りない場合このターンを遅くする
					if( turn_param[num] > 0 ) {
						turn_param[num] --;
						is_adjust = true;
					} else;
				}
			} else;

			// 一つ前のターンパラメータを取得
			before_v = turn_v;
			after_distance = Motion_GetSlalomAfterDistance(path.type, path.direction, turn_param[num]);
		}
		printf("\r\n\n");
	} while( is_adjust );
}

/* ----------------------------------------------------------------------------------
	最短走行モーション
-----------------------------------------------------------------------------------*/
t_path Route_StartPathSequence( int8_t param, int8_t is_return )
{
	uint8_t		num;
	t_path		path;
	t_path		path_next, path_old;

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

	// ターンパラメータの取得
	Route_AdjustTurnParameter(param, is_return);

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
		turn_velocity = Motion_GetSlalomVelocity( path.type, turn_param[num] );

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
				Control_SetMode(TURN);
			} else {
				Control_SetMode(FASTEST);
				if( is_return == false ) {
					Motion_StartStraight( acc_straight, dec_straight, max_straight, turn_velocity,
							45.f*path.straight + START_OFFSET - MAX((45.f*path.straight + START_OFFSET) * SLIP_RATE, SECTION_WALL_EDGE) );
				} else {
					Motion_StartStraight( acc_straight, dec_straight, max_straight, turn_velocity,
							45.f*path.straight - MAX(45.f*path.straight * SLIP_RATE, SECTION_WALL_EDGE) - START_OFFSET );
				}
				Motion_WaitStraight();
			}
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
				Motion_StartStraight( acc_straight, dec_straight, max_straight, 0.f, 45.f*path.straight + 11.f + after_distance );
			}
			Motion_WaitStraight();
		// 連続ターン間の直線走行
		} else if( path.straight == 0 ) {
			if( path.type >= turn_90v && path.type <= turn_135out ) {
				Control_SetMode(ADJUST);
			} else;
//			Motion_StartStraight( acc_straight, dec_straight, turn_velocity, turn_velocity,
//					after_distance - MAX(after_distance * SLIP_RATE, SECTION_WALL_EDGE) );
//			Motion_WaitStraight();
		// 斜め走行
		} else if( path.type >= turn_90v && path.type <= turn_135out ) {
			if( path.straight <= 1 ) {
				Control_SetMode(ADJUST);
				Motion_StartStraight( acc_diagonal, dec_diagonal, turn_velocity, turn_velocity,
						45.f*SQRT2*path.straight + after_distance - SECTION_WALL_EDGE - 5.f );
			} else {
				Motion_StartStraight( acc_diagonal, dec_diagonal, max_diagonal, turn_velocity,
						45.f*SQRT2*path.straight + after_distance - MAX((45.f*SQRT2*path.straight + after_distance) * SLIP_RATE, SECTION_WALL_EDGE) );
				Motion_WaitStraight();
				// 最後の1区画で斜め制御をなくすため
				Control_SetMode(TURN);
			}
			Motion_WaitStraight();
		// 直線走行
		} else {
			Motion_StartStraight( acc_straight, dec_straight, max_straight, turn_velocity,
					45.f*path.straight + after_distance - MAX((45.f*path.straight + after_distance) * SLIP_RATE, SECTION_WALL_EDGE) );
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
				//Control_ResetAngleDeviation();
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
			Motion_StartSlalom( path.type, path.direction, turn_param[num] );

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
				after_distance = Motion_GetSlalomAfterDistance( path.type, path.direction, turn_param[num] );

				// スラロームの待機時間
				Motion_WaitSlalom( path.type, path.direction, turn_param[num] );

				// ターン後の直線時における制御方法の切り替え
				if( path_next.straight == 0 ) {
					if( path_next.type >= turn_90v && path_next.type <= turn_135out ) {
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

					// 制御モードの切替
					if( path.type == turn_large || path.type == turn_180 ) {
						Control_SetMode( FASTEST );
					} else if( path.type == turn_45in || path.type == turn_135in ) {
						Control_SetMode( DIAGONAL );
					} else if( path.type == turn_45out || path.type == turn_135out ) {
						Control_SetMode( FASTEST );
					} else if( path.type == turn_90v || path.type == turn_kojima ) {
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
