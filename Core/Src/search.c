
#include "defines.h"
#include "global.h"

const float SEARCH_ACCEL 		= 4000.f;
const float SEARCH_DECEL 		= 4000.f;
const float SEARCH_VELOCITY		= 350.f;
const float SEARCH_MAX_VELOCITY	= 700.f;

#define SECTION_WALL_EDGE	(15.f)			// 壁切れ検出区間

/* ----------------------------------------------------------------------------------
	進む方向の決定（超信地旋回）
-----------------------------------------------------------------------------------*/
void Search_Rotate( t_maze local_maze )
{
	t_position	my				= Position_GetMyPlace();
	int8_t 		next_direction 	= Route_GetNextDirection_Adachi( my );

	Control_ResetEncoderDeviation();
	Control_ResetGyroDeviation();
	Control_ResetAngleDeviation();
	Control_ResetSensorDeviation();
	Control_ResetFrontSensorDeviation();

	// 前壁補正(前壁があるとき)
	if( local_maze.bit.north == true ) {
		Control_SetMode( FWALL );
		LL_mDelay( 500 );
	} else;

	switch( next_direction ) {
		case FRONT:
			break;
		case RIGHT:
			if( local_maze.bit.north == false && local_maze.bit.west == true ) {
				Motion_StartRotate( 90.f, LEFT );
				Control_SetMode( FWALL );
				LL_mDelay( 500 );
				Control_SetMode( TURN );
				Motion_StartRotate( 180.f, RIGHT );
			} else {
				Motion_StartRotate( 90.f, RIGHT );
			}
			Control_ResetGyroDeviation();
			Control_ResetAngleDeviation();
			break;
		case LEFT:
			if( local_maze.bit.north == false && local_maze.bit.east == true ) {
				Motion_StartRotate( 90.f, RIGHT );
				Control_SetMode( FWALL );
				LL_mDelay( 500 );
				Control_SetMode( TURN );
				Motion_StartRotate( 180.f, LEFT );
			} else {
				Motion_StartRotate( 90.f, LEFT );
			}
			Control_ResetGyroDeviation();
			Control_ResetAngleDeviation();
			break;
		case REAR:
			if( local_maze.bit.east == true ) {
				Motion_StartRotate( 90.f, RIGHT );
				Control_SetMode( FWALL );
				LL_mDelay( 500 );
				Control_SetMode( TURN );
				Motion_StartRotate( 90.f, RIGHT );
			} else if( local_maze.bit.west == true ) {
				Motion_StartRotate( 90.f, LEFT );
				Control_SetMode( FWALL );
				LL_mDelay( 500 );
				Control_SetMode( TURN );
				Motion_StartRotate( 90.f, LEFT );
			} else {
				Control_SetMode( TURN );
				Motion_StartRotate( 180.f, LEFT );
			}
			Control_ResetGyroDeviation();
			Control_ResetAngleDeviation();
			break;
	}
	Position_RotateMyDirection( next_direction );
}

/* ----------------------------------------------------------------------------------
	探索走行用パスの生成
-----------------------------------------------------------------------------------*/
void Search_SetPath( int8_t next_direction )
{
	switch( next_direction ) {
		case FRONT:	Path_SetStraightSection( 2 );			break;
		case RIGHT:	Path_SetTurnSection( turn_90, RIGHT );	break;
		case LEFT:	Path_SetTurnSection( turn_90, LEFT );	break;
		case REAR:											break;
	}
}

/* ----------------------------------------------------------------------------------
	探索走行モーション
-----------------------------------------------------------------------------------*/
t_path Search_StartPathSequence( void )
{
	uint8_t		num;
	t_path		path;
	float		turn_velocity;
	float		after_distance 	= 0.0f;

	// エラーかゴールパスに辿り着くまでパスにしたがって走行
	for( num = 0; num < 255; num++ ) {
		if( Control_GetMode() == FAULT ) { break; }
		// パスの読み込み
		path = Path_GetSequence( num );

		// スラローム速度の読み込み
		turn_velocity = Motion_GetSlalomVelocity( path.type, 0 );

		// 未探索区間
		if( path.straight == 2 && path.type == goal ) {
			Control_SetMode( SEARCH );
			Motion_StartStraight( SEARCH_ACCEL, SEARCH_DECEL, SEARCH_VELOCITY, SEARCH_VELOCITY, 90.f );
		// 探索済み区間から未探索区間
		} else if( path.type == turn_90 ) {
			if( path.straight != 0 ) {
				 if( path.straight <= 2 ) {
					 Control_SetMode( FASTEST );
					Motion_StartStraight( SEARCH_ACCEL, SEARCH_DECEL, SEARCH_VELOCITY, SEARCH_VELOCITY, after_distance );
					Motion_WaitStraight();
					Control_SetMode( SEARCH );
					Motion_StartStraight( SEARCH_ACCEL, SEARCH_DECEL, SEARCH_VELOCITY, SEARCH_VELOCITY, 45.f*path.straight );
				} else {
					Control_SetMode( FASTEST );
					Motion_StartStraight( SEARCH_ACCEL, SEARCH_DECEL, SEARCH_MAX_VELOCITY, SEARCH_VELOCITY, 45.f*(path.straight - 2) + after_distance );
					Motion_WaitStraight();
					Control_SetMode( SEARCH );
					Motion_StartStraight( SEARCH_ACCEL, SEARCH_DECEL, SEARCH_VELOCITY, SEARCH_VELOCITY, 90.f );
				}
				Motion_WaitStraight();
			} else;
		// ゴールまでの直線区間
		} else if( path.type == goal ) {
			Control_SetMode( FASTEST );
			Motion_StartStraight( SEARCH_ACCEL, SEARCH_DECEL, SEARCH_MAX_VELOCITY, SEARCH_VELOCITY, 45.f*path.straight + after_distance );
			Motion_WaitStraight();
		// 既知区間での直線走行
		} else {
			Control_SetMode( FASTEST );
			// 連続ターン間の直線部分
			if( path.straight == 0 ) {
				Motion_StartStraight( SEARCH_ACCEL, SEARCH_DECEL, SEARCH_MAX_VELOCITY, SEARCH_VELOCITY, after_distance - SECTION_WALL_EDGE );
			// 斜め区間
			} else if( path.type >= turn_90v && path.type <= turn_135out ) {
				Control_SetMode( DIAGONAL );
				Motion_StartStraight( SEARCH_ACCEL, SEARCH_DECEL, SEARCH_MAX_VELOCITY, turn_velocity, 45.f*SQRT2*path.straight + after_distance - SECTION_WALL_EDGE );
			// 直線区間
			} else {
				Motion_StartStraight( SEARCH_ACCEL, SEARCH_DECEL, SEARCH_MAX_VELOCITY, turn_velocity, 45.f*path.straight + after_distance - SECTION_WALL_EDGE );
			}
			Motion_WaitStraight();
			// 各パラメータのリセット
			if( path.straight > 1 ) {
				Vehicle_ResetTurning();
				Vehicle_ResetIntegral();
				Control_ResetFilterDistance();
				IMU_ResetGyroAngle_Z();
				Control_ResetAngleDeviation();
				Control_ResetSensorDeviation();
			} else;
		}

		// ターン区間
		if( (path.type == turn_0) || (path.type == goal) ) {
			return path;
		} else {
			// スラローム
			Motion_StartSlalom( path.type, path.direction, 0 );

			// スラロームの後距離
			if( path.type == turn_90 ) {
				after_distance = 0.f;
				// 未探索区間
				t_path next_path = Path_GetSequence( num + 1 );
				if( num == 0 && path.straight == 0 && next_path.straight == 0 && next_path.type == goal ) {
					path.direction = -1;
					return path;
				// 既知区間
				} else {
					Motion_WaitSlalom( path.type, path.direction, 0 );
				}
			} else {
				Motion_WaitSlalom( path.type, path.direction, 0 );
				after_distance = Motion_GetSlalomAfterDistance( path.type, path.direction, 0 );
			}

		}
	}
	path.type = turn_0;
	return path;
}

/* ----------------------------------------------------------------------------------
	探索走行
-----------------------------------------------------------------------------------*/
void Search_Run( int8_t gx, int8_t gy, uint8_t type )
{
	int8_t			next_direction = -1;
	t_position		my;
	t_path			path;
	t_maze			local_maze;

	Control_SetMode( FASTEST );
	Motion_StartStraight( SEARCH_ACCEL, SEARCH_DECEL, SEARCH_VELOCITY, SEARCH_VELOCITY, 45.f );
	if( type == ALL ) {
		Potential_MakeUnknownMap( gx, gy );
	} else {
		Potential_MakeMap( gx, gy );
	}
	Motion_WaitStraight();
	Control_SetMode( SEARCH );
	my = Position_MoveMyPlace( FRONT );

	while( Control_GetMode() != FAULT ) {
		Maze_SetFromSensor( &my );
		Maze_SetPillarAlone( &my );
		Maze_InsertDeadEnd();

		if( Position_GetIsGoal(gx, gy) != false ) {
			Control_SetMode( SEARCH );
			Motion_StartStraight( SEARCH_ACCEL, SEARCH_DECEL, SEARCH_VELOCITY, 0.f, 45.f );
			Motion_WaitStraight();
			LL_mDelay( 500 );
			Motor_StopPWM();
			LED_LightBinary( 0x01 ); 	LL_mDelay( 100 );
			LED_LightBinary( 0x02 );  	LL_mDelay( 100 );
			LED_LightBinary( 0x04 );  	LL_mDelay( 100 );
			LED_LightBinary( 0x02 );  	LL_mDelay( 100 );
			LED_LightBinary( 0x01 );  	LL_mDelay( 100 );
			LED_LightBinary( 0x00 );
			break;
		} else;

		// 探索経路生成
		Path_Reset();
		do {
			if( type == ADACHI ) {
				next_direction = Route_GetNextDirection_Adachi( my );
			} else {
				next_direction = Route_GetNextDirection_PrioritizeUnknown( my );
			}

			if( next_direction == REAR ) {
				break;
			} else {
				Search_SetPath( next_direction );
				my = Position_MoveMyPlace( next_direction );
			}
		} while( (Control_GetMode() != FAULT) && (Maze_GetIsUnknown(&my, -1) == true) && (Position_GetIsGoal(gx, gy) == false) );
		Path_SetTurnSection( goal, 0 );

		// パスの変換
//		Path_ConvertTurnLarge();
//		Path_ConvertTurn180();
//		Path_ConvertDiagonal();

		// パスに沿って走行開始
		path = Search_StartPathSequence();

		if( (path.straight == 2 && path.type == goal) || (path.type == turn_90 && path.direction == -1) ) {

			// ポテンシャルマップ生成
			if( type == ALL ) {
				Potential_MakeUnknownMap( gx, gy );
			} else {
				Potential_MakeMap( gx, gy );
			}

			// パス実行中の待機時間
			if( path.straight == 2 && path.type == goal ) {
				Motion_WaitStraight();
			} else if( path.straight == 0 && path.type == turn_90 && path.direction == -1 ) {
				Motion_WaitSlalom( path.type, path.direction, 0 );
			} else;
		} else;

		if( next_direction == REAR ) {
			local_maze.byte = Wall_GetIsMaze( -1 );
			Control_SetMode( ADJUST );
			Motion_StartStraight( SEARCH_ACCEL, SEARCH_DECEL, SEARCH_VELOCITY, 0.f, 45.f );
			// ポテンシャルマップ生成
			if( type == ALL ) {
				Potential_MakeUnknownMap( gx, gy );
			} else {
				Potential_MakeMap( gx, gy );
			}
			Motion_WaitStraight();
			LL_mDelay( 200 );
			// 前壁補正と反転
			Search_Rotate( local_maze );
			Control_SetMode( ADJUST );
			Motion_StartStraight( SEARCH_ACCEL, SEARCH_DECEL, SEARCH_VELOCITY, SEARCH_VELOCITY, 45.f );
			Motion_WaitStraight();
			my = Position_MoveMyPlace( FRONT );
		} else;
	}
}

/* ----------------------------------------------------------------------------------
	探索走行(1：足立法、2：拡張足立法、3：全面探索法)
-----------------------------------------------------------------------------------*/
void Search_RunStart( uint8_t type, int8_t is_return )
{
	Position_Reset();
	Maze_Reset( SEARCH );
	Maze_ResetBuffer();

	Control_SetMode( FASTEST );
	Motion_StartStraight( SEARCH_ACCEL, SEARCH_DECEL, SEARCH_VELOCITY, SEARCH_VELOCITY, START_OFFSET );
	Motion_WaitStraight();
	Search_Run( GOAL_X, GOAL_Y, type );

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

	if( is_return == true ) {
		Potential_MakeUnknownMap( 0, 0 );
		t_position	my = Position_GetMyPlace();
		Search_Rotate( Maze_GetLocal(&my) );
		Search_Run( 0, 0, ALL );
	} else;

	if( Control_GetMode() == FAULT ) {
		Maze_DeleteWhenClash();
	} else;
	Maze_StoreFlash();
}
