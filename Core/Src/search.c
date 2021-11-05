
#include "defines.h"
#include "global.h"

static t_init_straight 	param_search;
static float			turn_velocity;

/* ----------------------------------------------------------------------------------
	進む方向の決定（超信地旋回）
-----------------------------------------------------------------------------------*/
int8_t Search_Rotate( t_maze local_maze )
{
	t_position	my				= Position_GetMyPlace();
	int8_t 		next_direction 	= Route_GetNextDirection_Adachi( my );

	Control_ResetEncoderDeviation();
	Control_ResetGyroDeviation();
	Control_ResetAngleDeviation();
	Control_ResetSensorDeviation();
	Control_ResetFrontSensorDeviation();

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

	return next_direction;
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
	探索走行
-----------------------------------------------------------------------------------*/
void Search_Run( int8_t gx, int8_t gy, uint8_t type )
{
	int8_t			next_direction = -1;
	t_position		my;
	t_path			path;
	t_maze			local_maze;

	Control_SetMode( FASTEST );
	Motion_StartStraight( param_search.acceleration, param_search.deceleration, turn_velocity, turn_velocity, 45.f );
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
			// 前壁補正(前壁があるとき)
			local_maze.byte = Wall_GetIsMaze( -1 );
			if( local_maze.bit.north == true ) {
				Motion_StartStraight( param_search.acceleration, param_search.deceleration, turn_velocity, 0.f,
						(Wall_GetDistance(FRONT + LEFT) + Wall_GetDistance(FRONT + RIGHT))/2.f - 31.f );
			} else {
				Motion_StartStraight( param_search.acceleration, param_search.deceleration, turn_velocity, 0.f, 56.f );
			}
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
		Path_SetTurnSection( goal, -1 );

		// パスの変換
//		Path_ConvertTurnLarge();
//		Path_ConvertTurn180();
//		Path_ConvertDiagonal();

		// パスに沿って走行開始
		path = Route_StartPathSequence(0, false);

		if( ((path.straight == 2 && path.type == goal) || path.type == turn_90) && path.direction == -1 ) {

			// ポテンシャルマップ生成
			if( type == ALL ) {
				Potential_MakeUnknownMap( gx, gy );
			} else {
				Potential_MakeMap( gx, gy );
			}

			// パス実行中の待機時間
			if( path.straight == 2 && path.type == goal && path.direction == -1 ) {
				Motion_WaitStraight();
			} else if( path.straight == 0 && path.type == turn_90 && path.direction == -1 ) {
				Motion_WaitSlalom( path.type, path.direction, 0 );
			} else;
		} else;

		if( next_direction == REAR ) {
			Control_SetMode( ADJUST );
			// 前壁補正(前壁があるとき)
			local_maze.byte = Wall_GetIsMaze( -1 );
			if( local_maze.bit.north == true ) {
				Motion_StartStraight( param_search.acceleration, param_search.deceleration, turn_velocity, 0.f,
						(Wall_GetDistance(FRONT + LEFT) + Wall_GetDistance(FRONT + RIGHT))/2.f - 31.f );
			} else {
				Motion_StartStraight( param_search.acceleration, param_search.deceleration, turn_velocity, 0.f, 56.f );
			}
			// ポテンシャルマップ生成
			if( type == ALL ) {
				Potential_MakeUnknownMap( gx, gy );
			} else {
				Potential_MakeMap( gx, gy );
			}
			Motion_WaitStraight();
			LL_mDelay( 200 );
			// 前壁補正と反転
			next_direction = Search_Rotate( local_maze );
			Control_SetMode( ADJUST );
			Motion_StartStraight( param_search.acceleration, param_search.deceleration, turn_velocity, turn_velocity, 34.f );
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

	param_search = Route_GetParameters( SEARCH, 0 );
	turn_velocity = Motion_GetSlalomVelocity(turn_90, 0);

	Control_SetMode( FASTEST );
	Motion_StartStraight( param_search.acceleration, param_search.deceleration, turn_velocity, turn_velocity, START_OFFSET );
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
