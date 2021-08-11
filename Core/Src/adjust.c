
#include "defines.h"
#include "global.h"

/* ---------------------------------------------------------------
	タイヤ直径の調整関数
--------------------------------------------------------------- */
void Adjust_RunTireDiameter( int8_t section )
{
	Control_SetMode( FASTEST );
	Motion_StartStraight( 4000.f, 4000.f, 350.f, 0.f, 90.f*section + START_OFFSET );
	Motion_WaitStraight();
	LL_mDelay( 1000 );
}

/* ---------------------------------------------------------------
	ジャイロの角度係数の調整関数
--------------------------------------------------------------- */
void Adjust_RunGyroSensitivity( uint8_t count, int8_t direction )
{
	Control_SetMode( ROTATE );
	for( int i = 0; i < count; i++ ) {
		Motion_StartRotate( 180.f, direction );
	}
	LL_mDelay( 1000 );
}

/* ---------------------------------------------------------------
	櫛補正の調整関数
--------------------------------------------------------------- */
void Adjust_RunComb( int8_t section )
{
	Control_SetMode( FASTEST );
	Motion_StartStraight( 6000.f, 6000.f, 1000.f, 0.f, 90.f*section + START_OFFSET );
	Motion_WaitStraight();
	LL_mDelay( 1000 );
}

/* ---------------------------------------------------------------
	斜め補正の調整関数
--------------------------------------------------------------- */
void Adjust_RunDiagonal( int8_t section )
{
	Control_SetMode( DIAGONAL );
	Motion_StartStraight( 6000.f, 6000.f, 1000.f, 0.f, 45.f*SQRT2*section );
	Motion_WaitStraight();
	LL_mDelay( 1000 );
}

/* ---------------------------------------------------------------
	直進走行時の壁切れ補正の距離調整関数
--------------------------------------------------------------- */
void Adjust_RunSearchWallEdge( void )
{
	Control_SetMode( SEARCH );
	Motion_StartStraight( 4000.f, 4000.f, 350.f, 350.f, 45.f + START_OFFSET );
	Motion_WaitStraight();
	Motion_StartStraight( 4000.f, 4000.f, 350.f, 350.f, 90.f );
	Motion_WaitStraight();
	Motion_StartStraight( 4000.f, 4000.f, 350.f,   0.f, 45.f );
	Motion_WaitStraight();
	Control_SetMode( TURN );
	LL_mDelay( 1000 );
}

void Adjust_RunFastestWallEdge( void )
{
	Control_SetMode( FASTEST );
	Motion_StartStraight( 6000.f, 6000.f, 1000.f, 1000.f, 90.f*2.f - 10.f + START_OFFSET );
	Motion_WaitStraight();
	while( 1 ) {
		if(Wall_GetEdge(RIGHT) == true || Wall_GetEdge(LEFT) == true) {
			break;
		}
	}
	Motion_StartStraight( 6000.f, 6000.f, 1000.f,    0.f, 90.f );
	Motion_WaitStraight();
	Control_SetMode( TURN );
	LL_mDelay( 1000 );
}

/* ---------------------------------------------------------------
	壁切れ補正の横ずれ調整関数
--------------------------------------------------------------- */
void Adjust_RunGapWallEdge( void )
{
	Control_SetMode( ADJUST );
	Motion_StartStraight( 4000.f, 4000.f, 350.f, 350.f, 45.f + START_OFFSET );
	Motion_WaitStraight();
	Wall_ResetEdgeDistance();
	Motion_StartStraight( 4000.f, 4000.f, 350.f, 350.f, 90.f );
	Motion_WaitStraight();
	Motion_StartStraight( 4000.f, 4000.f, 350.f, 0.f, 45.f );
	Motion_WaitStraight();
	LL_mDelay( 1000 );
	Control_SetMode( NONE );
	while( Switch_GetIsPush() == -1 ) {
		printf("%f, %f\n\r", Wall_GetEdgeDistance(LEFT), Wall_GetEdgeDistance(RIGHT));
	}
}

/* ---------------------------------------------------------------
	直進性の確認関数
--------------------------------------------------------------- */
void Adjust_RunAlignment( void )
{
	Motor_SetDuty_Right( 80 );
	Motor_SetDuty_Left(  80 );
	Switch_WaitFrontSensor();
	Motor_StopPWM();
}

/* ---------------------------------------------------------------
	スラロームの調整関数
--------------------------------------------------------------- */
void Adjust_RunSlalom( int8_t type, int8_t direction, int8_t param )
{
	float 	turn_velocity;
	float	after_distance;

	turn_velocity = Motion_GetSlalomVelocity( type, param );
	after_distance = Motion_GetSlalomAfterDistance( type, direction, param );

	Control_SetMode( ADJUST );

	// 探索スラローム
	if( type == turn_90 ) {
		Motion_StartStraight( 4000.f, 4000.f, turn_velocity, turn_velocity, 45.f + START_OFFSET );
		Motion_WaitStraight();
		Motion_StartSlalom( type, direction, param );
		Motion_WaitSlalom( turn_0, direction, param );
		Control_SetMode( ADJUST );
		Motion_StartStraight( 4000.f, 4000.f, turn_velocity, 0.f, 45.f );

	// 大回り90度スラローム＆180度スラローム
	} else if( type == turn_large || type == turn_180  ) {
		Motion_StartStraight( 8000.f, 8000.f, turn_velocity, turn_velocity, 90.f + START_OFFSET - 20.f );
		Motion_WaitStraight();
		Motion_StartSlalom( type, direction, param );
		Motion_WaitSlalom( type, direction, param );
		Control_SetMode( ADJUST );
		Motion_StartStraight( 8000.f, 8000.f, turn_velocity, 0.f, 90.f + after_distance );

	// 斜め侵入スラローム
	} else if( type == turn_45in || type == turn_135in ) {
		Motion_StartStraight( 8000.f, 8000.f, turn_velocity, turn_velocity, 90.f + START_OFFSET - 20.f );
		Motion_WaitStraight();
		Motion_StartSlalom( type, direction, param );
		Motion_WaitSlalom( type, direction, param );
		Control_SetMode( ADJUST );
		Motion_StartStraight( 8000.f, 8000.f, turn_velocity, 0.f, 45.f*SQRT2 + after_distance );

	// 斜め脱出スラローム
	} else if( type == turn_45out || type == turn_135out ) {
		Motion_StartStraight( 8000.f, 8000.f, turn_velocity, turn_velocity, 45.f*SQRT2 - 20.f );
		Motion_WaitStraight();
		Motion_StartSlalom( type, direction, param );
		Motion_WaitSlalom( type, direction, param );
		Control_SetMode( ADJUST );
		Motion_StartStraight( 8000.f, 8000.f, turn_velocity, 0.f, 45.f + after_distance );

	// 斜め90度スラローム
	} else if( type == turn_90v ) {
		Motion_StartStraight( 8000.f, 8000.f, turn_velocity, turn_velocity, 45.f*SQRT2 - 20.f );
		Motion_WaitStraight();
		Motion_StartSlalom( type, direction, param );
		Motion_WaitSlalom( type, direction, param );
		Control_SetMode( ADJUST );
		Motion_StartStraight( 8000.f, 8000.f, turn_velocity, 0.f, 45.f*SQRT2 + after_distance );
	}
	Motion_WaitStraight();
	LL_mDelay( 500 );
}

void Adjust_RunSlalomSequence( int8_t type, int8_t direction, int8_t param, int8_t mode )
{
	float 	turn_velocity;

	turn_velocity = Motion_GetSlalomVelocity( type, param );

	Control_SetMode( ADJUST );
	Motion_StartStraight( 4000.f, 4000.f, turn_velocity, turn_velocity, 45.f + START_OFFSET );
	Motion_WaitStraight();
	for( int i = 0; i < 10; i++ ) {
		Motion_StartSlalom( type, direction, param );
		Motion_WaitSlalom( turn_0, direction, param );
		if( mode == true ) {
			direction = (direction + 2) % 4;
		} else;
	}
	Control_SetMode( ADJUST );
	Motion_StartStraight( 4000.f, 4000.f, turn_velocity, 0.f, 45.f );
	Motion_WaitStraight();
	LL_mDelay( 500 );
}

/* ---------------------------------------------------------------
	壁情報と最短走行パスの関数
--------------------------------------------------------------- */
void Adjust_DisplayMazeAndPath( int8_t gx, int8_t gy )
{
/*	int8_t		next_direction = -1;
	t_position 	my;
	t_maze 		start_maze = Maze_GetGlobal( 0, 0 );

	if( start_maze.byte != 0xfd ) {
		Maze_LoadFlash();
		Potential_MakeMap( gx, gy );
	} else;
	// 壁情報の表示
	Maze_Display();

	// 最短パスの計算
	if( Path_GetSequenceNumber() == 0 ) {
		Maze_Reset( FASTEST );
		Potential_MakeMap( gx, gy );
		Position_Reset();
		my = Position_GetMyPlace();
		if( Potential_GetAroundSection(&my, -1) != 0xffff ) {
			Path_Reset();
			my = Position_MoveMyPlace( FRONT );
			do {
				next_direction = Route_GetNextDirection_Adachi( my );
				my = Position_MoveMyPlace( next_direction );
				switch( next_direction ) {
					case FRONT:	Path_SetStraightSection( 2 );			break;
					case RIGHT:	Path_SetTurnSection( turn_90, RIGHT );	break;
					case LEFT:	Path_SetTurnSection( turn_90, LEFT );	break;
					case REAR:											break;
				}
			} while( Position_GetIsGoal(gx, gy) == false );
			Path_SetStraightSection( 2 );
			Path_SetTurnSection( goal, 0 );
			// パスの変換
			Path_ConvertTurnLarge();
			Path_ConvertTurn180();
			Path_ConvertDiagonal();
			//　最短パスの表示
			Path_Display();
		} else;
	} else {
		Path_Display();
	}
*/}
