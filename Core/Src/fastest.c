
#include "defines.h"
#include "global.h"


/* ----------------------------------------------------------------------------------
	最短走行
-----------------------------------------------------------------------------------*/
void Fastest_RunSimple( int8_t param, int8_t is_return )
{
//	int8_t			next_direction = -1;
	t_position		my;

	Motor_SetFrequency( 100 );

	Maze_LoadFlash();
	Maze_Reset( FASTEST );
	Maze_InsertDeadEnd();
	Position_Reset();

	// 最短経路生成
	Path_Reset();
	my = Dijkstra_ConvertPath( GOAL_X, GOAL_Y );

	// 吸引ファンの起動
//	if( param > 1 ) {
		SuctionFan_Start();
		LL_mDelay( 200 );
//	} else;

	// パスに沿って走行開始
	Route_StartPathSequence( param, false );

	// 制御のリセット
	SuctionFan_Stop();
	if( Control_GetMode() != FAULT ) {
		LL_mDelay( 500 );
		Motor_StopPWM();
		LED_LightBinary( 0x01 ); 	LL_mDelay( 100 );
		LED_LightBinary( 0x02 );  	LL_mDelay( 100 );
		LED_LightBinary( 0x04 );  	LL_mDelay( 100 );
		LED_LightBinary( 0x08 );  	LL_mDelay( 100 );
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
			Control_WaitFrontWallCorrection();
		} else;

		// 反転して帰還走行開始
		if( Maze_IsLocal(my.x, my.y, my.dir, RIGHT) == true ) {
			Motion_StartRotate( 90.f, RIGHT );
			Control_WaitFrontWallCorrection();
			Motion_StartRotate( 90.f, RIGHT );
		} else if( Maze_IsLocal(my.x, my.y, my.dir, LEFT) == true ) {
			Motion_StartRotate( 90.f, LEFT );
			Control_WaitFrontWallCorrection();
			Motion_StartRotate( 90.f, LEFT );
		} else {
			Motion_StartRotate( 180.f, RIGHT );
		}
		Control_ResetGyroDeviation();
		Control_ResetAngleDeviation();

		// 吸引ファンの起動
		if( param > 1 ) {
//			SuctionFan_Start();
			LL_mDelay( 200 );
		} else;

		// パスに沿って走行開始
		Route_StartPathSequence( 1, true );

		// 制御のリセット
		SuctionFan_Stop();
		if( Control_GetMode() != FAULT ) {
			LL_mDelay( 500 );
			Motor_StopPWM();
			LED_LightBinary( 0x01 ); 	LL_mDelay( 100 );
			LED_LightBinary( 0x02 );  	LL_mDelay( 100 );
			LED_LightBinary( 0x04 );  	LL_mDelay( 100 );
			LED_LightBinary( 0x08 );  	LL_mDelay( 100 );
			LED_LightBinary( 0x04 );  	LL_mDelay( 100 );
			LED_LightBinary( 0x02 );  	LL_mDelay( 100 );
			LED_LightBinary( 0x01 );  	LL_mDelay( 100 );
			LED_LightBinary( 0x00 );
		} else;
	}
}

