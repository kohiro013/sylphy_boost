
#include "defines.h"
#include "global.h"

#define SLIP_RATE			(0.90f)			// タイヤのスリップ率
#define SECTION_WALL_EDGE	(20.f)			// 壁切れ検出区間

const t_init_straight circuit[3] = {
	// 加速度	減速度	  最高速度
	{  6000.0f,  6000.0f, 1000.f },
	{  8000.0f,  8000.0f, 1500.f },
	{ 10000.0f, 10000.0f, 2000.f },
};
const int8_t PARAM_SLALOM = 0;


void Fastest_RunCircuit( int8_t size_x, int8_t size_y, uint8_t param )
{
	float	turn_velocity;
	float	after_distance;

	if( param > 2 ) {
		param = 2;
	} else if( param < 0 ) {
		param = 0;
	} else;

	turn_velocity = Motion_GetSlalomVelocity( turn_large, param + PARAM_SLALOM );
	after_distance = Motion_GetSlalomAfterDistance( turn_large, RIGHT, param + PARAM_SLALOM );

	Control_SetMode( FASTEST );
	Motion_StartStraight( circuit[param].acceleration, circuit[param].deceleration,
		circuit[param].max_velocity, circuit[param].max_velocity, 90.f + START_OFFSET + 30.f );
	Motion_WaitStraight();

	for( int i = 0; i < 4; i++ ) {
		Control_SetMode( FASTEST );
		Motion_StartStraight( circuit[param].acceleration, circuit[param].deceleration,
			circuit[param].max_velocity, turn_velocity, (90.f*(float)(size_y - 3) + after_distance)*SLIP_RATE - SECTION_WALL_EDGE - 30.f );
		Motion_WaitStraight();

		// 各パラメータのリセット
		Vehicle_ResetTurning();
		Vehicle_ResetIntegral();
		IMU_ResetGyroAngle_Z();
		Control_ResetAngleDeviation();

		Motion_StartSlalom( turn_large, RIGHT, param + PARAM_SLALOM );
		Motion_WaitSlalom( turn_large, RIGHT, param + PARAM_SLALOM );

		IMU_OffsetGyroAngle_Z();
		Vehicle_ResetAngle();
		Motion_StartStraight( circuit[param].acceleration, circuit[param].deceleration, turn_velocity, turn_velocity, 30.f );
		Motion_WaitStraight();

		Control_SetMode( FASTEST );
		Motion_StartStraight( circuit[param].acceleration, circuit[param].deceleration,
			circuit[param].max_velocity, turn_velocity, (90.f*(float)(size_x - 3) + after_distance)*SLIP_RATE - SECTION_WALL_EDGE - 30.f );
		Motion_WaitStraight();

		// 各パラメータのリセット
		Vehicle_ResetTurning();
		Vehicle_ResetIntegral();
		IMU_ResetGyroAngle_Z();
		Control_ResetAngleDeviation();

		Motion_StartSlalom( turn_large, RIGHT, param + PARAM_SLALOM );
		Motion_WaitSlalom( turn_large, RIGHT, param + PARAM_SLALOM );

		IMU_OffsetGyroAngle_Z();
		Vehicle_ResetAngle();
		Motion_StartStraight( circuit[param].acceleration, circuit[param].deceleration, turn_velocity, turn_velocity, 30.f );
		Motion_WaitStraight();
	}
	Control_SetMode( FASTEST );
	Motion_StartStraight( circuit[param].acceleration, circuit[param].deceleration,
		circuit[param].max_velocity, 0.f, 270.f + after_distance );
	Motion_WaitStraight();
	LL_mDelay( 500 );
}
