
#include "defines.h"
#include "global.h"

// ホイールクリック設定
#define ONE_CLICK_ANGLE			(PI/5.f)
#define ONE_CLICK_GAIN			(200)
#define ONE_CLICK_DUTY_LIMIT	(200)

static void 	selectWheelCrickMode( int8_t*, int8_t, int8_t );
static void		resetAllParams( void );
static int8_t	resetStartPreparation( void );

/* ---------------------------------------------------------------
	探索モード
--------------------------------------------------------------- */
void mode_search( int8_t param ) {
	int8_t 	sw_side = resetStartPreparation();
	switch( param ) {
		case 1:
			Search_RunStart( ADACHI, sw_side == RIGHT );
		break;

		case 2:
			Search_RunStart( UNKNOWN, sw_side == RIGHT );
		break;

		case 3:
			Search_RunStart( ALL, sw_side == RIGHT );
		break;

		case 4:

		break;

		case 5:
			Control_SetMode(FWALL);
			while(Control_GetMode() != FAULT && !Switch_GetIsPush());
		break;

		case 6:
			Control_SetMode(TURN);
			while(Control_GetMode() != FAULT && !Switch_GetIsPush());
		break;

		case 7:
			if( sw_side == RIGHT ) {
				Maze_LoadFlash();
			} else if( sw_side == LEFT ) {
				Maze_StoreFlash();
			}
		break;
	}
}

/* ---------------------------------------------------------------
	最短モード
--------------------------------------------------------------- */
void mode_fastest( int8_t param ) {
	int8_t 	sw_side = resetStartPreparation();
	if( sw_side == FRONT ) return;
	switch( param ) {
		case 1:
		case 2:
		case 3:
		case 4:
		case 5:
		case 6:
		case 7:
			Fastest_RunSimple( param, sw_side == RIGHT );
		break;
	}
}

/* ---------------------------------------------------------------
	調整モード
--------------------------------------------------------------- */
void mode_adjust( int8_t param ) {
	int8_t 	sw_side;
	switch( param ) {
		case 1:
		case 2:
		case 3:
			sw_side = resetStartPreparation();
			Fastest_RunCircuit( 9, 9, param-1 );
		break;

		case 4:
			sw_side = resetStartPreparation();
			if( sw_side == FRONT ) {
				LL_mDelay( 500 );
				IMU_ResetReference();
				resetAllParams();
				Adjust_RunTireDiameter( 31 );
			} else {
				Adjust_RunGyroSensitivity(20, sw_side);
			}
		break;

		case 5:
			sw_side = resetStartPreparation();
			if( sw_side == FRONT ) {
				LL_mDelay( 500 );
				IMU_ResetReference();
				resetAllParams();
				Adjust_RunGapWallEdge();
			} else if( sw_side == LEFT ){
				Adjust_RunSearchWallEdge();
			} else {
				Adjust_RunFastestWallEdge();
			}
		break;

		case 6:
			sw_side = resetStartPreparation();
			if( sw_side == FRONT ) {
				LL_mDelay( 500 );
				IMU_ResetReference();
				resetAllParams();
				Adjust_RunPID( 20 );
			} else if( sw_side == LEFT ){
				Adjust_RunComb( 8 );
			} else {
				Adjust_RunDiagonal( 15 );
			}
		break;

		case 7:
			sw_side = resetStartPreparation();
			if( sw_side == FRONT ) {
				LL_mDelay( 500 );			//
				IMU_ResetReference();		//
				resetAllParams();			//
				LED_LightBinary( 0x00 );
				Adjust_RunSlalomSequence( turn_90, RIGHT, 0, true );
			} else {
				Adjust_RunSlalomSequence( turn_90, sw_side, 0, false );
			}
		break;
	}
}

/* ---------------------------------------------------------------
	スラロームの調整モード
--------------------------------------------------------------- */
void mode_turn_adjust( int8_t param ) {
	int8_t 	sw_side = resetStartPreparation();
	if( sw_side == FRONT ) return;
	Adjust_RunSlalom( param+1, sw_side, 0 );
}

/* ---------------------------------------------------------------
	デバッグモード
--------------------------------------------------------------- */
void mode_debug( int8_t param ) {
	resetStartPreparation();
	// 探索
	Search_RunStart( UNKNOWN, true );

	for( int i = 0; i < 4; i++ ) {
		if( Control_GetMode() == FAULT ) break;
		// 前壁補正
		Control_SetMode( FWALL );
		LL_mDelay( 500 );
		Motion_StartRotate( 90.f, RIGHT );
		Control_SetMode( FWALL );
		LL_mDelay( 500 );
		Motion_StartRotate( 90.f, RIGHT );
		// 最短
		resetAllParams();
		Fastest_RunSimple( i, true );
	}
	Motion_StartRotate( 180.f, RIGHT );
}

/* ---------------------------------------------------------------
	モードセレクト
--------------------------------------------------------------- */
void Mode_SelectRunMode( void )
{
	static int8_t	run_mode		= 0;
	static int8_t	param			= 0;
	int8_t			select_mode		= false;
	int8_t			is_switch		= false;

	volatile uint32_t tmp_timer;

	void (*p_function[])(int8_t) = {
		mode_debug, mode_search, mode_fastest, mode_adjust, mode_turn_adjust,
	};

	// スイッチが押下されるまでモード選択
	Control_SetMode( NONE );
	resetAllParams();
	while( 1 ) {
		is_switch = Switch_GetIsPush();
		tmp_timer = Interrupt_GetGlobalTime();
		while( is_switch == false && Interrupt_GetGlobalTime() <= tmp_timer ) {
			is_switch = Switch_GetIsPush();
		}

		// 長押しでデバッグモード
		if( is_switch == 2 ) {
			select_mode = false;
			LED_LightBinary( 0x00 );
			if( select_mode == false ) {
				run_mode = -1;
				break;
			} else {
				select_mode = false;
			}
			while( is_switch == 2 );
		// スイッチ押下でモードとパラメータを決定
		} else if( is_switch == 1 ) {
			if( select_mode == false ) {
				select_mode = true;
				LL_mDelay(10);
			} else {
				break;
			}
		} else;

		if( select_mode == false ) {
			selectWheelCrickMode( &run_mode, 4, LEFT );
			LED_LightBinary( 1 << run_mode );
		} else {
			selectWheelCrickMode( &param, 7, LEFT );
			LED_LightBinary( 0x00 );
			LED_TimerLightBinary( param + 1, 50 );
		}
		Myshell_Execute();
	}
	LED_LightBinary( 0x00 );

	resetAllParams();
	(*p_function[run_mode+1])(param+1);
	resetAllParams();

	LL_mDelay(100);
	while( Control_GetMode() == FAULT ) {
		if( Switch_GetIsPush() != false ) break;
	}
}

/* ---------------------------------------------------------------
	ホイールクリック関数
--------------------------------------------------------------- */
static void selectWheelCrickMode( int8_t* mode, int8_t num_mode, int8_t direction )
{
	float 			enc_angle	= 0.0f;
	int16_t 		duty_click	= 0;

	if( direction == RIGHT )		enc_angle = Encoder_GetAngle_Right();
	else if( direction == LEFT )	enc_angle = Encoder_GetAngle_Left();

	if( enc_angle < -ONE_CLICK_ANGLE ) {
		*mode = (*mode + 1) % num_mode;
		enc_angle = duty_click = 0.0f;
		if( direction == RIGHT )		Encoder_ResetCount_Right();
		else if( direction == LEFT )	Encoder_ResetCount_Left();
	} else if( enc_angle > ONE_CLICK_ANGLE ) {
		*mode = (*mode + (num_mode-1)) % num_mode;
		enc_angle = duty_click = 0.0f;
		if( direction == RIGHT )		Encoder_ResetCount_Right();
		else if( direction == LEFT )	Encoder_ResetCount_Left();
	} else {
		if( ABS(enc_angle) > ONE_CLICK_ANGLE / 2.f ) {
			duty_click = ONE_CLICK_GAIN * (SIGN(enc_angle) * ONE_CLICK_ANGLE - duty_click);
		} else {
			duty_click = -ONE_CLICK_GAIN * enc_angle;
		}
		duty_click = SIGN(duty_click) * MIN(ONE_CLICK_DUTY_LIMIT, ABS(duty_click));
	}

	if( direction == RIGHT )		Motor_SetDuty_Right(duty_click);
	else if( direction == LEFT )	Motor_SetDuty_Left(duty_click);

//	printf("%2d %6.2f %5d\r\n", *mode, RAD2DEG(enc_angle), duty_click);
}

/* ---------------------------------------------------------------
	全リセット
--------------------------------------------------------------- */
static void resetAllParams( void )
{
	Motor_StopPWM();
	Motor_SetFrequency( 400 );
	SuctionFan_Stop();
	Sensor_StopLED();
	Encoder_ResetCount_Right();
	Encoder_ResetCount_Left();
	IMU_ResetGyroAngle_Z();
	Vehicle_ResetTimer();
	Vehicle_ResetStraight();
	Vehicle_ResetTurning();
	Vehicle_ResetIntegral();
	Vehicle_ResetTotalDistance();
	Control_ResetFilterDistance();
	Control_ResetEncoderDeviation();
	Control_ResetGyroDeviation();
	Control_ResetAngleDeviation();
	Control_ResetSensorDeviation();
	Control_ResetFrontSensorDeviation();
	Wall_ResetEdgeDistance();
}

/* ---------------------------------------------------------------
	スタート準備
--------------------------------------------------------------- */
static int8_t resetStartPreparation( void )
{
	int8_t sw_side = Switch_WaitFrontSensor();	//
	if( sw_side == FRONT ) {
		LED_LightBinary( 0x08 + 0x01 );
	} else {
		if( sw_side == RIGHT )	LED_LightBinary( 0x01 );
		else					LED_LightBinary( 0x08 );
		LL_mDelay( 500 );
		IMU_ResetReference();
		resetAllParams();
		LED_LightBinary( 0x00 );
	}
	return sw_side;
}

