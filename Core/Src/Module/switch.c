
#include "defines.h"
#include "global.h"

#define SWTICH_IsPush()		LL_GPIO_IsInputPinSet(SWITCH_GPIO_Port, SWITCH_Pin)

// プッシュスイッチの設定
#define SW_SHORT_PUSH		(10)
#define SW_LONG_PUSH		(500)

// センサースイッチの設定
#define SW_THRESHOLD		(800)		// スイッチ判定の閾値
#define SW_TIME				(300)		// スイッチ判定の時間閾値

volatile static uint32_t	push_switch_timer 	= 0;		// スイッチが押されている時間
volatile static int8_t		is_push_switch 		= false;	// スイッチの判定用フラグ

volatile static uint32_t 	sensor_switch_timer = 0;
volatile static int8_t 		is_sensor_switch 	= -1;

/* ---------------------------------------------------------------
	割り込み内でスイッチが反応している時間をカウントする関数
--------------------------------------------------------------- */
void Switch_UpdateTimer( void )
{
	// プッシュスイッチが押下されている時間をカウントする
	if( SWTICH_IsPush() == true ) {
		push_switch_timer ++;
	} else {
		push_switch_timer = 0;
	}

	// 両方の前センサが反応している時間をカウントする
	if( ( Sensor_GetValue(FRONT + RIGHT) > SW_THRESHOLD ) || ( Sensor_GetValue(FRONT + LEFT) > SW_THRESHOLD ) ) {
		if( is_sensor_switch == -1 ) {
			sensor_switch_timer ++;
		} else {
			sensor_switch_timer = 0;
		}
	} else {
		sensor_switch_timer = 0;
		is_sensor_switch = -1;
	}
}

/* ---------------------------------------------------------------
	スイッチが押された時間が指定した時間を超えるとtrueを返す関数
--------------------------------------------------------------- */
int8_t Switch_GetIsPush( void )
{
	if( SWTICH_IsPush() == true ) {
		if( push_switch_timer > SW_LONG_PUSH ) {
			return 2;
		} else {
			is_push_switch = false;
		}
	} else if( push_switch_timer > SW_SHORT_PUSH ) {
		is_push_switch = 1;
	} else {
		is_push_switch = false;
	}

	return is_push_switch;
}

/* ---------------------------------------------------------------
	両方の前センサが反応するまで待機
--------------------------------------------------------------- */
int8_t Switch_GetIsFrontSensor( void )
{
	if( sensor_switch_timer > SW_TIME ) {
		if( ( Sensor_GetValue(FRONT + RIGHT) > SW_THRESHOLD ) && ( Sensor_GetValue(FRONT + LEFT) > SW_THRESHOLD ) ) {
			is_sensor_switch = FRONT;
		} else if( Sensor_GetValue(FRONT + RIGHT) > SW_THRESHOLD ) {
			is_sensor_switch = RIGHT;
		} else if( Sensor_GetValue(FRONT + LEFT) > SW_THRESHOLD ) {
			is_sensor_switch = LEFT;
		} else;
	} else {
		is_sensor_switch = -1;
	}
	return is_sensor_switch;
}

/* ---------------------------------------------------------------
	両方の前センサが反応するまで待機
--------------------------------------------------------------- */
int8_t Switch_WaitFrontSensor( void )
{
	Sensor_StartLED();
	while( Switch_GetIsFrontSensor() != -1 );
	while( 1 ) {
		if( Switch_GetIsFrontSensor() != -1 ) {
			return is_sensor_switch;
		} else;
	}
	Sensor_StopLED();
}
