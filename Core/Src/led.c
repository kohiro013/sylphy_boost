
#include "defines.h"
#include "global.h"
//#include "knight_rider.h"

// LEDの点灯モード列挙
typedef enum {
	NORMAL 			= 0,
	TIMER 			= 1,
	KNIGHT_RIDER	= 2,
} t_led_mode;

// LEDのバイナリ表示用構造体
typedef union {
	uint8_t byte;
	struct {
		uint8_t led0			:1;
		uint8_t led1			:1;
		uint8_t led2			:1;
		uint8_t led3			:1;
		uint8_t dummy			:4;
	} bit;
} t_led_num;

// グローバル変数群
volatile static t_led_mode		led_mode;
volatile static t_led_num 		led_num;
volatile static uint16_t 		led_interval = 0;


/* ---------------------------------------------------------------
	割り込み内でLEDの点灯時間をカウントする関数
--------------------------------------------------------------- */
void LED_UpdateTimer( void )
{
	static uint16_t		led_timer = 0;

	if( led_mode == TIMER ) {
		led_timer++;
		if( led_timer > led_interval ) {
			led_timer = 0;
			if( led_num.bit.led0 ) 	LED_YELLOW_TOGGLE();
			else					LED_YELLOW_OFF();
			if( led_num.bit.led1 ) 	LED_RED_TOGGLE();
			else					LED_RED_OFF();
			if( led_num.bit.led2 ) 	LED_GREEN_TOGGLE();
			else					LED_GREEN_OFF();
			if( led_num.bit.led3 ) 	LED_BLUE_TOGGLE();
			else					LED_BLUE_OFF();
		} else;
	} else {
		if( led_num.bit.led0 ) 	LED_YELLOW_ON();
		else					LED_YELLOW_OFF();
		if( led_num.bit.led1 ) 	LED_RED_ON();
		else					LED_RED_OFF();
		if( led_num.bit.led2 ) 	LED_GREEN_ON();
		else					LED_GREEN_OFF();
		if( led_num.bit.led3 ) 	LED_BLUE_ON();
		else					LED_BLUE_OFF();
	}
}

/* ---------------------------------------------------------------
	バイナリ指定でLEDを点灯させる関数
--------------------------------------------------------------- */
void LED_LightBinary( uint8_t num )
{
	led_mode = NORMAL;
	led_interval = 0;
	led_num.byte = num;
}

/* ---------------------------------------------------------------
	バイナリ指定でLEDをトグル点灯させる関数
--------------------------------------------------------------- */
void LED_ToggleLightBinary( uint8_t num )
{
	led_mode = NORMAL;
	led_interval = 0;
	led_num.byte ^= num;
}

/* ---------------------------------------------------------------
	バイナリ指定でLEDを指定時間点滅させる関数
--------------------------------------------------------------- */
void LED_TimerLightBinary( uint8_t num, uint16_t ms )
{
	led_mode = TIMER;
	led_interval = ms;
	led_num.byte = num;
}

