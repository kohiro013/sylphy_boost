
#ifndef INC_GLOBAL_H_
#define INC_GLOBAL_H_

	#include "main.h"
	#include "adc.h"
	#include "dma.h"
	#include "spi.h"
	#include "tim.h"
	#include "usart.h"
	#include "gpio.h"
	#include "defines.h"

#ifdef  GLOBAL_DEFINE
#define GLOBAL
#else
#define GLOBAL	extern
#endif

	// LEDマクロ関数群
	#define LED_YELLOW_ON()			LL_GPIO_SetOutputPin(	LED_YELLOW_GPIO_Port, 	LED_YELLOW_Pin)	// 黄LEDを点灯する
	#define LED_YELLOW_OFF()		LL_GPIO_ResetOutputPin(	LED_YELLOW_GPIO_Port, 	LED_YELLOW_Pin)	// 黄LEDを消灯する
	#define LED_YELLOW_TOGGLE()		LL_GPIO_TogglePin(		LED_YELLOW_GPIO_Port, 	LED_YELLOW_Pin)	// この関数を呼ぶたびに黄LEDの点灯と消灯を切り替える
	#define LED_RED_ON()			LL_GPIO_SetOutputPin(	LED_RED_GPIO_Port, 		LED_RED_Pin)	// 赤LEDを点灯する
	#define LED_RED_OFF()			LL_GPIO_ResetOutputPin(	LED_RED_GPIO_Port, 		LED_RED_Pin)	// 赤LEDを消灯する
	#define LED_RED_TOGGLE()		LL_GPIO_TogglePin(		LED_RED_GPIO_Port, 		LED_RED_Pin)	// この関数を呼ぶたびに赤LEDの点灯と消灯を切り替える
	#define LED_GREEN_ON()			LL_GPIO_SetOutputPin(	LED_GREEN_GPIO_Port,  	LED_GREEN_Pin)	// 緑LEDを点灯する
	#define LED_GREEN_OFF()			LL_GPIO_ResetOutputPin(	LED_GREEN_GPIO_Port,  	LED_GREEN_Pin)	// 緑LEDを消灯する
	#define LED_GREEN_TOGGLE()		LL_GPIO_TogglePin(		LED_GREEN_GPIO_Port,  	LED_GREEN_Pin)	// この関数を呼ぶたびに緑LEDの点灯と消灯を切り替える
	#define LED_BLUE_ON()			LL_GPIO_SetOutputPin(	LED_BLUE_GPIO_Port, 	LED_BLUE_Pin)	// 青LEDを点灯する
	#define LED_BLUE_OFF()			LL_GPIO_ResetOutputPin(	LED_BLUE_GPIO_Port, 	LED_BLUE_Pin)	// 青LEDを消灯する
	#define LED_BLUE_TOGGLE()		LL_GPIO_TogglePin(		LED_BLUE_GPIO_Port, 	LED_BLUE_Pin)	// この関数を呼ぶたびに青LEDの点灯と消灯を切り替える

	// UART通信関数群
	uint8_t		Communicate_Receive1byte( void );
	uint8_t 	Communicate_ReceiceDMA( void );
	void 		Communicate_ClearReceiveBuffer( void );



#endif /* INC_GLOBAL_H_ */
