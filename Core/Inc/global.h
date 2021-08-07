
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

	// LED関数群(led.c)
	void 		LED_UpdateTimer( void );
	void 		LED_LightBinary( uint8_t );
	void 		LED_ToggleLightBinary( uint8_t );
	void 		LED_TimerLightBinary( uint8_t, uint16_t );

	// スイッチ関数群(switch.c)
	void 		Switch_UpdateTimer(void);					// スイッチ反応中の時間を更新する
	int8_t 		Switch_GetIsPush( void );					// プッシュスイッチの押下状態を取得する
	int8_t 		Switch_GetIsFrontSensor( void );			// 前センサの反応状態を取得する
	int8_t 		Switch_WaitFrontSensor( void );				// 前センサが反応するまで待機する

	// UART通信関数群
	uint8_t		Communicate_Receive1byte( void );
	uint8_t 	Communicate_ReceiceDMA( void );
	void 		Communicate_ClearReceiveBuffer( void );

	// モータ関数群
	void 		Motor_StopPWM( void );						// モータを停止
	void 		Motor_SetDuty_Right( int16_t );				// 右モータを指定したDutyで回転させる[0-1000]
	void 		Motor_SetDuty_Left( int16_t );				// 左モータを指定したDutyで回転させる[0-1000]

	// 赤外センサ関数群(ir_sensor.c)
	void 		Sensor_StartLED( void );					// AD変換を開始する
	void 		Sensor_StopLED( void );						// AD変換を停止する
	uint16_t 	Sensor_GetBatteryValue( void );				// 電源電圧のAD値を取得する
	uint16_t 	Sensor_GetBoostValue( void );
	int16_t 	Sensor_GetValue( uint8_t );					// 赤外センサのLEDオンオフ差分値を取得する
															// 0:前右、1:横右、2:横左、3:前左
	void 		Sensor_DebugPrintf( void );

	// バッテリー関数群(battery.c)
	float 		Battery_GetVoltage( void );					// バッテリの電圧を取得する[V]
	float 		Battery_GetBoostVoltage( void );
	void 		Battery_LimiterVoltage( void );				// バッテリの電圧が3.2V以下になると起動しないように制限する

	// 慣性センサ関数群(imu.c)
	void		IMU_Update( void );							// 慣性センサの値を更新する
	uint8_t		IMU_CheckWHOAMI( void );					// 慣性センサの動作確認関数
	void 		IMU_ResetReference( void );					// 慣性センサのリファレンスを補正する
	float 		IMU_GetAccel_X( void );						// X軸加速度計の加速度を取得する[m/s^2]
	float 		IMU_GetGyro_Z( void );						// Z軸ジャイロの角速度を取得する[rad/s]
	float 		IMU_GetGyroAngle_Z( void );					// Z軸ジャイロの角度を取得する[rad]
	void 		IMU_SetGyroAngle_Z( float );
	void 		IMU_ResetGyroAngle_Z( void );				// Z軸ジャイロの角度をリセットする[rad]
	void 		IMU_OffsetGyroAngle_Z( void );
	void 		IMU_DebugPrintf( void );

	// エンコーダ関数群(encoder.c)
	void		Encoder_Update( void );						// エンコーダの値を更新する
	void 		Encoder_ResetCount_Left( void );			// 左エンコーダのカウントを初期値にする
	void 		Encoder_ResetCount_Right( void );			// 右エンコーダのカウントを初期値にする
	float 		Encoder_GetAngle_Left( void );				// 左タイヤの角度を取得する[rad]
	float 		Encoder_GetAngle_Right( void );				// 右タイヤの角度を取得する[rad]
	float 		Encoder_GetAnglerVelocity_Left( void );		// 左タイヤの角速度を取得する[rad/s]
	float 		Encoder_GetAnglerVelocity_Right( void );	// 右タイヤの角速度を取得する[rad/s]
	void 		Encoder_DebugPrintf( void );

	// 割り込み関数群(interrupt.c)
	uint32_t 	Interrupt_GetGlobalTime( void );			// 絶対時間を取得する
	int32_t 	Interrupt_GetDuty( void );					// 割り込み処理内の計算割合を取得する
	int32_t 	Interrupt_GetDuty_Max( void );				// 割り込み処理内の最大計算割合を取得する
	float		Interrupt_GetBootTime( void );				// マイコンが起動してから経過した時間を取得する[s]

	// モジュールテスト関数群(module_test.c)
	void 		module_test( void );						// 全モジュールの動作確認用テスト関数

#endif /* INC_GLOBAL_H_ */
