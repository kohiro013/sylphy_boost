
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
	void 		Communicate_TransmitDMA( uint8_t );
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
	void 		IMU_StartDMA( void );						// 慣性センサの読取りを開始する
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
	int 		module_test( int, char** );					// 全モジュールの動作確認用テスト関数

	// データフラッシュ関数群(flash.c)
	void 		Flash_Lock(void);
	void 		Flash_Unlock(void);
	void 		Flash_EraseSector( uint32_t );
	void 		Flash_WriteData( uint32_t, uint8_t*, uint32_t );
	void 		Flash_ReadData( uint32_t, uint8_t*, uint32_t );

	// NT-Shell関数群(myshell.c)
	void 		Myshell_Execute( void );

	// 車両運動計算関数群(vehicle.c)
	void	 	Vehicle_UpdateDynamics( void );
	void 		Vehicle_AdjustLossTorque( void );
	void 		Vehicle_SetTimer( float );
	void 		Vehicle_SetAcceleration( float );
	void 		Vehicle_SetVelocity( float );
	void 		Vehicle_SetDistance( float );
	void 		Vehicle_SetAngularAcceleration( float );
	void 		Vehicle_SetGap( float );
	float 		Vehicle_GetTimer( void );
	float 		Vehicle_GetAcceleration( void );
	float 		Vehicle_GetVelocity( void );
	float 		Vehicle_GetDistance( void );
	float		Vehicle_GetTotalDistance( void );
	float 		Vehicle_GetAngularAcceleration( void );
	float 		Vehicle_GetAngularVelocity( void );
	float 		Vehicle_GetAngle( void );
	float 		Vehicle_GetGap( void );
	int16_t		Vehicle_GetDuty_Right( void );
	int16_t		Vehicle_GetDuty_Left( void );
	void	 	Vehicle_ResetTimer( void );
	void 		Vehicle_ResetDistance( void );
	void		Vehicle_ResetTotalDistance( void );
	void 		Vehicle_ResetAngle( void );
	void 		Vehicle_ResetStraight( void );
	void 		Vehicle_ResetTurning( void );
	void 		Vehicle_ResetIntegral( void );

	// 車両制御関数群(control.c)
	int8_t 		Control_GetMode( void );
	void 		Control_SetMode( int8_t );
	void 		Control_UpdateDeviation( void );
	float 		Control_GetValue_Velocity( void );
	float 		Control_GetValue_Angular( void );
	float 		Control_GetFilterVelocity( void );
	float 		Control_GetFilterDistance( void );
	void 		Control_ResetFilterDistance( void );
	void 		Control_ResetEncoderDeviation( void );
	void 		Control_ResetGyroDeviation( void );
	void 		Control_ResetAngleDeviation( void );
	void 		Control_ResetSensorDeviation( void );
	void 		Control_ResetFrontSensorDeviation( void );
	float 		Control_GetEncoderDeviationValue( void );
	float 		Control_GetGyroDeviationValue( void );
	float 		Control_GetAngleDeviationValue( void );
	float 		Control_GetSensorDeviationValue( void );
	float 		Control_GetFrontWallVelocityDeviationValue( void );
	float 		Control_GetFrontWallAngularDeviationValue( void );

	// 動作関数群(motion.c)
	float 		mynapier( float );
	void 		Motion_StartStraight( float, float, float, float, float );
	float		Motion_GetStraightTime( float, float, float, float, float );
	float 		Motion_SetStraightAcceleration( float );
	void 		Motion_WaitStraight( void );
	void 		Motion_StartRotate( float, int8_t );

	// スラローム関数群(motion_slalom.c)
	void 		Motion_StartSlalom( int8_t, int8_t, int8_t );
	float 		Motion_SetSlalomAngularAcceleration( float );
	void 		Motion_WaitSlalom( int8_t, int8_t, int8_t );
	void 		Motion_SetSlalomAcceleration( float );
	float 		Motion_GetSlalomTime( int8_t, int8_t );
	float 		Motion_GetSlalomVelocity( int8_t, int8_t );
	float 		Motion_GetSlalomBeforeDistance( int8_t, int8_t, int8_t );
	float 		Motion_GetSlalomAfterDistance( int8_t, int8_t, int8_t );

	// 調整関連関数群(adjust.c)
	void 		Adjust_RunTireDiameter( int8_t );
	void 		Adjust_RunGyroSensitivity( uint8_t, int8_t );
	void 		Adjust_RunSearchWallEdge( void );
	void		Adjust_RunFastestWallEdge( void );
	void 		Adjust_RunDiagonal( int8_t );
	void 		Adjust_RunComb( int8_t );
	void 		Adjust_RunGapWallEdge( void );
	void 		Adjust_RunAlignment( void );
	void 		Adjust_RunSlalom( int8_t, int8_t, int8_t );
	void 		Adjust_RunSlalomSequence( int8_t, int8_t, int8_t, int8_t );
	void 		Adjust_DisplayMazeAndPath( int8_t, int8_t );

	// 壁関連関数群(wall.c)
	void 		Wall_Update( void );
	float 		Wall_GetDistance( uint8_t );
	float 		Wall_GetDistanceDelta( uint8_t );
	float	 	Wall_GetDeviation( uint8_t );
	uint8_t 	Wall_GetIsMaze( uint8_t );
	uint8_t 	Wall_GetEdge( uint8_t );
	float 		Wall_GetEdgeDistance( int8_t );
	float 		Wall_GetEdgeMinDistance( int8_t );
	void 		Wall_ResetEdgeMinDistance( void );
	void 		Wall_ResetEdgeDistance( void );
	void 		Wall_DebugPrintf( void );

	// 自己位置関数群(position.c)
	t_position 	Position_GetMyPlace( void );
	void 		Position_SetMyPlace( t_position );
	void 		Position_Reset( void );
	t_position	Position_RotateMyDirection( int8_t );
	t_position	Position_MoveMyPlace( int8_t );
	int8_t 		Position_GetIsGoal( int8_t, int8_t );

	// 壁情報関数群(wall.c)
	void 		Maze_LoadFlash( void );
	void 		Maze_StoreFlash( void );
	uint8_t 	Maze_RotateBit( uint8_t, int8_t, int8_t );
	t_maze 		Maze_ConvertGlobalAndLocal( t_maze, int8_t );
	t_maze 		Maze_GetGlobal( int8_t, int8_t );
	t_maze 		Maze_GetLocal( t_position* );
	int8_t 		Maze_IsLocal( int8_t, int8_t, int8_t, int8_t );
	int8_t 		Maze_GetIsUnknown( t_position*, int8_t );
	void 		Maze_SetFromSensor( t_position* );
	void 		Maze_ResetBuffer( void );
	void 		Maze_DeleteWhenClash( void );
	void 		Maze_InsertDeadEnd( void );
	void 		Maze_SetPillarAlone( t_position* );
	void 		Maze_Reset( int8_t );
	void 		Maze_Display( void );
	void 		Maze_SetDebugData( void );
	void 		Maze_SetDebugData_32x32( void );

	// ポテンシャルマップ関数群(potential.c)
	uint16_t 	Potential_GetAroundSection( t_position*, int8_t );
	void 		Potential_MakeMap( int8_t, int8_t );
	void 		Potential_MakeUnknownMap( int8_t, int8_t );

#endif /* INC_GLOBAL_H_ */
