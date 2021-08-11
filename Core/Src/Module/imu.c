
#include "defines.h"
#include "global.h"
#include "ism6dsrx_reg.h"

#define REFFERENCE_NUM		(1000)		// 何回の平均をもってジャイロのリファレンス電圧とするか

// ジャイロ関連マクロ
#define GYRO_Z_SIGN			(1.f)		// ジャイロの出力の符号（自分の座標系に合った方向に、1.0fか－1.0fを掛けて修正する）
#define GYRO_Z_SENSITIVITY	(0.135f)

// 加速度計関連マクロ
#define ACCEL_X_SIGN		(1.f)		// 加速度計の出力の符号（自分の座標系に合った方向に、1.0fか－1.0fを掛けて修正する）
#define ACCEL_X_SENSITIVITY	(0.244f/1000.f)

// ローカル関数宣言
void 	IMU_Communication( uint8_t*, uint8_t*, uint8_t );
void 	IMU_Write1byte( uint8_t , uint8_t );
uint8_t IMU_Read1byte( uint8_t );

// グローバル変数宣言
static uint8_t  imu_address = LSM6DSRX_OUTX_L_G | 0x80;
static uint8_t	imu_value[13];			// value[0]はダミーデータ

static uint16_t	accel_x_value;			// X軸加速度計の生データ
static int16_t	accel_x_reference;		// X軸加速度計のリファレンス
static float	accel_x;

static uint16_t	gyro_z_value;			// Z軸ジャイロの生データ
static int16_t	gyro_z_reference;		// Z軸ジャイロのリファレンス
static float	gyro_z;
static float	angle_z;


/* ---------------------------------------------------------------
	IMUと通信する関数
--------------------------------------------------------------- */
void IMU_Communication(uint8_t *tx_data, uint8_t *rx_data, uint8_t length)
{
	uint8_t count = length;

	// SPIが無効の場合有効化する
	if(LL_SPI_IsEnabled(SPI1) == RESET) {
		LL_SPI_Enable(SPI1);
	} else;

	LL_GPIO_ResetOutputPin(SPI1_CS0_GPIO_Port, SPI1_CS0_Pin);
	while(count > 0) {
		while(LL_SPI_IsActiveFlag_TXE(SPI1) == RESET);
		LL_SPI_TransmitData8(SPI1, *tx_data++);
		while(LL_SPI_IsActiveFlag_RXNE(SPI1) == RESET);
		*rx_data++ = LL_SPI_ReceiveData8(SPI1);
		count--;
	}
	while(LL_SPI_IsActiveFlag_BSY(SPI1) == SET);
	LL_GPIO_SetOutputPin(SPI1_CS0_GPIO_Port, SPI1_CS0_Pin);

	// SPIの無効化
	LL_SPI_Disable(SPI1);
}

/* ---------------------------------------------------------------
	IMUに1byte書き込む関数
--------------------------------------------------------------- */
void IMU_Write1byte( uint8_t addr , uint8_t value )
{
	uint8_t tx_data[2] = {addr & 0x7F, value};
	uint8_t rx_data[2];

	IMU_Communication(tx_data, rx_data, sizeof(tx_data)/sizeof(tx_data[0]));
}

/* ---------------------------------------------------------------
	IMUから1byte読み出す関数
--------------------------------------------------------------- */
uint8_t IMU_Read1byte( uint8_t addr )
{
	uint8_t tx_data[2] = {addr | 0x80, 0x00};
	uint8_t rx_data[2];

	IMU_Communication(tx_data, rx_data, sizeof(tx_data)/sizeof(tx_data[0]));
	return rx_data[1];
}

/* ---------------------------------------------------------------
	動作確認関数（WHO_AM_I(0x6b)を取得する）
--------------------------------------------------------------- */
uint8_t IMU_CheckWHOAMI( void )
{
	// 起動時にCSピンがLowになっていると初回の通信に失敗するためCSピンをHighにする
	LL_GPIO_SetOutputPin(SPI1_CS0_GPIO_Port, SPI1_CS0_Pin);
	LL_mDelay(10);
	return IMU_Read1byte( LSM6DSRX_WHO_AM_I );
}

/* ---------------------------------------------------------------
	初期設定用関数
--------------------------------------------------------------- */
void IMU_StartDMA( void )
{
	LL_GPIO_ResetOutputPin(SPI1_CS0_GPIO_Port, SPI1_CS0_Pin);
	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_0);
	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_3);
}

void IMU_Initialize( void )
{
	// 起動時にCSピンがLowになっていると初回の通信に失敗するためCSピンをHighにする
	LL_GPIO_SetOutputPin(SPI1_CS0_GPIO_Port, SPI1_CS0_Pin);
	LL_mDelay(10);

	IMU_Write1byte(LSM6DSRX_CTRL3_C, 0x01);		// LSM6DSRXをリセット
	LL_mDelay(100);
	while( (IMU_Read1byte(LSM6DSRX_CTRL3_C)&0x01) == 0x01 );

	IMU_Write1byte(LSM6DSRX_CTRL9_XL, 0xE2);	// I3CモードをDisableに設定
	LL_mDelay(1);
	IMU_Write1byte(LSM6DSRX_CTRL4_C, 0x06);		// I2CモードをDisableに設定
	LL_mDelay(1);

	// 加速度計の設定
	IMU_Write1byte(LSM6DSRX_CTRL1_XL, 0xae);	// 加速度計のスケールを±8gに設定
	LL_mDelay(1);								// 加速度計の出力データレートを416Hzに設定
	IMU_Write1byte(LSM6DSRX_CTRL8_XL, 0xb0);	// 加速度計のLPFを100Hzに設定
	LL_mDelay(1);

	// ジャイロの設定
	IMU_Write1byte(LSM6DSRX_CTRL2_G, 0xad);		// ジャイロのスケールを±4000deg/sに設定
												// ジャイロの出力データレートを6.66Hzに設定
	LL_mDelay(100);

	// USARTによるDMAリクエストon
	LL_SPI_EnableDMAReq_TX(SPI1);
	LL_SPI_EnableDMAReq_RX(SPI1);
	LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_0);

	// 受信DMA動作設定
	LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_0);
	LL_DMA_ConfigAddresses(DMA2, LL_DMA_STREAM_0, (uint32_t)LL_SPI_DMA_GetRegAddr(SPI1),
		(uint32_t)imu_value, LL_DMA_GetDataTransferDirection(DMA2, LL_DMA_STREAM_0));
	LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_0, sizeof(imu_value)/sizeof(imu_value[0]) - 1);
	LL_DMA_ClearFlag_TC0(DMA2);

	// 送信DMA動作設定
	LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_3);
	LL_DMA_ConfigAddresses(DMA2, LL_DMA_STREAM_3, (uint32_t)(&imu_address),
		(uint32_t)LL_SPI_DMA_GetRegAddr(SPI1), LL_DMA_GetDataTransferDirection(DMA2, LL_DMA_STREAM_3));
	LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_3, sizeof(imu_value)/sizeof(imu_value[0]) - 1);
	LL_DMA_ClearFlag_TC3(DMA2);

	// DMAの開始
	LL_SPI_Enable(SPI1);
	IMU_StartDMA();
}

/* ---------------------------------------------------------------
	DMA送受信完了後のコールバック関数
--------------------------------------------------------------- */
void IMU_Callback( void )
{
	if(LL_DMA_IsActiveFlag_TC0(DMA2)){
		// 割り込みフラグのクリア
		LL_DMA_ClearFlag_TC0(DMA2);
		LL_DMA_ClearFlag_TC3(DMA2);
		LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_0, sizeof(imu_value)/sizeof(imu_value[0]) - 1);
		LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_3, sizeof(imu_value)/sizeof(imu_value[0]) - 1);

		LL_GPIO_SetOutputPin(SPI1_CS0_GPIO_Port, SPI1_CS0_Pin);
		if( imu_value[8] == 0xff && imu_value[7] == 0x00 ) {
			accel_x_value = 0x0000;
		} else if( imu_value[8] != 0x00 || imu_value[7] != 0xff ) {
			accel_x_value = ( ((uint16_t)imu_value[8]<<8 ) | ( (uint16_t)imu_value[7]&0x00ff ) );
		}

		if( imu_value[6] == 0xff && imu_value[5] == 0x00 ) {
			gyro_z_value = 0x0000;
		} else if( imu_value[6] != 0x00 || imu_value[5] != 0xff ) {
			gyro_z_value =  ( ((uint16_t)imu_value[6]<<8 ) | ( (uint16_t)imu_value[5]&0x00ff ) );
		}

		// 通信再開
		IMU_StartDMA();
	} else;
}

/* ---------------------------------------------------------------
	IMUの更新関数
--------------------------------------------------------------- */
void IMU_Update( void )
{
	accel_x	= ACCEL_X_SIGN * G * ((int16_t)accel_x_value - accel_x_reference) * ACCEL_X_SENSITIVITY;
	gyro_z	= GYRO_Z_SIGN * DEG2RAD(((int16_t)gyro_z_value - gyro_z_reference) * GYRO_Z_SENSITIVITY);
	angle_z	+= gyro_z * SYSTEM_PERIOD;
}

/* ---------------------------------------------------------------
	IMUのリファレンスを補正する関数
--------------------------------------------------------------- */
void IMU_ResetReference( void )
{
	int16_t i;

	for(i = 0; i < REFFERENCE_NUM; i++) {
		LL_mDelay(1);
		accel_x_reference += (int16_t)accel_x_value;
		gyro_z_reference += (int16_t)gyro_z_value;
	}
	accel_x_reference /= REFFERENCE_NUM;
	gyro_z_reference /= REFFERENCE_NUM;
}

/* ---------------------------------------------------------------
	X軸加速度計の加速度を取得する関数[m/s^2]
--------------------------------------------------------------- */
float IMU_GetAccel_X( void )
{
	return accel_x;
}

/* ---------------------------------------------------------------
	Z軸ジャイロの角速度を取得する関数[rad/s]
--------------------------------------------------------------- */
float IMU_GetGyro_Z( void )
{
	return gyro_z;
}

/* ---------------------------------------------------------------
	Z軸ジャイロの角度を取得する関数[rad]
--------------------------------------------------------------- */
float IMU_GetGyroAngle_Z( void )
{
	return angle_z;
}

/* ---------------------------------------------------------------
	Z軸ジャイロの角度に入力する関数[rad]
--------------------------------------------------------------- */
void IMU_SetGyroAngle_Z( float rad )
{
	angle_z = rad;
}

/* ---------------------------------------------------------------
	Z軸ジャイロの角度をリセットする関数[rad/s]
--------------------------------------------------------------- */
void IMU_ResetGyroAngle_Z( void )
{
	angle_z = 0.f;
}

void IMU_OffsetGyroAngle_Z( void )
{
	angle_z -= Vehicle_GetAngle();
}

/* ---------------------------------------------------------------
	デバッグ用
--------------------------------------------------------------- */
void IMU_DebugPrintf( void )
{
	angle_z = 0.f;
	while( Communicate_ReceiceDMA() != 0x1b ) {
		printf("%04x, %5d, %5d, %f | %04x, %5d, %5d, %f\r\n",
				accel_x_value, (int16_t)accel_x_value, accel_x_reference, accel_x,
				gyro_z_value,  (int16_t)gyro_z_value,  gyro_z_reference,  gyro_z);
		LL_mDelay(1);
	}
}
