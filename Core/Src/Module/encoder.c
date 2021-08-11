
#include "defines.h"
#include "global.h"

const int16_t 		ENC_RESOLUTION = 1024 - 1;

// グローバル変数宣言
static uint16_t 	encoder_address = (0x3fff<<1) | 0x8000;
static uint16_t		encoder_value;

static uint16_t		encoder_count_r 	= 0;
static uint16_t		encoder_count_r_old = 0;
static float		encoder_angle_r		= 0.f;

static uint16_t		encoder_count_l 	= 0;
static uint16_t		encoder_count_l_old = 0;
static float		encoder_angle_l		= 0.f;

/* ---------------------------------------------------------------
	AS5050に2byte書き込む関数
--------------------------------------------------------------- */
void Encoder_Write2byte( uint16_t addr , uint16_t data )
{
	uint16_t address = (addr<<1) & 0x7fff;
	addr |= __builtin_parity(addr);

	// SPIが無効の場合有効化する
	if(LL_SPI_IsEnabled(SPI2) == RESET) {
		LL_SPI_Enable(SPI2);
	} else;

	LL_GPIO_ResetOutputPin(SPI2_CS0_GPIO_Port, SPI2_CS0_Pin);
	while(LL_SPI_IsActiveFlag_TXE(SPI2) == RESET);
	LL_SPI_TransmitData16(SPI2, (uint16_t)address);
	LL_SPI_TransmitData16(SPI2, (uint16_t)data);
	while(LL_SPI_IsActiveFlag_RXNE(SPI2) == RESET);
	while(LL_SPI_IsActiveFlag_BSY(SPI2) == SET);
	LL_GPIO_SetOutputPin(SPI2_CS0_GPIO_Port, SPI2_CS0_Pin);

	LL_GPIO_ResetOutputPin(SPI2_CS1_GPIO_Port, SPI2_CS1_Pin);
	while(LL_SPI_IsActiveFlag_TXE(SPI2) == RESET);
	LL_SPI_TransmitData16(SPI2, (uint16_t)address);
	LL_SPI_TransmitData16(SPI2, (uint16_t)data);
	while(LL_SPI_IsActiveFlag_RXNE(SPI2) == RESET);
	while(LL_SPI_IsActiveFlag_BSY(SPI2) == SET);
	LL_GPIO_SetOutputPin(SPI2_CS1_GPIO_Port, SPI2_CS1_Pin);

	// SPIの無効化
	LL_SPI_Disable(SPI2);
}

/* ---------------------------------------------------------------
	AS5050の初期設定関数
--------------------------------------------------------------- */
void Encoder_Initialize( void )
{
	encoder_address |= __builtin_parity(encoder_address);
	Encoder_Write2byte(0x33A5, 0x0000);
	LL_mDelay(100);

	// USARTによるDMAリクエストon
	LL_SPI_EnableDMAReq_TX(SPI2);
	LL_SPI_EnableDMAReq_RX(SPI2);
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_3);

	// 受信DMA動作設定
	LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_3);
	LL_DMA_ConfigAddresses(DMA1, LL_DMA_STREAM_3, (uint32_t)LL_SPI_DMA_GetRegAddr(SPI2),
		(uint32_t)(&encoder_value), LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_STREAM_3));
	LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_3, 1);
	LL_DMA_ClearFlag_TC3(DMA1);

	// 送信DMA動作設定
	LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_4);
	LL_DMA_ConfigAddresses(DMA1, LL_DMA_STREAM_4, (uint32_t)(&encoder_address),
		(uint32_t)LL_SPI_DMA_GetRegAddr(SPI2), LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_STREAM_4));
	LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_4, 1);
	LL_DMA_ClearFlag_TC4(DMA1);

	// DMAの開始
	LL_SPI_Enable(SPI2);
	LL_GPIO_ResetOutputPin(SPI2_CS0_GPIO_Port, SPI2_CS0_Pin);
	LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_0);
	LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_3);
}

/* ---------------------------------------------------------------
	DMA送受信完了後のコールバック関数
--------------------------------------------------------------- */
void Encoder_Callback( void )
{
	volatile static int8_t switching = LEFT;

	if(LL_DMA_IsActiveFlag_TC3(DMA1)){
		// 割り込みフラグのクリア
		LL_DMA_ClearFlag_TC3(DMA1);
		LL_DMA_ClearFlag_TC4(DMA1);
		LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_3, 1);
		LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_4, 1);

		if( switching == LEFT ) {
			LL_GPIO_SetOutputPin(SPI2_CS1_GPIO_Port, SPI2_CS1_Pin);
			encoder_count_l = (encoder_value>>2)&0x03ff;
			LL_GPIO_ResetOutputPin(SPI2_CS0_GPIO_Port, SPI2_CS0_Pin);
		} else {
			LL_GPIO_SetOutputPin(SPI2_CS0_GPIO_Port, SPI2_CS0_Pin);
			encoder_count_r = ENC_RESOLUTION - ((encoder_value>>2)&0x03ff);
			LL_GPIO_ResetOutputPin(SPI2_CS1_GPIO_Port, SPI2_CS1_Pin);
		}
		switching ^= 1;	// 左右の切り替え

		// 通信再開
		LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_3);
		LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_4);
	} else;
}

/* ---------------------------------------------------------------
	エンコーダの更新関数
--------------------------------------------------------------- */
void Encoder_Update( void )
{
	volatile int32_t	encoder_delta_l		= 0;
	volatile int32_t	encoder_delta_r		= 0;

	// 左エンコーダ
	encoder_delta_l = (int32_t)encoder_count_l - (int32_t)encoder_count_l_old;
	if( ABS(encoder_delta_l) < MIN(ABS(encoder_delta_l - ENC_RESOLUTION), ABS(encoder_delta_l + ENC_RESOLUTION)) ) {
		encoder_angle_l = 2*PI * (float)encoder_delta_l / (float)ENC_RESOLUTION;
	} else {
		if( ABS(encoder_delta_l - ENC_RESOLUTION) < ABS(encoder_delta_l + ENC_RESOLUTION) ) {
			encoder_angle_l = 2*PI * (float)(encoder_delta_l - ENC_RESOLUTION) / (float)ENC_RESOLUTION;
		} else {
			encoder_angle_l = 2*PI * (float)(encoder_delta_l + ENC_RESOLUTION) / (float)ENC_RESOLUTION;
		}
	}

	// 右エンコーダ
	encoder_delta_r = (int32_t)encoder_count_r - (int32_t)encoder_count_r_old;
	if( ABS(encoder_delta_r) < MIN(ABS(encoder_delta_r - ENC_RESOLUTION), ABS(encoder_delta_r + ENC_RESOLUTION)) ) {
		encoder_angle_r = 2*PI * (float)encoder_delta_r / (float)ENC_RESOLUTION;
	} else {
		if( ABS(encoder_delta_r - ENC_RESOLUTION) < ABS(encoder_delta_r + ENC_RESOLUTION) ) {
			encoder_angle_r = 2*PI * (float)(encoder_delta_r - ENC_RESOLUTION) / (float)ENC_RESOLUTION;
		} else {
			encoder_angle_r = 2*PI * (float)(encoder_delta_r + ENC_RESOLUTION) / (float)ENC_RESOLUTION;
		}
	}
}

/* ---------------------------------------------------------------
	右タイヤの位相係数カウントを初期化する関数
--------------------------------------------------------------- */
void Encoder_ResetCount_Right( void )
{
	encoder_count_r_old = encoder_count_r;
}

/* ---------------------------------------------------------------
	左タイヤの位相係数カウントを初期化する関数
--------------------------------------------------------------- */
void Encoder_ResetCount_Left( void )
{
	encoder_count_l_old = encoder_count_l;
}

/* ---------------------------------------------------------------
	右タイヤの角度を取得する関数[rad]
--------------------------------------------------------------- */
float Encoder_GetAngle_Right( void )
{
	return encoder_angle_r;
}

/* ---------------------------------------------------------------
	左タイヤの角度を取得する関数[rad]
--------------------------------------------------------------- */
float Encoder_GetAngle_Left( void )
{
	return encoder_angle_l;
}

/* ---------------------------------------------------------------
	右タイヤの角速度を取得する関数[rad/s]
--------------------------------------------------------------- */
float Encoder_GetAnglerVelocity_Right( void )
{
	return encoder_angle_r / SYSTEM_PERIOD;
}

/* ---------------------------------------------------------------
	左タイヤの角速度を取得する関数[rad/s]
--------------------------------------------------------------- */
float Encoder_GetAnglerVelocity_Left( void )
{
	return encoder_angle_l / SYSTEM_PERIOD;
}

/* ---------------------------------------------------------------
	デバッグ用
--------------------------------------------------------------- */
void Encoder_DebugPrintf( void )
{
	while( Communicate_ReceiceDMA() != 0x1b ) {
//		printf("%5d, %5d\r\n", encoder_count_l, encoder_count_r);
		printf("%5d, %5d, %8.3f\r\n", encoder_count_l, encoder_count_l_old, encoder_angle_l);
//		Encoder_ResetCount_Left();
		LL_mDelay(100);
	}
}

