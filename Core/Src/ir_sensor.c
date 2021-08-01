
#include "defines.h"
#include "global.h"

#define NUM_ADC				(12)
#define GET_ADC_DATA(x)		adc_value[x-1]

typedef enum {
	LED_FL_OFF 	= 5,
	LED_FL_ON 	= 6,
	LED_SL_OFF 	= 9,
	LED_SL_ON 	= 10,
	LED_SR_OFF 	= 7,
	LED_SR_ON 	= 8,
	LED_FR_OFF 	= 3,
	LED_FR_ON 	= 4,
} t_sensor_mode;

static uint32_t		led_on_pattern[]  = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};				// LED点灯コマンド
static uint32_t		led_off_pattern[] = {0x00F00000, 0x00F00000, 0x00F00000, 0x00F00000,
										 0x00F00000, 0x00F00000, 0x00F00000, 0x00F00000,
										 0x00F00000, 0x00F00000, 0x00F00000, 0x00F00000};	// LED消灯コマンド
static uint16_t		adc_value[NUM_ADC];		// AD変換値

/* ---------------------------------------------------------------
	赤外センサの出力設定関数
--------------------------------------------------------------- */
void Sensor_TurnOffLED( void )
{
	for( int8_t i = 0; i < NUM_ADC; i++ ) {
		led_on_pattern[i] = 0;
	}
}

void Sensor_TurnOnLED( void )
{
	Sensor_TurnOffLED();
	led_on_pattern[LED_FL_OFF-1] = LED_FL_Pin;
	led_on_pattern[LED_SL_OFF-1] = LED_SL_Pin;
	led_on_pattern[LED_SR_OFF-1] = LED_SR_Pin;
	led_on_pattern[LED_FR_OFF-1] = LED_FR_Pin;
}

/* ---------------------------------------------------------------
	赤外センサの初期設定関数
--------------------------------------------------------------- */
void Sensor_Initialize( void )
{
	// TIMによるDMAリクエストon
	LL_TIM_EnableDMAReq_CC1(TIM1);
	LL_TIM_EnableDMAReq_CC2(TIM1);
	LL_TIM_EnableAllOutputs(TIM1);

	// AD変換用DMAの設定
	LL_DMA_DisableStream(DMA2,LL_DMA_STREAM_4);
	LL_DMA_ConfigAddresses(DMA2, LL_DMA_STREAM_4, LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA),
		(uint32_t)adc_value, LL_DMA_GetDataTransferDirection(DMA2, LL_DMA_STREAM_4));
	LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_4, NUM_ADC);
	LL_DMA_ClearFlag_TC0(DMA2);
	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_4);

	// 点灯用DMAの設定
	LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_1);
	LL_DMA_ConfigAddresses(DMA2, LL_DMA_STREAM_1, (uint32_t)led_on_pattern,
		(uint32_t)(&(GPIOA->BSRR)), LL_DMA_GetDataTransferDirection(DMA2, LL_DMA_STREAM_1));
	LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_1, sizeof(led_on_pattern)/sizeof(led_on_pattern[0]));
	LL_DMA_ClearFlag_TC1(DMA2);
	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_1);

	// 消灯用DMAの設定
	LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_2);
	LL_DMA_ConfigAddresses(DMA2, LL_DMA_STREAM_2, (uint32_t)led_off_pattern,
		(uint32_t)(&(GPIOA->BSRR)), LL_DMA_GetDataTransferDirection(DMA2, LL_DMA_STREAM_2));
	LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_2, sizeof(led_off_pattern)/sizeof(led_off_pattern[0]));
	LL_DMA_ClearFlag_TC2(DMA2);
	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_2);

	// LEDの出力設定
	Sensor_TurnOnLED();

	// AD変換の開始
	LL_ADC_Enable(ADC1);
	LL_TIM_EnableCounter(TIM1);
	LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3);
}

/* ---------------------------------------------------------------
	AD変換を開始する関数
--------------------------------------------------------------- */
void Sensor_StartLED( void )
{
	LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
	LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2);
}

/* ---------------------------------------------------------------
	AD変換を停止する関数
--------------------------------------------------------------- */
void Sensor_StopLED( void )
{
	LL_GPIO_ResetOutputPin(GPIOA, LED_FL_Pin|LED_SL_Pin|LED_SR_Pin|LED_FR_Pin);
	LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH1);
	LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH2);
}

/* ---------------------------------------------------------------
	バッテリのAD値を取得する関数
--------------------------------------------------------------- */
uint16_t Sensor_GetBatteryValue( void )
{
	return( (adc_value[0] + adc_value[10]) / 2 );
}

uint16_t Sensor_GetBoostValue( void )
{
	return( (adc_value[1] + adc_value[11]) / 2 );
}


/* ---------------------------------------------------------------
	赤外センサの偏差値を取得する関数
--------------------------------------------------------------- */
int16_t Sensor_GetValue( uint8_t dir )
{
	switch( dir ) {
		case 3: 	return GET_ADC_DATA(LED_FL_ON) - GET_ADC_DATA(LED_FL_OFF);	break;
		case 2: 	return GET_ADC_DATA(LED_SL_ON) - GET_ADC_DATA(LED_SL_OFF);	break;
		case 0: 	return GET_ADC_DATA(LED_SR_ON) - GET_ADC_DATA(LED_SR_OFF);	break;
		case 1: 	return GET_ADC_DATA(LED_FR_ON) - GET_ADC_DATA(LED_FR_OFF);	break;
		default:	return -1;													break;
	}
}

/* ---------------------------------------------------------------
	デバッグ用
--------------------------------------------------------------- */
void Sensor_DebugPrintf( void )
{
	Sensor_StartLED();
	while(Communicate_ReceiceDMA() != 0x1b) {
		printf( "%5d (%4d - %3d), %5d (%4d - %3d), %5d (%4d - %3d), %5d (%4d - %3d)\r\n",
				GET_ADC_DATA(LED_FL_ON) - GET_ADC_DATA(LED_FL_OFF), GET_ADC_DATA(LED_FL_ON), GET_ADC_DATA(LED_FL_OFF),
				GET_ADC_DATA(LED_SL_ON) - GET_ADC_DATA(LED_SL_OFF), GET_ADC_DATA(LED_SL_ON), GET_ADC_DATA(LED_SL_OFF),
				GET_ADC_DATA(LED_SR_ON) - GET_ADC_DATA(LED_SR_OFF), GET_ADC_DATA(LED_SR_ON), GET_ADC_DATA(LED_SR_OFF),
				GET_ADC_DATA(LED_FR_ON) - GET_ADC_DATA(LED_FR_OFF), GET_ADC_DATA(LED_FR_ON), GET_ADC_DATA(LED_FR_OFF) );
/*
		printf("%5d, %5d, %5d, %5d, %5d, %5d, %5d, %5d, %5d, %5d, %5d, %5d\r\n",
				adc_value[0], adc_value[1], adc_value[2], adc_value[3], adc_value[4], adc_value[5],
				adc_value[6], adc_value[7], adc_value[8], adc_value[9], adc_value[10], adc_value[11]);
*/	}
	Sensor_StopLED();
}
