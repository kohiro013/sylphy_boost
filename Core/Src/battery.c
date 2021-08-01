
#include "defines.h"
#include "global.h"

#define BATTERY_REFERENCE	(3.0f)
#define BATTERY_LIMIT		(3.4f)

#define	BATTERY_R1			(9.75f)
#define	BATTERY_R2			(9.93f)

#define	BOOST_R1			(13.50f)
#define	BOOST_R2			(9.32f)

/* ---------------------------------------------------------------
	バッテリの電圧を取得する関数
--------------------------------------------------------------- */
float Battery_GetVoltage( void )
{
	return (BATTERY_REFERENCE * ((BATTERY_R1 + BATTERY_R2) / BATTERY_R2) * (float)Sensor_GetBatteryValue()) / 4096.f;
}

float Battery_GetBoostVoltage( void )
{
	return (BATTERY_REFERENCE * ((BOOST_R1 + BOOST_R2) / BOOST_R2) * (float)Sensor_GetBoostValue()) / 4096.f;
}

/* ---------------------------------------------------------------
	バッテリの電圧制限関数
--------------------------------------------------------------- */
void Battery_LimiterVoltage( void )
{
	volatile int	i;
	volatile float	battery_voltage_average;

	for( i = 0; i < 10; i++) {
		LL_mDelay(10);
		battery_voltage_average += Battery_GetVoltage();
	}
	battery_voltage_average /= 10;

	if( battery_voltage_average < BATTERY_LIMIT ) {
		while( 1 ) {
			LED_LightBinary( 0x05 );
			LL_mDelay(300);
			LED_LightBinary( 0x02 );
			LL_mDelay(300);
		}
	} else;
}
