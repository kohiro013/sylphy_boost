
#include "defines.h"
#include "global.h"

#define N 		(3000+1)
#define THIN	(1)
#define TYPE	(24)

volatile static uint16_t	count 			= 0;
volatile static int16_t		data[N][TYPE]	= {0};
volatile static uint8_t		header[] =
		"Time, Mode, Battery, Load, Duty_L, Duty_R, Target_V, Measure_V, Target_Omega, Measure_Omega, "
		"Target_D, Measure_D, Target_Theta, Measure_Theta, Sensor_FL, Sensor_SL, Sensor_SR, Sensor_FR, "
		"Edge_SL, Edge_SR, Control_Encoder, Control_Gyro, Control_Angle, Control_Sensor, Gap";

void Log_WriteRecodeData( void )
{
	volatile static int thin_count = 0;

	if( thin_count == 0 ) {
		if( count >= N ) {
			count = 0;
		} else;

		if( Control_GetMode() > NONE ) {
			data[count][0]  = (int16_t)Control_GetMode();
			data[count][1]  = (int16_t)(Battery_GetVoltage()*100.f);
			data[count][2]  = (int16_t)Interrupt_GetDuty();
			data[count][3]  = (int16_t)Vehicle_GetDuty_Left();
			data[count][4]  = (int16_t)Vehicle_GetDuty_Right();
			data[count][5]  = (int16_t)Vehicle_GetVelocity();
			data[count][6]  = (int16_t)Control_GetFilterVelocity();
			data[count][7]  = (int16_t)(RAD2DEG(Vehicle_GetAngularVelocity())*10.f);
			data[count][8]  = (int16_t)(RAD2DEG(IMU_GetGyro_Z())*10.f);
			data[count][9]  = (int16_t)(Vehicle_GetDistance()*10.f);
			data[count][10] = (int16_t)(Control_GetFilterDistance()*10.f);
			data[count][11] = (int16_t)(RAD2DEG(Vehicle_GetAngle())*10.f);
			data[count][12] = (int16_t)(RAD2DEG(IMU_GetGyroAngle_Z())*10.f);
			data[count][13] = (int16_t)(Wall_GetDistance(FRONT + LEFT)*10.f);
			data[count][14] = (int16_t)(Wall_GetDistance(LEFT)*10.f);
			data[count][15] = (int16_t)(Wall_GetDistance(RIGHT)*10.f);
			data[count][16] = (int16_t)(Wall_GetDistance(FRONT + RIGHT)*10.f);
			data[count][17] = (int16_t)Wall_GetEdge(LEFT);
			data[count][18] = (int16_t)Wall_GetEdge(RIGHT);
			data[count][19] = (int16_t)(Control_GetEncoderDeviationValue());
			data[count][20] = (int16_t)(Control_GetGyroDeviationValue()*10.f);
			data[count][21] = (int16_t)(Control_GetAngleDeviationValue()*10.f);
			data[count][22] = (int16_t)(Control_GetSensorDeviationValue()*10.f);
			data[count][23] = (int16_t)(Vehicle_GetGap()*10.f);
			count++;
		} else;
	} else;

	thin_count = (thin_count + 1) % THIN;
}

void Log_ReadRecodeData( void )
{
	int			i;
	int			num;
	int16_t		log_read[TYPE] = {0};

	printf("%s\r\n", header);
	for( num = 0; num < N; num++ ) {
		if( num + count < N ) {
			for( i = 0; i < TYPE; i++ ) {
				log_read[i] = data[num + count][i];
			}
		} else {
			for( i = 0; i < TYPE; i++ ) {
				log_read[i] = data[num + count - N][i];
			}
		}

		printf("%5.3f", num*SYSTEM_PERIOD*THIN);
		for( i = 0; i < TYPE; i++ ) {
			printf(", %6d", log_read[i]);
		}
		printf("\r\n");
	}
	printf("%c\r\n", 0x1b);
}
