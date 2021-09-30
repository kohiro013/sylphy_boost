
#include "defines.h"
#include "global.h"

#define STR(var) #var   //引数にした変数を変数名を示す文字列リテラルとして返すマクロ関数

#define NUM 	(3000+1)
#define THIN	(1)

typedef enum {
	Mode,
	Battery, Boost,
	Load,
	Duty_L, Duty_R,
	Target_V, Measure_V,
	Target_Omega, Measure_Omega,
	Target_D, Measure_D,
	Target_Theta, Measure_Theta,
	Sensor_FL, Sensor_SL, Sensor_SR, Sensor_FR,
	Edge_SL, Edge_SR,
	Control_Encoder,
	Control_Gyro, Control_Angle, Control_Sensor,
	Gap,
	NUM_HEADER
} header;

volatile static uint16_t	count 					= 0;
volatile static int16_t		data[NUM][NUM_HEADER]	= {0};

void Log_WriteRecodeData( void )
{
	volatile static int thin_count = 0;

	if( thin_count == 0 ) {
		if( count >= NUM ) {
			count = 0;
		} else;

		if( Control_GetMode() > NONE ) {
			data[count][Mode]  			 = (int16_t)Control_GetMode();
			data[count][Battery]  		 = (int16_t)(Battery_GetVoltage()*100.f);
			data[count][Boost] 	 		 = (int16_t)(Battery_GetBoostVoltage()*100.f);
			data[count][Load]  			 = (int16_t)Interrupt_GetDuty();
			data[count][Duty_L]  		 = (int16_t)Vehicle_GetDuty_Left();
			data[count][Duty_R]  		 = (int16_t)Vehicle_GetDuty_Right();
			data[count][Target_V]  		 = (int16_t)Vehicle_GetVelocity();
			data[count][Measure_V]  	 = (int16_t)Control_GetFilterVelocity();
			data[count][Target_Omega]  	 = (int16_t)(RAD2DEG(Vehicle_GetAngularVelocity())*10.f);
			data[count][Measure_Omega]   = (int16_t)(RAD2DEG(IMU_GetGyro_Z())*10.f);
			data[count][Target_D] 		 = (int16_t)(Vehicle_GetDistance()*10.f);
			data[count][Measure_D] 		 = (int16_t)(Control_GetFilterDistance()*10.f);
			data[count][Target_Theta] 	 = (int16_t)(RAD2DEG(Vehicle_GetAngle())*10.f);
			data[count][Measure_Theta] 	 = (int16_t)(RAD2DEG(IMU_GetGyroAngle_Z())*10.f);
			data[count][Sensor_FL] 		 = (int16_t)(Wall_GetDistance(FRONT + LEFT)*10.f);
			data[count][Sensor_SL] 		 = (int16_t)(Wall_GetDistance(LEFT)*10.f);
			data[count][Sensor_SR] 		 = (int16_t)(Wall_GetDistance(RIGHT)*10.f);
			data[count][Sensor_FR] 		 = (int16_t)(Wall_GetDistance(FRONT + RIGHT)*10.f);
			data[count][Edge_SL] 		 = (int16_t)Wall_GetEdge(LEFT);
			data[count][Edge_SR] 		 = (int16_t)Wall_GetEdge(RIGHT);
			data[count][Control_Encoder] = (int16_t)(Control_GetEncoderDeviationValue());
			data[count][Control_Gyro] 	 = (int16_t)(Control_GetGyroDeviationValue()*10.f);
			data[count][Control_Angle] 	 = (int16_t)(Control_GetAngleDeviationValue()*10.f);
			data[count][Control_Sensor]	 = (int16_t)(Control_GetSensorDeviationValue()*10.f);
			data[count][Gap] 			 = (int16_t)(Vehicle_GetGap()*10.f);
			count++;
		} else;
	} else;

	thin_count = (thin_count + 1) % THIN;
}

void Log_ReadRecodeData( void )
{
	int			i;
	int			num;
	int16_t		log_read[NUM_HEADER] = {0};

	printf("Time");
	for( i = 0; i < NUM_HEADER; i++ ) {
		printf(", ");
		switch(i) {
			case Mode:			 printf("%s", STR(Mode)); 			break;
			case Battery: 		 printf("%s", STR(Battery));		break;
			case Boost:			 printf("%s", STR(Boost)); 			break;
			case Load:			 printf("%s", STR(Load)); 			break;
			case Duty_L:		 printf("%s", STR(Duty_L)); 		break;
			case Duty_R:		 printf("%s", STR(Duty_R)); 		break;
			case Target_V:		 printf("%s", STR(Target_V)); 		break;
			case Measure_V:		 printf("%s", STR(Measure_V)); 		break;
			case Target_Omega:	 printf("%s", STR(Target_Omega)); 	break;
			case Measure_Omega:	 printf("%s", STR(Measure_Omega)); 	break;
			case Target_D:		 printf("%s", STR(Target_D)); 		break;
			case Measure_D:		 printf("%s", STR(Measure_D)); 		break;
			case Target_Theta:	 printf("%s", STR(Target_Theta)); 	break;
			case Measure_Theta:	 printf("%s", STR(Measure_Theta)); 	break;
			case Sensor_FL:		 printf("%s", STR(Sensor_FL)); 		break;
			case Sensor_SL:		 printf("%s", STR(Sensor_SL)); 		break;
			case Sensor_SR:		 printf("%s", STR(Sensor_SR)); 		break;
			case Sensor_FR:		 printf("%s", STR(Sensor_FR)); 		break;
			case Edge_SL:		 printf("%s", STR(Edge_SL)); 		break;
			case Edge_SR:		 printf("%s", STR(Edge_SR)); 		break;
			case Control_Encoder:printf("%s", STR(Control_Encoder));break;
			case Control_Gyro:	 printf("%s", STR(Control_Gyro)); 	break;
			case Control_Angle:	 printf("%s", STR(Control_Angle)); 	break;
			case Control_Sensor: printf("%s", STR(Control_Sensor)); break;
			case Gap:			 printf("%s", STR(Gap)); 			break;
			default:			 printf("undefined");				break;
		}
	}
	printf("\r\n");

	for( num = 0; num < NUM; num++ ) {
		if( num + count < NUM ) {
			for( i = 0; i < NUM_HEADER; i++ ) {
				log_read[i] = data[num + count][i];
			}
		} else {
			for( i = 0; i < NUM_HEADER; i++ ) {
				log_read[i] = data[num + count - NUM][i];
			}
		}

		printf("%5.3f", num*SYSTEM_PERIOD*THIN);
		for( i = 0; i < NUM_HEADER; i++ ) {
			printf(", %6d", log_read[i]);
		}
		printf("\r\n");
	}
	printf("%c\r\n", 0x1b);
	Communicate_ClearReceiveBuffer();
}
