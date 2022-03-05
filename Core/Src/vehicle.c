
#include "defines.h"
#include "global.h"

#define KT						(0.594f)	// トルク定数[mNm/A]
#define KE						(0.062f)	// 逆起電圧定数[mV/rpm]
#define MOT_RESIST				(5.0f)		// 巻線抵抗[Ω]
#define GEAR_RATIO				(37/9)		// ギア比

#define TORQUE_R_LOSS			(20.f)		// 右タイヤの損失トルク補償
#define TORQUE_L_LOSS			(31.f)		// 左タイヤの損失トルク補償
#define MOT_DUTY_MIN			(36)		// モータドライバのデッドタイム補償(400kHz)
											// 90ns * 400kHz = 0.036

volatile static float			t;
volatile static float			a, v, d;
volatile static float			alpha, omega, theta;
volatile static float			gap;
volatile static int16_t			duty_l, duty_r;

volatile static float			total_distance;


/* ---------------------------------------------------------------
	車両運動計算関数
--------------------------------------------------------------- */
void Vehicle_UpdateDynamics( void )
{
	volatile static float	v_battery;
	volatile float			motor_rpm_l, motor_rpm_r;
	volatile float			torque_l, torque_r;

	t += SYSTEM_PERIOD;

	if( Control_GetMode() != TURN && Control_GetMode() != ROTATE ) {
		a = Motion_SetStraightAcceleration(t);
	} else;
	v += a * SYSTEM_PERIOD;
	d += v * SYSTEM_PERIOD;
	total_distance += v * SYSTEM_PERIOD;

	if( Control_GetMode() == TURN || Control_GetMode() == ROTATE ) {
		alpha = Motion_SetSlalomAngularAcceleration(t);
	} else;
	omega += alpha * SYSTEM_PERIOD;
	theta += omega * SYSTEM_PERIOD;

	if( Control_GetMode() == SEARCH || Control_GetMode() == FASTEST ) {
		if( (Wall_GetDistance(FRONT+LEFT) > 45.f) || (Wall_GetDistance(FRONT+RIGHT) > 45.f) ) {
			if( Wall_GetEdge(LEFT) == true || Wall_GetEdge(RIGHT) == true ) {
				if( ABS(Wall_GetEdgeDistance(0xff)) < 25.f && total_distance > 45.f + START_OFFSET ) {
					if( Control_GetMode() == FASTEST ) {
						gap = -42.f * Wall_GetEdgeDistance(0xff) / (90.f - 7.f);
					} else;

					if( Wall_GetEdgeDistance(0xff) != 0.f ) {
						IMU_SetGyroAngle_Z(theta - atanf((sqrtf(7056.f + Wall_GetEdgeDistance(0xff) * Wall_GetEdgeDistance(0xff)) - 84.f) / Wall_GetEdgeDistance(0xff)));
					} else;
				} else;
			} else {
				if( ABS(gap) > 0.5f ) {
					gap += Control_GetFilterVelocity() * arm_sin_f32(theta - IMU_GetGyroAngle_Z()) * SYSTEM_PERIOD;
				} else;
			}
		} else;
	} else {
		gap = 0.f;
	}
	// 制御量の計算
	Control_UpdateDeviation();

	v_battery = 0.9f * v_battery + 0.1f * Battery_GetBoostVoltage();

	torque_l = TIRE/2.0f * ( MASS * (a + Control_GetValue_Velocity()) - INERTIA * (alpha + Control_GetValue_Angular()) / TREAD ) / 2.0f / GEAR_RATIO / 1000.0f;
	torque_r = TIRE/2.0f * ( MASS * (a + Control_GetValue_Velocity()) + INERTIA * (alpha + Control_GetValue_Angular()) / TREAD ) / 2.0f / GEAR_RATIO / 1000.0f;

	motor_rpm_l = ( v - omega * TREAD ) / TIRE * 60/PI * GEAR_RATIO;
	motor_rpm_r = ( v + omega * TREAD ) / TIRE * 60/PI * GEAR_RATIO;

	if( motor_rpm_r == 0.f ) {
		duty_r = ( (MOT_RESIST * (torque_r + SIGN(torque_r) * TORQUE_R_LOSS) / KT + KE * motor_rpm_r) / v_battery );
	} else {
		duty_r = ( (MOT_RESIST * (torque_r + SIGN(Encoder_GetAnglerVelocity_Right()) * TORQUE_R_LOSS) / KT + KE * motor_rpm_r) / v_battery );
	}
	if( motor_rpm_l == 0.f ) {
		duty_l = ( (MOT_RESIST * (torque_l + SIGN(torque_l) * TORQUE_L_LOSS) / KT + KE * motor_rpm_l) / v_battery );
	} else {
		duty_l = ( (MOT_RESIST * (torque_l + SIGN(Encoder_GetAnglerVelocity_Left( )) * TORQUE_L_LOSS) / KT + KE * motor_rpm_l) / v_battery );
	}

	if( Control_GetMode() == FAULT ) {
		Motor_StopPWM();
	} else {
		Motor_SetDuty_Left(  duty_l + SIGN(duty_l) * MOT_DUTY_MIN * 120.f/(LL_TIM_GetAutoReload(TIM4)+1) );
		Motor_SetDuty_Right( duty_r + SIGN(duty_r) * MOT_DUTY_MIN * 120.f/(LL_TIM_GetAutoReload(TIM4)+1) );
	}
}

/* ---------------------------------------------------------------
	各車両状態変数の入力関数
--------------------------------------------------------------- */
void Vehicle_SetTimer( float value )
{
	t = value;
}

void Vehicle_SetAcceleration( float value )
{
	a = value;
}

void Vehicle_SetVelocity( float value )
{
	v = value;
}

void Vehicle_SetDistance( float value )
{
	d = value;
}

void Vehicle_SetAngularAcceleration( float value )
{
	alpha = value;
}

void Vehicle_SetGap( float value )
{
	gap = value;
}

/* ---------------------------------------------------------------
	各車両状態変数の取得関数
--------------------------------------------------------------- */
float Vehicle_GetTimer( void )
{
	return t;
}

float Vehicle_GetAcceleration( void )
{
	return a;
}

float Vehicle_GetVelocity( void )
{
	return v;
}

float Vehicle_GetDistance( void )
{
	return d;
}

float Vehicle_GetTotalDistance( void )
{
	return total_distance;
}

float Vehicle_GetAngularAcceleration( void )
{
	return alpha;
}

float Vehicle_GetAngularVelocity( void )
{
	return omega;
}

float Vehicle_GetAngle( void )
{
	return theta;
}

float Vehicle_GetGap( void )
{
	return gap;
}

int16_t Vehicle_GetDuty_Right( void )
{
	return duty_r;
}

int16_t Vehicle_GetDuty_Left( void )
{
	return duty_l;
}

/* ---------------------------------------------------------------
	各車両状態変数の初期化関数
--------------------------------------------------------------- */
void Vehicle_ResetTimer( void )
{
	t = 0.0f;
}

void Vehicle_ResetDistance( void )
{
	d = 0.f;
}

void Vehicle_ResetTotalDistance( void )
{
	total_distance = 0.f;
}

void Vehicle_ResetAngle( void )
{
	theta = 0.f;
}

void Vehicle_ResetStraight( void )
{
	a = v = d = 0.0f;
	gap = 0.f;
}

void Vehicle_ResetTurning( void )
{
	//alpha = omega = theta = 0.0f;
	alpha = omega = 0.0f;
	gap = 0.f;
}

void Vehicle_ResetIntegral( void )
{
	d = theta = 0.0f;
	gap = 0.f;
}

/* ---------------------------------------------------------------
	損失トルク調整関数
--------------------------------------------------------------- */
void Vehicle_AdjustLossTorque( void )
{
	volatile float		torque_r;
	volatile float		torque_l;
	volatile float		v_battery;

	printf("Adjust Loss Torque Mode\r\n");
	while( Communicate_ReceiceDMA() != 0x1b ) {
		// 損失トルクの入力
		scanf("%f, %f", &torque_l, &torque_r);
		if(0.f < torque_l && torque_l < 100.f && 0.f < torque_r && torque_r < 100.f) {
			// 入力したトルクの表示
			printf("Left : %4.1f [mN/m], Right : %4.1f [mN/m]\r\n", torque_l, torque_r);
			// 無負荷回転させる
			v_battery = Battery_GetBoostVoltage();
			Motor_SetDuty_Right( (int16_t)((MOT_RESIST * torque_r) / KT / v_battery) + MOT_DUTY_MIN );
			Motor_SetDuty_Left(  (int16_t)((MOT_RESIST * torque_l) / KT / v_battery) + MOT_DUTY_MIN );
		} else;
	}
}
