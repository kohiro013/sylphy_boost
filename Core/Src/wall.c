
#include "defines.h"
#include "global.h"

// キャリブレーション関連定義
#define CORRECT_DISTANCE_MAX			(135.f)
const float	CORRECT_VALUE_MAX[4]		= { 3500, 3488, 3498, 3628 };
const float	CORRECT_VALUE_MIN[4]		= {  258,  288,  356,  348 };

const float	CORRECT_DISTANCE_FRONT[2] 	= { 87.0f, 26.0f };	// 前後の壁に押し当てたときの位置
const float	CORRECT_DISTANCE_SIDE[2] 	= { 65.0f, 19.0f };	// 左右の壁に押し当てたときの位置

// 直進制御関連定義
#define REF_SIDE_L			(42.f)		// 横左センサのリファレンス
#define REF_SIDE_R			(42.f)		// 横右センサのリファレンス

#define TH_CONTROL_SIDE_L	(65.f)		// 左センサの制御閾値
#define TH_CONTROL_SIDE_R	(65.f)		// 右センサの制御閾値

#define TH_CONTROL_CLOSE_FL	(30.f)		// 前壁に近づき過ぎたときの閾値
#define TH_CONTROL_CLOSE_FR	(30.f)		//

#define TH_CONTROL_CUT		(3)			// 現在値と一個前の値との偏差閾値
#define TH_CONTROL_BAND		(10)		// リファレンスのデッドバンド

// 斜め制御関連定義
#define REF_FRONT_L			(130.f)		// 前左センサのリファレンス
#define REF_FRONT_R			(130.f)		// 前右センサのリファレンス

#define TH_CONTROL_FRONT_L	(150.f)		// 前左センサの制御閾値
#define TH_CONTROL_FRONT_R	(150.f)		// 前右センサの制御閾値

// 前壁制御関連定義
#define REF_FWALL_L			(42.f)		// 前左センサのリファレンス
#define REF_FWALL_R			(42.f)		// 前右センサのリファレンス

#define TH_CONTROL_FWALL_L	(60.f)		// 前壁制御の前左センサの閾値
#define TH_CONTROL_FWALL_R	(60.f)		// 前壁制御の前右センサの閾値

// 壁情報関連定義
#define TH_WALL_SIDE_L		(70.f)		// 横左センサの壁閾値
#define TH_WALL_SIDE_R		(70.f)		// 横右センサの壁閾値
#define TH_WALL_FRONT_L		(130.f)		// 前左センサの壁閾値
#define TH_WALL_FRONT_R		(130.f)		// 前右センサの壁閾値

// 壁切れ関連定義
#define TH_EDGE_RATE		(0.6f)		// 横センサの壁切れ閾値倍率
#define ZONE_HYSTERESIS		(5)			// 壁切れのヒステリシス区間

// ローカル関数群
void Wall_EstimateDistance( void );
void Wall_UpdateEdge( void );
void Wall_UpdateDeviation( void );

// センサ情報用構造体
typedef struct {
	volatile int16_t	now;			//
	volatile int16_t	old;			//
	volatile float		distance;		//
	volatile float		distance_old;	//
	volatile uint8_t	is_edge;		// 壁切れか否か
} t_control_wall;

volatile static float			correct_A[4];
volatile static float			correct_B[4];

volatile static t_control_wall	sen_fr;
volatile static t_control_wall 	sen_sr;
volatile static t_control_wall	sen_sl;
volatile static t_control_wall 	sen_fl;

volatile static float 			distance_min_l = CORRECT_DISTANCE_MAX;
volatile static float 			distance_min_r = CORRECT_DISTANCE_MAX;
volatile static float 			distance_edge_sl 	 = 0.f;
volatile static float 			distance_edge_sr 	 = 0.f;

/* ---------------------------------------------------------------
	壁センサ値を距離に変換する補正係数の計算関数
--------------------------------------------------------------- */
void Wall_Initialize( void )
{
	const float correct_distance_max[] =
			{ CORRECT_DISTANCE_FRONT[0], CORRECT_DISTANCE_SIDE[0], CORRECT_DISTANCE_SIDE[0], CORRECT_DISTANCE_FRONT[0] };
	const float correct_distance_min[] =
			{ CORRECT_DISTANCE_FRONT[1], CORRECT_DISTANCE_SIDE[1], CORRECT_DISTANCE_SIDE[1], CORRECT_DISTANCE_FRONT[1] };

	float correct_log_max[4];
	float correct_log_min[4];

	arm_vlog_f32(CORRECT_VALUE_MAX, correct_log_max, 4);
	arm_vlog_f32(CORRECT_VALUE_MIN, correct_log_min, 4);

	for(int8_t i = 0; i < 4; i++) {
		correct_B[i] = (correct_distance_max[i] * correct_log_min[i]
				- correct_distance_min[i] * correct_log_max[i]) / (correct_log_max[i] - correct_log_min[i]);
		correct_A[i] = (correct_distance_max[i] + correct_B[i]) * correct_log_min[i];
	}
}

/* ---------------------------------------------------------------
	壁センサ値を距離データに変換
--------------------------------------------------------------- */
void Wall_EstimateDistance( void )
{
	const int16_t	MIN_ADVALUE		= 20;
	const float		MAX_DISTANCE 	= CORRECT_DISTANCE_MAX;
	float			log_value;
	float			cast_data;

	if( sen_fl.now > MIN_ADVALUE ) {
		sen_fl.distance_old = sen_fl.distance;
		cast_data = (float)sen_fl.now;
		arm_vlog_f32(&cast_data, &log_value, 1);
		sen_fl.distance = MIN(CORRECT_DISTANCE_MAX, correct_A[0] / log_value - correct_B[0]);
	} else {
		sen_fl.distance = sen_fl.distance_old = MAX_DISTANCE;
	}

	if( sen_sl.now > MIN_ADVALUE ) {
		sen_sl.distance_old = sen_sl.distance;
		cast_data = (float)sen_sl.now;
		arm_vlog_f32(&cast_data, &log_value, 1);
		if( correct_A[1] / log_value - correct_B[1] > CORRECT_DISTANCE_MAX ) {
			sen_sl.distance = CORRECT_DISTANCE_MAX;
			distance_min_l = CORRECT_DISTANCE_MAX;
		} else {
			sen_sl.distance = correct_A[1] / log_value - correct_B[1];
		}
	} else {
		sen_sl.distance = sen_sl.distance_old = MAX_DISTANCE;
	}

	if( sen_sr.now > MIN_ADVALUE ) {
		sen_sr.distance_old = sen_sr.distance;
		cast_data = (float)sen_sr.now;
		arm_vlog_f32(&cast_data, &log_value, 1);
		if( correct_A[2] / log_value - correct_B[2] > CORRECT_DISTANCE_MAX ) {
			sen_sr.distance = CORRECT_DISTANCE_MAX;
			distance_min_r = CORRECT_DISTANCE_MAX;
		} else {
			sen_sr.distance = correct_A[2] / log_value - correct_B[2];
		}
	} else {
		sen_sr.distance = sen_sr.distance_old = MAX_DISTANCE;
	}

	if( sen_fr.now > MIN_ADVALUE ) {
		sen_fr.distance_old = sen_fr.distance;
		cast_data = (float)sen_fr.now;
		arm_vlog_f32(&cast_data, &log_value, 1);
		sen_fr.distance = MIN(CORRECT_DISTANCE_MAX, correct_A[3] / log_value - correct_B[3]);
	} else {
		sen_fr.distance = sen_fr.distance_old = MAX_DISTANCE;
	}
}

/* ---------------------------------------------------------------
	壁センサの更新関数
--------------------------------------------------------------- */
void Wall_Update( void )
{
	// 一個前のデータを更新
	sen_fr.old = sen_fr.now;
	sen_sr.old = sen_sr.now;
	sen_sl.old = sen_sl.now;
	sen_fl.old = sen_fl.now;

	// 最新のデータに更新
	sen_fr.now = Sensor_GetValue( FRONT + RIGHT );
	sen_sr.now = Sensor_GetValue( RIGHT );
	sen_sl.now = Sensor_GetValue( LEFT );
	sen_fl.now = Sensor_GetValue( FRONT + LEFT );
	Wall_EstimateDistance();

	// 壁切れ判定
	Wall_UpdateEdge();
}

/* ---------------------------------------------------------------
	横壁センサの壁切れ情報を更新
--------------------------------------------------------------- */
void Wall_UpdateEdge( void )
{
/*	if( Control_GetMode() == TURN || Control_GetMode() == ROTATE ) {
		distance_min_l = distance_min_r = CORRECT_DISTANCE_MAX;
		sen_sl.is_edge = sen_sr.is_edge = false;
	} else {
*/		// 右センサの壁切れ判定
		if( sen_sr.distance - distance_min_r / TH_EDGE_RATE > ZONE_HYSTERESIS && Vehicle_GetTotalDistance() - distance_edge_sr > 45.f ) {
			sen_sr.is_edge = true;
			distance_edge_sr = Vehicle_GetTotalDistance();
			distance_min_r = CORRECT_DISTANCE_MAX;
		} else {
			sen_sr.is_edge = false;
			distance_min_r = MIN(distance_min_r, sen_sr.distance);
		}

		// 左センサの壁切れ判定
		if( sen_sl.distance - distance_min_l / TH_EDGE_RATE > ZONE_HYSTERESIS && Vehicle_GetTotalDistance() - distance_edge_sl > 45.f ) {
			sen_sl.is_edge = true;
			distance_edge_sl = Vehicle_GetTotalDistance();
			distance_min_l = CORRECT_DISTANCE_MAX;
		} else {
			sen_sl.is_edge = false;
			distance_min_l = MIN(distance_min_l, sen_sl.distance);
		}
//	}
}

float Wall_GetEdgeMinDistance( int8_t dir )
{
	switch( dir ) {
		case RIGHT:			return( distance_min_r );	break;
		case LEFT:			return( distance_min_l );	break;
	}
	return 0;
}

void Wall_ResetEdgeMinDistance( void )
{
	distance_min_l = distance_min_r = CORRECT_DISTANCE_MAX;
}

/* ---------------------------------------------------------------
	壁センサ値を取得
--------------------------------------------------------------- */
float Wall_GetDistance( uint8_t dir )
{
	switch( dir ) {
		case FRONT + RIGHT:	return( sen_fr.distance );	break;
		case RIGHT:			return( sen_sr.distance );	break;
		case LEFT:			return( sen_sl.distance );	break;
		case FRONT + LEFT:	return( sen_fl.distance );	break;
	}
	return 0;
}

float Wall_GetDistanceDelta( uint8_t dir )
{
	switch( dir ) {
		case FRONT + RIGHT:	return( sen_fr.distance - sen_fr.distance_old );	break;
		case RIGHT:			return( sen_sr.distance - sen_sr.distance_old );	break;
		case LEFT:			return( sen_sl.distance - sen_sl.distance_old );	break;
		case FRONT + LEFT:	return( sen_fl.distance - sen_fl.distance_old );	break;
	}
	return 0;
}

/* ---------------------------------------------------------------
	壁との距離偏差取得
--------------------------------------------------------------- */
float Wall_GetDeviation( uint8_t dir )
{
	float deviation = 0.f;

	switch( dir ) {
		case FRONT + RIGHT:
			if( Control_GetMode() == FWALL ) {	// 前壁制御時
				if( sen_fr.distance < TH_CONTROL_FWALL_R ) {
					deviation = sen_fr.distance - REF_FWALL_R;
				} else {
					deviation = 0.f;
				}
			} else {							// 前壁制御以外
				if( sen_fr.distance < TH_CONTROL_FRONT_R ) {
					deviation = sen_fr.distance - REF_FRONT_R;
				} else {
					deviation = 0;
				}
			}
		break;

		case RIGHT:
			if( (sen_sr.distance < TH_CONTROL_SIDE_R) && (ABS(sen_sr.now - sen_sr.old) < TH_CONTROL_CUT)
					&& (sen_fr.distance > TH_CONTROL_CLOSE_FR) && (sen_fl.distance > TH_CONTROL_CLOSE_FL) ) {
				deviation = sen_sr.distance - REF_SIDE_R;
			} else {
				deviation = 0.f;
			}
		break;

		case LEFT:
			if( (sen_sl.distance < TH_CONTROL_SIDE_L) && (ABS(sen_sl.now - sen_sl.old) < TH_CONTROL_CUT)
					&& (sen_fr.distance > TH_CONTROL_CLOSE_FR) && (sen_fl.distance > TH_CONTROL_CLOSE_FL) ) {
				deviation = sen_sl.distance - REF_SIDE_L;
			} else {
				deviation = 0.f;
			}
		break;

		case FRONT + LEFT:
			if( Control_GetMode() == FWALL ) {	// 前壁制御時
				if( sen_fl.distance < TH_CONTROL_FWALL_L ) {
					deviation = sen_fl.distance - REF_FWALL_L;
				} else {
					deviation = 0.f;
				}
			} else {							// 前壁制御以外
				if( sen_fl.distance < TH_CONTROL_FRONT_L ) {
					deviation = sen_fl.distance - REF_FRONT_L;
				} else {
					deviation = 0.f;
				}
			}
		break;
	}
	return deviation;
}

/* ---------------------------------------------------------------
	壁の有無判定取得
--------------------------------------------------------------- */
uint8_t Wall_GetIsMaze( uint8_t dir )
{
	t_maze is_maze;

	switch( dir ) {
		case FRONT + RIGHT:	return( sen_fr.distance < TH_WALL_FRONT_R );	break;
		case RIGHT:			return( sen_sr.distance < TH_WALL_SIDE_R  );	break;
		case LEFT:			return( sen_sl.distance < TH_WALL_SIDE_L );		break;
		case FRONT + LEFT:	return( sen_fl.distance < TH_WALL_FRONT_L );	break;
		default:
			is_maze.byte = 0x00;

			if( (sen_fr.distance < TH_WALL_FRONT_R) && (sen_fl.distance < TH_WALL_FRONT_L) ) {
				is_maze.bit.north = true;
			} else {
				is_maze.bit.north = false;
			}

			if( sen_sr.distance < TH_WALL_SIDE_R ) {
				is_maze.bit.east = true;
			} else {
				is_maze.bit.east = false;
			}

			if( sen_sl.distance < TH_WALL_SIDE_L ) {
				is_maze.bit.west = true;
			} else {
				is_maze.bit.west = false;
			}

			return is_maze.byte;
		break;
	}
}

/* ---------------------------------------------------------------
	壁切れ判定取得
--------------------------------------------------------------- */
uint8_t Wall_GetEdge( uint8_t dir )
{
	switch( dir ) {
	//	case FRONT + RIGHT:	return( sen_fr.is_edge );	break;
		case RIGHT:			return( sen_sr.is_edge );	break;
		case LEFT:			return( sen_sl.is_edge );	break;
	//	case FRONT + LEFT:	return( sen_fl.is_edge );	break;
	}
	return false;
}

float Wall_GetEdgeDistance( int8_t dir )
{
	if( dir == LEFT ) {
		return distance_edge_sl;
	} else if( dir == RIGHT ) {
		return distance_edge_sr;
	} else;
	return (distance_edge_sr - distance_edge_sl);
}

void Wall_ResetEdgeDistance( void )
{
	distance_edge_sl = distance_edge_sr = 0.f;
}

/* ---------------------------------------------------------------
	デバッグ用
--------------------------------------------------------------- */
void Wall_DebugPrintf( void )
{
	Sensor_StartLED();
	while( Communicate_ReceiceDMA() != 0x1b ) {
		printf("%5.1f, %5.1f, %5.1f, %5.1f\r\n",
				sen_fl.distance, sen_sl.distance, sen_sr.distance, sen_fr.distance);
	}
	Sensor_StopLED();
}
