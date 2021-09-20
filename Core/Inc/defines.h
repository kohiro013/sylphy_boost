
#ifndef INC_DEFINES_H_
#define INC_DEFINES_H_

	#include <stdio.h>
	#include <string.h>
	#include "arm_math.h"

	#define GOAL_X				(9)
	#define GOAL_Y				(9)
	#define GOAL_SIZE			(2)
	#define MAZE_X				(32)
	#define MAZE_Y				(32)

	#define SYSTEM_PERIOD		(0.001f)  		// 積分周期[s]

	#define G					(9.80665f)		// 重量加速度[m/s^2]
	#define MASS				(13.0f)			// 質量[g]
	#define INERTIA				(2000.f)		// 慣性モーメント[g*mm^2]
	#define TIRE				(13.21f)		// タイヤの直径[mm] (小さくすると距離が延びる)
	#define TREAD				(17.0f)			// 重心からタイヤまでの距離
	#define CF					(1000000.0F)	// コーナリングフォース [g/rad]
	#define START_OFFSET		(42.f - 35.f)	// スタートラインまでの距離 [mm]

//	#define PI					(3.1415926f)		// 円周率
	#define SQRT2				(1.41421356237f)	// ルート2
	#define SQRT3				(1.73205080757f)	// ルート3
	#define SQRT5				(2.2360679775f)		// ルート5
	#define SQRT7				(2.64575131106f)	// ルート7
	#define DEG2RAD(x)			(((x)/180.0f)*PI)	// 度数法からラジアンに変換
	#define RAD2DEG(x)			(180.0f*((x)/PI))	// ラジアンから度数法に変換
	#define NAPEIR_INTGRAL		(0.76321461819897f)	// 単位ネイピア関数の積分値

	#define SWAP(a, b) 			((a != b) && (a += b, b = a - b, a -= b))
	#define ABS(x) 				((x) < 0 ? -(x) : (x))		// 絶対値
	#define SIGN(x)				((x) < 0 ? -1 : 1)			// 符号
	#define ROUND(x)			((int)(x + SIGN(x) * 0.5f))	// 四捨五入で整数を返す
	#define MAX(a, b) 			((a) > (b) ? (a) : (b))		// 2つのうち大きい方を返します
	#define MIN(a, b) 			((a) < (b) ? (a) : (b))		// 2つのうち小さい方を返します
	#define MAX3(a, b, c) 		((a) > (MAX(b, c)) ? (a) : (MAX(b, c)))
	#define MIN3(a, b, c) 		((a) < (MIN(b, c)) ? (a) : (MIN(b, c)))
	#define MAX4(a, b, c, d) 	((a) > (MAX3(b, c, d)) ? (a) : (MAX3(b, c, d)))
	#define MIN4(a, b, c, d) 	((a) < (MIN3(b, c, d)) ? (a) : (MIN3(b, c, d)))

	typedef enum {
		false	= 0,
		true	= 1,
	} t_bool;

	typedef struct {
		int8_t x;
		int8_t y;
		int8_t dir;
	} t_position;

	typedef enum {	// ローカル方向列挙
		RIGHT	= 0,
		FRONT	= 1,
		LEFT	= 2,
		REAR	= 3,
	} t_localdir;

	typedef enum{	// グローバル方向列挙
		EAST	= 0,
		NORTH	= 1,
		WEST	= 2,
		SOUTH	= 3,
	} t_globaldir;

	typedef enum {	// 制御モード列挙
		FAULT	 = -1,
		NONE	 = 0,
		SEARCH	 = 1,
		FASTEST	 = 2,
		TURN	 = 3,
		ROTATE	 = 4,
		DIAGONAL = 5,
		FWALL	 = 6,
		ADJUST	 = 7,
	} t_control_mode;

	typedef enum {	// 走行パス列挙
		turn_0		= 0,
		turn_90		= 1,
		turn_large	= 2,
		turn_180	= 3,
		turn_45in	= 4,
		turn_135in	= 5,
		turn_90v	= 6,
		turn_45out	= 7,
		turn_135out	= 8,
		turn_kojima	= 9,
		goal		= 10,
	} t_path_turn;

	typedef enum{	// 探索タイプ列挙
		ADACHI		= 0,
		UNKNOWN		= 1,
		ALL			= 2,
	} t_search;

	typedef struct {	// 制御パラメータ用構造体
		float kp;	// 比例制御量
		float ki;	// 積分制御量
		float kd;	// 微分制御量
	} t_control_gain;

	typedef union {		// 壁情報用構造体
		uint8_t byte;
		struct {
			uint8_t east			:1;
			uint8_t north			:1;
			uint8_t west			:1;
			uint8_t south			:1;
			uint8_t unknown_east	:1;
			uint8_t unknown_north	:1;
			uint8_t unknown_west	:1;
			uint8_t unknown_south	:1;
		} bit;
	} t_maze;

	typedef struct {	// 走行パス用構造体
		volatile int8_t straight;
		volatile int8_t direction;
		volatile int8_t type;
	} t_path;

	typedef struct {	// 直進用構造体
		volatile float acceleration;
		volatile float deceleration;
		volatile float max_velocity;
	} t_init_straight;

	typedef struct {	// スラローム用構造体
		volatile float velocity;
		volatile float degree;
		volatile float time;
		volatile float radius;
		volatile float before;
		volatile float after;
	} t_init_slalom;

#endif /* INC_DEFINES_H_ */
