
#include "defines.h"
#include "global.h"

#define FLASH_SECTOR_1		(1U)
#define FLASH_START_ADRESS	(0x08004000)

volatile static t_maze 		maze[MAZE_X][MAZE_Y] = {{{0}}};
volatile static t_position	maze_buffer[5];
volatile static uint8_t		maze_buffer_num = 0;

/* ----------------------------------------------------------------------------------
	デバッグ用壁情報入力
-----------------------------------------------------------------------------------*/
#if (MAZE_X >= 16 && MAZE_Y >= 16)
void Maze_SetDebugData( void )	// 16x16デバッグ迷路のゴールは(7, 7)
{
	int		i, j;
	uint8_t test[MAZE_X][MAZE_Y] = {
		{13, 12, 10, 10,  8,  8,  8,  8, 10,  8, 10,  8,  8, 10,  8, 11},
		{ 4,  2,  8, 11,  5,  5,  5,  6,  9,  5, 12,  3,  6,  8,  2,  9},
		{ 4,  9,  6,  9,  5,  7,  4,  9,  5,  7,  6,  9, 12,  3, 14,  1},
		{ 5,  4, 11,  6,  1, 12,  1,  5,  5, 12,  9,  4,  1, 13, 14,  1},
		{ 5,  6, 11, 12,  1,  5,  7,  7,  6,  3,  5,  5,  6,  2, 10,  1},
		{ 5, 14,  8,  1,  4,  3, 12, 10,  9, 13,  6,  1, 13, 14, 10,  1},
		{ 7, 12,  3,  5,  5, 12,  0,  9,  6,  1, 13,  6,  1, 12,  9,  5},
		{14,  0, 11,  5,  5,  5,  5,  4,  9,  6,  1, 13,  6,  3,  6,  1},
		{14,  0, 11,  6,  3,  4,  3,  6,  3, 13,  6,  1, 13, 14,  8,  3},
		{14,  0, 10, 10,  9,  6,  8, 10, 10,  1, 12,  0,  1, 14,  1, 15},
		{12,  0, 11, 12,  3, 13,  6, 10, 10,  2,  1,  5,  4,  9,  6,  9},
		{ 5,  4, 11,  5, 14,  1, 12,  8, 10,  9,  6,  1,  5,  6,  9,  5},
		{ 5,  5, 12,  2, 11,  5,  5,  4,  9,  6,  9,  4,  2,  9,  5,  5},
		{ 5,  5,  5, 12, 10,  3,  4,  1,  4,  9,  4,  2, 10,  0,  1,  5},
		{ 4,  3,  6,  2, 10, 11,  5,  4,  3,  6,  3, 13, 13,  5,  5,  5},
		{ 6, 10, 10, 10, 10, 10,  2,  2, 10, 10, 10,  2,  2,  2,  2,  3},
	}; // GOAL_X = 7, GOAL_Y = 7

	for( j = 0; j < MAZE_Y; j++ ) {
		for( i = 0; i < MAZE_X; i++ ) {
			if( i > 15 || j > 15 ) {
				maze[i][j].byte = 0xff;
			} else {
				maze[i][j].byte = test[j][i];
				maze[i][j].byte |= 0xf0;
			}
		}
	}
}
#endif

#if (MAZE_X >= 32 && MAZE_Y >= 32)
void Maze_SetDebugData_32x32( void )	// 32x32デバッグ迷路のゴールは(1, 2)
{
	int		i, j;
	uint8_t test[MAZE_X][MAZE_Y] = {
	//	  0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23  24  25  26  27  28  29  30  31
		{13, 12, 10, 10,  9, 12, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10,  8,  8,  8,  8, 11},	// 0
		{ 4,  2, 10, 11,  4,  2, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10,  8, 11,  5,  7,  7,  4,  9},	// 1
		{ 5, 12,  8,  9,  6,  9, 12,  9, 12, 10,  9, 14,  8, 10,  9, 13, 12,  9, 12, 10, 10, 10, 10, 10,  9,  6,  9,  6, 10, 10,  3,  5},	// 2
		{ 5,  4,  0,  0,  9,  6,  3,  5,  6,  9,  6,  9,  6,  9,  5,  4,  3,  5,  6, 10, 10, 10, 10,  9,  6,  9,  6, 10, 10, 10,  9,  5},	// 3
		{ 5,  6,  2,  3,  6,  9, 14,  2,  9,  4,  9,  6,  8,  3,  6,  3, 13,  6,  9, 12, 10,  9, 12,  2, 11,  5, 12, 10,  9, 12,  3,  5},	// 4
		{ 6,  8,  9, 12,  9,  6,  9, 14,  1,  7,  4, 11,  7, 12,  9, 12,  2,  9,  6,  3, 13,  5,  6,  8,  9,  5,  4, 11,  5,  6,  9,  5},	// 5
		{12,  1,  5,  5,  6,  9,  6,  9,  6, 10,  3, 12, 10,  3,  6,  3, 13,  6,  9, 12,  1,  5, 12,  1,  5,  5,  5, 12,  1, 12,  3,  5},	// 6
		{ 5,  5,  6,  3, 14,  0, 11,  6,  9, 14,  8,  2, 10, 10, 10, 10,  2,  9,  6,  3,  6,  3,  5,  5,  5,  6,  3,  5,  5,  6,  9,  5},	// 7
		{ 5,  5, 14,  8, 10,  3, 12,  9,  6,  9,  7, 12,  9, 12,  8,  9, 13,  6,  9, 12,  8,  9,  5,  5,  6,  9, 12,  1,  5, 12,  3,  5},	// 8
		{ 5,  5, 14,  1, 12, 10,  3,  6,  9,  6,  8,  3,  5,  6,  0,  1,  4,  9,  5,  4,  0,  1,  5,  5, 12,  1,  5,  4,  3,  6,  9,  5},	// 9
		{ 5,  5, 13,  6,  3, 12, 11, 12,  3, 12,  2,  9,  6,  9,  6,  3,  5,  6,  3,  6,  2,  3,  5,  5,  5,  5,  5,  5, 12,  9,  5,  5},	// 10
		{ 5,  5,  6,  8,  8,  3, 12,  3, 14,  2,  9,  6,  9,  6,  9, 14,  1, 14,  8, 10,  8,  9,  5,  5,  4,  3,  4,  3,  5,  5,  5,  5},	// 11
		{ 5,  5, 12,  3,  6,  8,  3, 12,  9, 13,  6,  9,  6,  9,  6,  9,  6,  9,  5, 12,  3,  5,  5,  5,  6,  9,  5, 12,  1,  5,  5,  5},	// 12
		{ 5,  5,  6,  8, 11,  7, 12,  0,  1,  4,  9,  4, 11,  6,  9,  6,  9,  6,  3,  4,  9,  5,  5,  6,  9,  5,  5,  5,  5,  5,  5,  5},	// 13
		{ 5,  5, 13,  6,  9, 13,  6,  2,  3,  5,  6,  2, 11, 13,  6,  9,  6,  8,  9,  5,  4,  3,  6, 10,  2,  1,  5,  5,  5,  4,  3,  5},	// 14
		{ 5,  5,  4, 10,  3,  4, 11, 12,  9,  6, 10, 10,  9,  6,  9,  6,  9,  7,  5,  5,  5, 12, 10,  8, 10,  3,  6,  3,  6,  3, 12,  1},	// 15
		{ 5,  5,  5, 12,  9,  4,  9,  5,  5, 12,  9, 12,  2, 10,  2,  9,  6,  9,  6,  1,  5,  5, 12,  2,  9, 12, 10, 10, 10, 10,  3,  7},	// 16
		{ 5,  5,  6,  3,  5,  5,  5,  5,  5,  5,  5,  5, 12, 10,  9,  4, 11,  6,  9,  7,  5,  5,  4, 10,  1,  6, 10, 10, 10, 10, 10,  9},	// 17
		{ 5,  5, 12,  9,  5,  5,  6,  3,  6,  3,  5,  6,  3, 14,  1,  6, 10,  9,  6,  9,  6,  1,  6,  8,  3, 14, 10,  8, 10, 10, 10,  3},	// 18
		{ 5,  5,  5,  5,  6,  3, 12,  8,  9, 12,  1, 12, 10, 10,  2, 10, 11,  6,  9,  6,  9,  7, 12,  3, 12,  9, 12,  2,  8, 10,  8, 11},	// 19
		{ 5,  5,  5,  5, 12,  9,  4,  0,  1,  5,  7,  6, 10, 10,  9, 12, 10, 10,  2,  9,  6,  8,  3, 12,  0,  1,  4, 10,  0, 10,  0, 11},	// 20
		{ 5,  5,  5,  5,  5,  5,  6,  2,  3,  6, 10, 10, 10, 10,  3,  5, 12, 10, 10,  2, 11,  4,  9,  6,  0,  1,  6,  8,  2,  8,  2,  9},	// 21
		{ 5,  5,  5,  5,  5,  5, 12, 10, 10, 10,  8, 10,  8,  8, 10,  3,  5, 12,  8,  9, 12,  3,  6,  9,  6,  1, 14,  0, 10,  0, 10,  1},	// 22
		{ 5,  5,  5,  5,  5,  5,  6, 10,  9, 12,  2, 10,  3,  6, 10,  9,  4,  1,  5,  4,  3, 12,  9,  6,  9,  7, 12,  2,  8,  2,  8,  3},	// 23
		{ 5,  5,  5,  6,  1,  6, 10,  9,  5,  6, 10, 10, 10, 10,  8,  3,  5,  6,  2,  3, 12,  0,  0,  9,  6,  9,  6, 10,  0, 10,  0, 11},	// 24
		{ 5,  5,  6,  9,  5, 12, 10,  3,  6, 10, 10, 10, 10,  9,  6, 10,  3, 12,  9, 13,  6,  2,  2,  2, 11,  6,  9, 14,  2,  8,  2,  9},	// 25
		{ 5,  6, 10,  1,  5,  6,  8, 11, 12, 10, 10, 10, 10,  2, 10, 10,  9,  5,  5,  5, 12,  8,  9, 13, 12,  9,  6,  9, 14,  0, 10,  1},	// 26
		{ 4, 10,  9,  5,  5, 12,  0, 11,  6, 10,  9, 12, 10, 10, 10, 10,  3,  5,  5,  4,  1,  5,  4,  0,  1,  5, 13,  6,  9,  6,  8,  3},	// 27
		{ 4, 11,  5,  5,  5,  7,  4, 11, 12, 10,  3,  6, 10, 10, 10, 10,  9,  5,  5,  5,  4,  0,  1,  5,  4,  0,  1, 13,  6,  9,  6,  9},	// 28
		{ 4, 11,  5,  5,  6,  9,  4, 11,  6, 10, 10, 10, 10, 10, 10,  8,  3,  5,  5,  5,  5,  5,  4,  0,  1,  5,  4,  0,  9,  6,  9,  5},	// 29
		{ 4,  8,  3,  6, 10,  3,  6, 10, 10, 10, 10, 10, 10, 10, 10,  3, 12,  3,  5,  5,  4,  0,  1,  5,  4,  0,  1,  5,  4,  9,  6,  1},	// 30
		{ 7,  6, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10,  2, 11,  6,  3,  7,  7,  6,  2,  3,  7,  6,  2,  3,  6, 10,  3},	// 31
	};	// GOAL_X = 1, GOAL_Y = 2

	for( j = 0; j < MAZE_Y; j++ ) {
		for( i = 0; i < MAZE_X; i++ ) {
			maze[i][j].byte = test[j][i];
			maze[i][j].byte |= 0xf0;
		}
	}
}
#endif

/* ----------------------------------------------------------------------------------
	壁情報をFlash領域に保存
-----------------------------------------------------------------------------------*/
void Maze_StoreFlash( void )
{
	Flash_Unlock();
	Flash_EraseSector(FLASH_SECTOR_1);
	Flash_WriteData(FLASH_START_ADRESS, (uint8_t*)maze, sizeof(maze)/sizeof(uint8_t));
	Flash_Lock();
}

/* ----------------------------------------------------------------------------------
	壁情報をFlash領域から読み込む
-----------------------------------------------------------------------------------*/
void Maze_LoadFlash( void )
{
	Flash_ReadData(FLASH_START_ADRESS, (uint8_t*)maze, sizeof(maze)/sizeof(uint8_t));
}

/* ----------------------------------------------------------------------------------
	ビットローテーション
-----------------------------------------------------------------------------------*/
uint8_t Maze_RotateBit( uint8_t byte, int8_t n, int8_t nbit )
{
	if ( n == 0 ) {
		return(byte);
	} else if ( n > 0 ) {
		byte = ((byte << n) | (byte >> (nbit - n)));
	} else {
		byte = ((byte >> -n) | (byte << (nbit + n)));
	}
	return byte;
}

/* ----------------------------------------------------------------------------------
	相対座標系と絶対座標系の相互変換
-----------------------------------------------------------------------------------*/
t_maze Maze_ConvertGlobalAndLocal( t_maze tmaze, int8_t dir )
{
	t_maze temp;

	temp.byte = ((tmaze.byte & 0xf0) >> 4);
	temp.byte = Maze_RotateBit( temp.byte, dir, 4 );

	tmaze.byte = ( tmaze.byte & 0x0f );
	tmaze.byte = Maze_RotateBit( tmaze.byte, dir, 4 );

	tmaze.byte = ( tmaze.byte & 0x0f );
	tmaze.byte |= ( ( temp.byte << 4 ) & 0xf0 );

	return tmaze;
}

/* ----------------------------------------------------------------------------------
	絶対座標系の壁情報
-----------------------------------------------------------------------------------*/
t_maze Maze_GetGlobal( int8_t x, int8_t y )
{
	return maze[x][y];
}

/* ----------------------------------------------------------------------------------
	相対座標系の壁情報
-----------------------------------------------------------------------------------*/
t_maze Maze_GetLocal( t_position *my )
{
	int8_t x 	= ( my -> x );
	int8_t y 	= ( my -> y );
	int8_t dir 	= ( my -> dir );

	return Maze_ConvertGlobalAndLocal( maze[x][y], 1 - dir );
}

int8_t Maze_IsLocal( int8_t x, int8_t y, int8_t gdir, int8_t dir )
{
	t_maze local = Maze_ConvertGlobalAndLocal( maze[x][y], 1 - gdir );
	switch( dir ) {
		case RIGHT:	return local.bit.east;	break;
		case FRONT:	return local.bit.north;	break;
		case LEFT:	return local.bit.west;	break;
		case REAR:	return local.bit.south;	break;
	}
	return -1;
}

/* ----------------------------------------------------------------------------------
	探索済みの壁かどうかの判定
-----------------------------------------------------------------------------------*/
int8_t Maze_GetIsUnknown( t_position *my, int8_t dir )
{
	int8_t x 	= (my -> x);
	int8_t y 	= (my -> y);
	int8_t gdir = (my -> dir);

	if( dir == -1 ) {
		return ((maze[x][y].byte &0xf0) == 0xf0);
	} else {
		gdir += ( dir - 1 );
		if( gdir < EAST ) {
			gdir = SOUTH;
		} else if( gdir > SOUTH ) {
			gdir -= (SOUTH + 1);
		} else;

		x += (int)(arm_cos_f32(DEG2RAD(90.f * gdir)) + SIGN(arm_cos_f32(DEG2RAD(90.f * gdir))) * 0.5f);
		y += (int)(arm_sin_f32(DEG2RAD(90.f * gdir)) + SIGN(arm_sin_f32(DEG2RAD(90.f * gdir))) * 0.5f);

		if( (x < 0) || (y < 0) ) {
			return false;
		} else if( (x > MAZE_X-1) || (y > MAZE_Y-1) ) {
			return false;
		} else;
	}
	return ( (maze[x][y].byte &0xf0) == 0xf0 );
}

/* ----------------------------------------------------------------------------------
	更新する壁情報を一時記録する
-----------------------------------------------------------------------------------*/
void Maze_ResetBuffer( void )
{
	for( uint8_t i = 0; i < sizeof(maze_buffer) / sizeof(maze_buffer[0]); i++ ) {
		maze_buffer[i].x 	= 0;
		maze_buffer[i].y 	= 0;
		maze_buffer[i].dir	= -1;
	}
	maze_buffer_num = 0;
}

/* ----------------------------------------------------------------------------------
	更新する壁情報を一時記録する
-----------------------------------------------------------------------------------*/
void Maze_StoreBuffer( int8_t x, int8_t y, int8_t dir )
{
	maze_buffer[maze_buffer_num].x 	 = x;
	maze_buffer[maze_buffer_num].y 	 = y;
	maze_buffer[maze_buffer_num].dir = dir;
	maze_buffer_num = (maze_buffer_num + 1) % (sizeof(maze_buffer) / sizeof(maze_buffer[0]));
}

/* ----------------------------------------------------------------------------------
	探索失敗時にバッファした壁情報を破棄する
-----------------------------------------------------------------------------------*/
void Maze_DeleteWhenClash( void )
{
	for( uint8_t i = 0; i < sizeof(maze_buffer) / sizeof(maze_buffer[0]); i++ ) {
		if( maze_buffer[i].dir == EAST ) {
			maze[maze_buffer[i].x][maze_buffer[i].y].bit.east = 0;
			maze[maze_buffer[i].x][maze_buffer[i].y].bit.unknown_east = 0;
			if( maze_buffer[i].x != MAZE_X-1 ) {
				maze[maze_buffer[i].x+1][maze_buffer[i].y].bit.west = 0;
				maze[maze_buffer[i].x+1][maze_buffer[i].y].bit.unknown_west = 0;
			} else;

		} else if( maze_buffer[i].dir == NORTH ) {
			maze[maze_buffer[i].x][maze_buffer[i].y].bit.north = 0;
			maze[maze_buffer[i].x][maze_buffer[i].y].bit.unknown_north = 0;
			if( maze_buffer[i].y != MAZE_Y-1 ) {
				maze[maze_buffer[i].x][maze_buffer[i].y+1].bit.south = 0;
				maze[maze_buffer[i].x][maze_buffer[i].y+1].bit.unknown_south = 0;
			} else;

		} else if( maze_buffer[i].dir == WEST ) {
			maze[maze_buffer[i].x][maze_buffer[i].y].bit.west = 0;
			maze[maze_buffer[i].x][maze_buffer[i].y].bit.unknown_west = 0;
			if( maze_buffer[i].x != 0 ) {
				maze[maze_buffer[i].x-1][maze_buffer[i].y].bit.east = 0;
				maze[maze_buffer[i].x-1][maze_buffer[i].y].bit.unknown_east = 0;
			} else;

		} else if( maze_buffer[i].dir == SOUTH ) {
			maze[maze_buffer[i].x][maze_buffer[i].y].bit.south = 0;
			maze[maze_buffer[i].x][maze_buffer[i].y].bit.unknown_south = 0;
			if( maze_buffer[i].y != 0 ) {
				maze[maze_buffer[i].x][maze_buffer[i].y-1].bit.north = 0;
				maze[maze_buffer[i].x][maze_buffer[i].y-1].bit.unknown_north = 0;
			} else;

		} else;
	}
	Maze_ResetBuffer();
}

/* ----------------------------------------------------------------------------------
	壁情報更新関数
-----------------------------------------------------------------------------------*/
void Maze_Update( int8_t x, int8_t y, int8_t dir, int8_t with )
{
	switch( dir ) {
		case EAST:
			maze[x][y].bit.east = with;
			maze[x][y].bit.unknown_east = true;
			Maze_StoreBuffer( x, y, EAST );
			if( x != MAZE_X-1 ) {
				maze[x+1][y].bit.west = maze[x][y].bit.east;
				maze[x+1][y].bit.unknown_west = true;
			} else;
		break;

		case NORTH:
			maze[x][y].bit.north = with;
			maze[x][y].bit.unknown_north = true;
			Maze_StoreBuffer( x, y, NORTH );
			if( y != MAZE_Y-1 ) {
				maze[x][y+1].bit.south = maze[x][y].bit.north;
				maze[x][y+1].bit.unknown_south = true;
			} else;
		break;

		case WEST:
			maze[x][y].bit.west = with;
			maze[x][y].bit.unknown_west = true;
			Maze_StoreBuffer( x, y, WEST );
			if( x != 0 ) {
				maze[x-1][y].bit.east = maze[x][y].bit.west;
				maze[x-1][y].bit.unknown_east = true;
			} else;
		break;

		case SOUTH:
			maze[x][y].bit.south = with;
			maze[x][y].bit.unknown_south = true;
			Maze_StoreBuffer( x, y, SOUTH );
			if( y != 0 ) {
				maze[x][y-1].bit.north = maze[x][y].bit.south;
				maze[x][y-1].bit.unknown_north = true;
			} else;
		break;
	}
}

/* ----------------------------------------------------------------------------------
	探索中の壁情報取得
-----------------------------------------------------------------------------------*/
void Maze_SetFromSensor( t_position *my )
{
	t_maze temp;
	int8_t x 	= ( my -> x );
	int8_t y 	= ( my -> y );
	int8_t dir 	= ( my -> dir );

	temp.byte = Wall_GetIsMaze( -1 );
	temp = Maze_ConvertGlobalAndLocal( temp, dir - 1 );

	if( maze[x][y].bit.unknown_east == false ) {
		Maze_Update( x, y, EAST,  temp.bit.east );
	} else;
	if( maze[x][y].bit.unknown_north == false ) {
		Maze_Update( x, y, NORTH, temp.bit.north );
	} else;
	if( maze[x][y].bit.unknown_west == false ) {
		Maze_Update( x, y, WEST,  temp.bit.west );
	} else;
	if( maze[x][y].bit.unknown_south == false ) {
		Maze_Update( x, y, SOUTH, temp.bit.south );
	} else;
}

/* ----------------------------------------------------------------------------------
	壁情報の初期化
-----------------------------------------------------------------------------------*/
void Maze_Reset( int8_t mode )
{
	int8_t x, y;

	// 外周
	for( x = 0; x < MAZE_X; x++ ) {
		maze[x][0].bit.south = true;
		maze[x][0].bit.unknown_south = true;
		maze[x][MAZE_Y-1].bit.north = true;
		maze[x][MAZE_Y-1].bit.unknown_north = true;
	}
	for( y = 0; y < MAZE_Y; y++ ) {
		maze[0][y].bit.west = true;
		maze[0][y].bit.unknown_west = true;
		maze[MAZE_X-1][y].bit.east = true;
		maze[MAZE_X-1][y].bit.unknown_east = true;
	}

	// スタート区画
	maze[0][0].bit.east = maze[1][0].bit.west = true;
	maze[0][0].bit.north = maze[0][1].bit.south = false;
	maze[0][0].bit.unknown_east = maze[1][0].bit.unknown_west = true;
	maze[0][0].bit.unknown_north = maze[0][1].bit.unknown_south = true;

	// ゴール区画（中心）
	if( GOAL_SIZE > 1 ) {
		for( x = GOAL_X; x < GOAL_X + GOAL_SIZE; x++ ) {
			for( y = GOAL_Y; y < GOAL_Y + GOAL_SIZE - 1; y++ ) {
				maze[x][y].bit.north = false;
				maze[x][y].bit.unknown_north = true;
				maze[x][y+1].bit.south = false;
				maze[x][y+1].bit.unknown_south = true;
			}
		}
		for( x = GOAL_X; x < GOAL_X + GOAL_SIZE - 1; x++ ) {
			for( y = GOAL_Y; y < GOAL_Y + GOAL_SIZE; y++ ) {
				maze[x][y].bit.east = false;
				maze[x][y].bit.unknown_east = true;
				maze[x+1][y].bit.west = false;
				maze[x+1][y].bit.unknown_west = true;
			}
		}
	} else;

	// 未探索区画
	for( y = 0; y < MAZE_Y; y++ ) {
		for(x = 0; x < MAZE_X; x++ ) {
			if( maze[x][y].bit.unknown_north == false ) {
				if( mode == SEARCH )		{	maze[x][y].bit.north = false;	}
				else if( mode == FASTEST ) 	{	maze[x][y].bit.north = true;	}
				else;
			} else;
			if( maze[x][y].bit.unknown_east == false ) {
				if( mode == SEARCH )		{	maze[x][y].bit.east = false;	}
				else if( mode == FASTEST ) 	{	maze[x][y].bit.east = true;	}
				else;
			} else;
			if( maze[x][y].bit.unknown_west == false ) {
				if( mode == SEARCH )		{	maze[x][y].bit.west = false;	}
				else if( mode == FASTEST ) 	{	maze[x][y].bit.west = true;	}
				else;
			} else;
			if( maze[x][y].bit.unknown_south == false ) {
				if( mode == SEARCH )		{	maze[x][y].bit.south = false;	}
				else if( mode == FASTEST ) 	{	maze[x][y].bit.south = true;	}
				else;
			} else;
		}
	}
}

/* ----------------------------------------------------------------------------------
	袋小路の壁を閉じる
-----------------------------------------------------------------------------------*/
void Maze_InsertDeadEnd( void )
{
	int8_t	x, y;
	int8_t	is_deadend = false;
	t_position my = Position_GetMyPlace();

	do{
		is_deadend = false;

		for( y = 0; y < MAZE_Y; y++ ) {
			for( x = 0; x < MAZE_X; x++ ) {
				// スタート・ゴール座標もしくは自己位置をスキップ
				if( x == 0 && y == 0 ) {
					// 何もしない
					continue;
				} else if( GOAL_X <= x && x <= GOAL_X + GOAL_SIZE - 1 &&
						   GOAL_Y <= y && y <= GOAL_Y + GOAL_SIZE - 1) {
					continue;
				} else if( x == my.x && y == my.y ){
					continue;
				} else {
					// 袋小路の穴埋め
					if( (maze[x][y].byte & 0x7f) == 0x77 ) {
						maze[x][y].byte |= 0xff;
						if( y != 0 ) {
							maze[x][y-1].bit.north = true;
							maze[x][y-1].bit.unknown_north = true;
						} else;
						is_deadend = true;
					} else if( (maze[x][y].byte & 0xbf) == 0xbb ) {
						maze[x][y].byte |= 0xff;
						if( x != 0 ) {
							maze[x-1][y].bit.east = true;
							maze[x-1][y].bit.unknown_east = true;
						} else;
						is_deadend = true;
					} else if( (maze[x][y].byte & 0xdf) == 0xdd ) {
						maze[x][y].byte |= 0xff;
						if( y != MAZE_Y-1 ){
							maze[x][y+1].bit.south = true;
							maze[x][y+1].bit.unknown_south = true;
						} else;
						is_deadend = true;
					} else if( (maze[x][y].byte & 0xef) == 0xee ) {
						maze[x][y].byte |= 0xff;
						if( x != MAZE_X-1 ){
							maze[x+1][y].bit.west = true;
							maze[x+1][y].bit.unknown_west = true;
						} else;
						is_deadend = true;
					} else;
				}
			}
		}
	} while ( is_deadend == true );
}

/* ----------------------------------------------------------------------------------
	柱
-----------------------------------------------------------------------------------*/
void Maze_SetPillarAlone( t_position *my )
{
	int8_t	x = (my -> x);
	int8_t	y = (my -> y);
	int8_t	dir = (my -> dir);

	if( x == GOAL_X && y == GOAL_Y ) {
		return;
	} else if( x == GOAL_X + 1 && y == GOAL_Y ) {
		return;
	} else if( x == GOAL_X + 1 && y == GOAL_Y + 1 ) {
		return;
	} else if( x == GOAL_X && y == GOAL_Y + 1 ) {
		return;
	} else;

	switch( dir ) {
		case EAST:
			if( maze[x][y].bit.west == false || maze[x][y].bit.unknown_west == true ) {
				if( maze[x][y+1].bit.unknown_west == false && maze[x-1][y+1].bit.unknown_east == false ) {
					if( maze[x][y].bit.north == false && maze[x][y].bit.unknown_north == true ) {
						if( maze[x-1][y].bit.north == false && maze[x-1][y].bit.unknown_north == true ) {
							Maze_Update( x, y+1, WEST, true );
						} else;
					} else;
				} else;
				if( maze[x][y-1].bit.unknown_west == false || maze[x-1][y-1].bit.unknown_east == false ){
					if( maze[x][y].bit.south == false && maze[x][y].bit.unknown_south == true ) {
						if( maze[x-1][y].bit.south == false && maze[x-1][y].bit.unknown_south == true ) {
							Maze_Update( x, y-1, WEST, true );
						} else;
					} else;
				} else;
			} else;
			break;

		case NORTH:
			if( maze[x][y].bit.south == false && maze[x][y].bit.unknown_south == true ) {
				if( maze[x+1][y].bit.unknown_south == false || maze[x+1][y-1].bit.unknown_north == false ) {
					if( maze[x][y].bit.east == false && maze[x][y].bit.unknown_east == true ) {
						if( maze[x][y-1].bit.east == false && maze[x][y-1].bit.unknown_east == true ) {
							Maze_Update( x+1, y, SOUTH, true );
						} else;
					} else;
				} else;
				if( maze[x-1][y].bit.unknown_south == false || maze[x-1][y-1].bit.unknown_north == false ) {
					if( maze[x][y].bit.west == false && maze[x][y].bit.unknown_west == true ) {
						if( maze[x][y-1].bit.west == false && maze[x][y-1].bit.unknown_west == true ) {
							Maze_Update( x-1, y, SOUTH, true );
						} else;
					} else;
				} else;
			} else;
			break;

		case WEST:
			if( maze[x][y].bit.east == false && maze[x][y].bit.unknown_east == true ) {
				if( maze[x][y+1].bit.unknown_east == false || maze[x+1][y+1].bit.unknown_west == false ) {
					if( maze[x][y].bit.north == false && maze[x][y].bit.unknown_north == true ) {
						if( maze[x+1][y].bit.north == false && maze[x+1][y].bit.unknown_north == true ) {
							Maze_Update( x, y+1, EAST, true );
						} else;
					} else;
				} else;
				if( maze[x][y-1].bit.unknown_east == false || maze[x+1][y-1].bit.unknown_west == false ) {
					if( maze[x][y].bit.south == false && maze[x][y].bit.unknown_south == true ) {
						if( maze[x+1][y].bit.south == false && maze[x+1][y].bit.unknown_south == true){
							Maze_Update( x, y-1, EAST, true );
						} else;
					} else;
				} else;
			} else;
			break;

		case SOUTH:
			if( maze[x][y].bit.north == false && maze[x][y].bit.unknown_north == true ) {
				if( maze[x+1][y].bit.unknown_north == false || maze[x+1][y+1].bit.unknown_south == false ) {
					if( maze[x][y].bit.east == false && maze[x][y].bit.unknown_east == true ) {
						if( maze[x][y+1].bit.east == false && maze[x][y+1].bit.unknown_east == true ) {
							Maze_Update( x+1, y, NORTH, true );
						} else;
					} else;
				} else;
				if( maze[x-1][y].bit.unknown_north == false || maze[x-1][y+1].bit.unknown_south == false ) {
					if( maze[x][y].bit.west == false && maze[x][y].bit.unknown_west == true ) {
						if( maze[x][y+1].bit.west == false && maze[x][y+1].bit.unknown_west == true ) {
							Maze_Update( x-1, y, NORTH, true );
						} else;
					} else;
				} else;
			} else;
			break;
	}
}

/* ----------------------------------------------------------------------------------
	壁情報の表示
-----------------------------------------------------------------------------------*/
void Maze_Display( void )
{
	int8_t i, j;
	t_position mouse;

	printf("\n\r");
	for( j = MAZE_Y-1; j >= 0; j-- ) {
		printf("   ");
		for( i = 0; i < MAZE_X; i++ ) {
			printf("+");
			if( maze[i][j].bit.north == true ) {
				printf("----");
			} else {
				printf("    ");
			}
		}
		printf("+\n\r%02d ", j);
		for( i = 0; i < MAZE_X; i++ ) {
			if( maze[i][j].bit.west == true ) {
				printf("|");
			} else {
				printf(" ");
			}
			mouse.x = i, mouse.y = j;
			//printf("    " );
			printf(" %02x ", Potential_GetAroundSection(&mouse, -1)&0x00ff );
		}
		if( maze[MAZE_X-1][j].bit.east == true ) {
			printf("|\n\r");
		} else {
			printf(" \n\r");
		}
	}
	printf("   ");
	for( i = 0; i < MAZE_X; i++ ) {
		printf("+");
		if( maze[i][0].bit.south == true ) {
			printf("----");
		} else {
			printf("    ");
		}
	}
	printf("+\r\n   ");
	for( i = 0; i < MAZE_X; i++ ) {
		printf("  %02d ", i);
	}
	printf("\r\n");
}
