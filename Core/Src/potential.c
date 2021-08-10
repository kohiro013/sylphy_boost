
#include "defines.h"
#include "global.h"

volatile static uint16_t potential[MAZE_X][MAZE_Y];

/* ----------------------------------------------------------------------------------
	周囲のポテンシャルを取得（向きに-1を入れると自己位置のポテンシャルを取得）
-----------------------------------------------------------------------------------*/
uint16_t Potential_GetAroundSection( t_position *my, int8_t dir )
{
	int8_t	x = ( my -> x );
	int8_t	y = ( my -> y );
	int8_t	gdir = ( my -> dir );

	if ( dir == -1 ) {
		return potential[x][y];
	} else;

	gdir += (dir - 1);
	if ( gdir < EAST ) {
		gdir = SOUTH;
	} else if( gdir > SOUTH ) {
		gdir -= ( SOUTH + 1 );
	} else;

	x += (int)( arm_cos_f32(DEG2RAD(90.f * gdir)) + SIGN(arm_cos_f32(DEG2RAD(90.f * gdir))) * 0.5f );
	y += (int)( arm_sin_f32(DEG2RAD(90.f * gdir)) + SIGN(arm_sin_f32(DEG2RAD(90.f * gdir))) * 0.5f );

	if( (x < 0) || (y < 0) ) {
		return 0xffff;
	} else if( (x > MAZE_X-1) || (y > MAZE_Y-1) ) {
		return 0xffff;
	} else {
		return potential[x][y];
	}
}

/* ----------------------------------------------------------------------------------
	高速ポテンシャルマップ生成
-----------------------------------------------------------------------------------*/
void Potential_MakeMap( int8_t gx, int8_t gy )
{
	int8_t			x, y;
	t_maze			maze;
	static uint16_t queue[1025];	// 区画の座標(0～255)を入れる配列
									// 左下0、右下15、左上240、右上255
	static uint16_t step;
	static int16_t 	head, tail;		// 先頭位置, 末尾位置

	for( y = 0; y < MAZE_Y; y++ ) {
		for( x = 0; x < MAZE_X; x++ ) {
			potential[x][y] = 0xffff;
		}
	}

	potential[gx][gy] = 0;				// 目標地点に距離０を書き込む
	queue[0] = (gy << 8) + gx;			// 目標地点の座標を記憶
	head = 0;							// 先頭位置を初期化
	tail = 1;							// 末尾位置は、最後の情報位置＋１

	while( head != tail ) {				// 配列の中身が空ならループを抜ける
		x = queue[head] & 255;			// 配列から区画の座標を取り出す
		y = queue[head] >> 8;
		head++;							// 情報を取り出したので先頭位置をずらす
		step = potential[x][y] + 1;		// 新しいステップ数
		maze = Maze_GetGlobal(x, y);

		if( (maze.bit.east == false) && (x != MAZE_X-1) ) {
			if( potential[x+1][y] == 0xffff ) {
				potential[x+1][y] = step;
				queue[tail] = (y << 8) + x + 1;	// 次の区間の座標を記憶
				tail++;							// 情報を入れたので末尾位置をすらす
			} else;
		} else;

		if( (maze.bit.north == false) && (y != MAZE_Y-1) ) {
			if( potential[x][y+1] == 0xffff ) {
				potential[x][y+1] = step;
				queue[tail] = ((y + 1) << 8) + x;
				tail++;
			} else;
		} else;

		if( (maze.bit.west == false) && (x != 0) ) {
			if( potential[x-1][y] == 0xffff ) {
				potential[x-1][y] = step;
				queue[tail] = (y << 8) + x - 1;
				tail++;
			} else;
		} else;

		if( (maze.bit.south == false) && (y != 0) ) {
			if( potential[x][y-1] == 0xffff ) {
				potential[x][y-1] = step;
				queue[tail] = ((y - 1) << 8) + x;
				tail++;
			} else;
		} else;
	}
}

/* ----------------------------------------------------------------------------------
	高速ポテンシャルマップ生成
-----------------------------------------------------------------------------------*/
void Potential_MakeUnknownMap( int8_t gx, int8_t gy )
{
	int8_t				x, y;
	t_maze				maze;
	static uint16_t 	queue[1025];	// 区画の座標(0～255)を入れる配列
										// 左下0、右下15、左上240、右上255
	static uint16_t 	step;
	static int16_t 		head, tail;		// 先頭位置, 末尾位置
	int8_t				flag = false;

	head = 0;						// 先頭位置を初期化
	tail = 1;						// 末尾位置は、最後の情報位置＋１

	Potential_MakeMap( gx, gy );
	for( y = 0; y < MAZE_Y; y++ ) {
		for( x = 0; x < MAZE_X; x++ ) {
			maze = Maze_GetGlobal(x, y);
			if( potential[x][y] != 0xffff && ((maze.byte &0xf0) != 0xf0) ) {
				potential[x][y] = 0x0000;		// 目標地点に距離０を書き込む
				queue[head] = (y << 8) + x;		// 目標地点の座標を記憶
				head++;
				tail++;
				flag = true;
			} else {
				potential[x][y] = 0xffff;
			}
		}
	}

	if( flag == false ) {
		potential[gx][gy] = 0;
		queue[0] = (gy << 8) + gx;
	} else;

	head = 0;
	while( head != tail ) {				// 配列の中身が空ならループを抜ける
		x = queue[head] & 255;			// 配列から区画の座標を取り出す
		y = queue[head] >> 8;
		head++;							// 情報を取り出したので先頭位置をずらす
		step = potential[x][y] + 1;		// 新しいステップ数
		maze = Maze_GetGlobal(x, y);

		// 何故かないと動かない
		if( step == 0 ) {
			continue;
		} else;

		if( (maze.bit.east == false) && (x != MAZE_X-1) ) {
			if( potential[x+1][y] == 0xffff ) {
				potential[x+1][y] = step;
				queue[tail] = (y << 8) + x + 1;	// 次の区間の座標を記憶
				tail++;							// 情報を入れたので末尾位置をすらす
			} else;
		} else;

		if( (maze.bit.north == false) && (y != MAZE_Y-1) ) {
			if( potential[x][y+1] == 0xffff ) {
				potential[x][y+1] = step;
				queue[tail] = ((y + 1) << 8) + x;
				tail++;
			} else;
		} else;

		if( (maze.bit.west == false) && (x != 0) ) {
			if( potential[x-1][y] == 0xffff ) {
				potential[x-1][y] = step;
				queue[tail] = (y << 8) + x - 1;
				tail++;
			} else;
		} else;

		if( (maze.bit.south == false) && (y != 0) ) {
			if( potential[x][y-1] == 0xffff ) {
				potential[x][y-1] = step;
				queue[tail] = ((y - 1) << 8) + x;
				tail++;
			} else;
		} else;
	}
}

