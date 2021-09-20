
#include "defines.h"
#include "global.h"


/* ----------------------------------------------------------------------------------
	進む方向の決定（足立法）
-----------------------------------------------------------------------------------*/
int8_t Route_GetNextDirection_Adachi( t_position my )
{
	static int8_t 	old_direction 	= -1;
	int8_t 			next_direction	= REAR;
	uint16_t 		min_potential 	= Potential_GetAroundSection( &my, -1 );
	t_maze 			local_maze 		= Maze_GetLocal( &my );

	if( local_maze.bit.east == false ) {
		next_direction = RIGHT;
	} else;

	if( local_maze.bit.west == false ) {
		next_direction = LEFT;
	} else;

	if( local_maze.bit.north == false ) {
		next_direction = FRONT;
	} else;

	// 斜め優先（前回の方向が左だった場合右を優先）
	if( (old_direction == LEFT) && (local_maze.bit.east == false) ) {
		if( min_potential > Potential_GetAroundSection( &my, RIGHT ) ) {
			min_potential = Potential_GetAroundSection( &my, RIGHT );
			next_direction = RIGHT;
		} else;
	} else;

	// 斜め優先（前回の方向が右だった場合左を優先）
	if( (old_direction == RIGHT) && (local_maze.bit.west == false) ) {
		if( min_potential > Potential_GetAroundSection( &my, LEFT ) ) {
			min_potential = Potential_GetAroundSection( &my, LEFT );
			next_direction = LEFT;
		} else;
	} else;

	if( local_maze.bit.north == false ) {
		if( min_potential > Potential_GetAroundSection( &my, FRONT ) ) {
			min_potential = Potential_GetAroundSection( &my, FRONT );
			next_direction = FRONT;
		} else;
	} else;

	if( local_maze.bit.east == false ) {
		if( min_potential > Potential_GetAroundSection( &my, RIGHT ) ) {
			min_potential = Potential_GetAroundSection( &my, RIGHT );
			next_direction = RIGHT;
		} else;
	} else;

	if( local_maze.bit.west == false ) {
		if( min_potential > Potential_GetAroundSection( &my, LEFT ) ) {
			min_potential = Potential_GetAroundSection( &my, LEFT );
			next_direction = LEFT;
		} else;
	} else;

	if( min_potential > Potential_GetAroundSection( &my, REAR ) ) {
		min_potential = Potential_GetAroundSection( &my, REAR );
		next_direction = REAR;
	} else;

	old_direction = next_direction;
	return next_direction;
}

/* ----------------------------------------------------------------------------------
	進む方向の決定（拡張足立法）
-----------------------------------------------------------------------------------*/
int8_t Route_GetNextDirection_PrioritizeUnknown( t_position my )
{
	static int8_t 	old_direction 	= -1;
	int8_t 			next_direction 	= Route_GetNextDirection_Adachi( my );
	t_maze 			local_maze 		= Maze_GetLocal( &my );


	if( (local_maze.bit.west == false) && (Maze_GetIsUnknown(&my, LEFT) == false) ) {
		next_direction = LEFT;
	} else;

	if( (local_maze.bit.east == false) && (Maze_GetIsUnknown(&my, RIGHT) == false) ) {
		next_direction = RIGHT;
	} else;

	// 斜め優先（前回の方向が左だった場合右を優先）
	if( (old_direction == LEFT) && (local_maze.bit.east == false) && (Maze_GetIsUnknown(&my, RIGHT) == false) ) {
		next_direction = RIGHT;
	} else;

	// 斜め優先（前回の方向が右だった場合左を優先）
	if( (old_direction == RIGHT) && (local_maze.bit.west == false) && (Maze_GetIsUnknown(&my, LEFT) == false) ) {
		next_direction = LEFT;
	} else;

	// 直進優先
	if( (local_maze.bit.north == false) && (Maze_GetIsUnknown(&my, FRONT) == false) ) {
		next_direction = FRONT;
	} else;

	old_direction = next_direction;
	return next_direction;
}
