
#include "defines.h"
#include "global.h"

#define NUM_PATH	255

volatile uint8_t	num;
volatile t_path		path[NUM_PATH];

/* ----------------------------------------------------------------------------------
	パスの初期設定
-----------------------------------------------------------------------------------*/
void Path_Reset( void )
{
	for( num = 0; num < NUM_PATH-1; num++ ) {
		path[num].straight = 0;
		path[num].direction = 0;
		path[num].type = 0;
	}
	num = 0;
}

/* ----------------------------------------------------------------------------------
	直線パスのセット
-----------------------------------------------------------------------------------*/
void Path_SetStraightSection( int8_t n )
{
	path[num].straight += n;
}

/* ----------------------------------------------------------------------------------
	ターンパスのセット
-----------------------------------------------------------------------------------*/
void Path_SetTurnSection( int8_t type, int8_t direction )
{
	path[num].direction = direction;
	path[num].type = type;
	num++;
}

/* ----------------------------------------------------------------------------------
	生成したパスの参照
-----------------------------------------------------------------------------------*/
t_path Path_GetSequence( uint8_t n )
{
	return path[n];
}

t_path Path_GetReturnSequence( uint8_t n )
{
	t_path return_path;
	if( (num-1) - n == 0 ) {
		return_path.straight	= path[0].straight;
		return_path.type		= goal;
		return_path.direction	= 0;

	} else if( (num-1) - n-1 < 0 ) {
		return_path.straight	= 0;
		return_path.type		= turn_0;
		return_path.direction	= -1;

	} else {
		return_path.straight	= path[(num-1)-n].straight;

		if( path[(num-1)-n-1].type == turn_45in ) {
			return_path.type = turn_45out;
		} else if( path[(num-1)-n-1].type == turn_135in ) {
			return_path.type = turn_135out;
		} else if( path[(num-1)-n-1].type == turn_45out ) {
			return_path.type = turn_45in;
		} else if( path[(num-1)-n-1].type == turn_135out ) {
			return_path.type = turn_135in;
		} else {
			return_path.type = path[(num-1)-n-1].type;
		}

		if( path[(num-1)-n-1].direction == RIGHT ) {
			return_path.direction = LEFT;
		} else if( path[(num-1)-n-1].direction == LEFT ) {
			return_path.direction = RIGHT;
		} else {
			return_path.direction = -1;
		}
	}
	return return_path;
}

/* ----------------------------------------------------------------------------------
	生成したパス数の参照
-----------------------------------------------------------------------------------*/
uint8_t Path_GetSequenceNumber( void )
{
	uint8_t		i;
	for( i = 0; i < NUM_PATH-1; i++ ) {
		if( path[i].type == turn_0 || path[i].type == goal ) {
			break;
		} else;
	}
	return i;
}

/* ----------------------------------------------------------------------------------
	大回りターンのパス変換
-----------------------------------------------------------------------------------*/
void Path_ConvertTurnLarge( void )
{
	uint8_t i;

	for( i = 0; i < NUM_PATH-1; i++ ) {
		if( path[i].type == goal ) {
			num = i + 1;
			break;
		} else;

		if( path[i].type == turn_90 ) {
			if( (path[i].straight > 0) && (path[i+1].straight > 0) ) {
				path[i].straight--;
				path[i].type = turn_large;
				path[i+1].straight--;
			} else;
		} else;
	}
}

/* ----------------------------------------------------------------------------------
	180度ターンのパス変換
-----------------------------------------------------------------------------------*/
void Path_ConvertTurn180( void )
{
	uint8_t i, j;

	for( i = 0; i < NUM_PATH-1; i++ )	{
		if( path[i].type == goal ) {
			num = i + 1;
			break;
		} else;

		if( (path[i].straight > 0) && (path[i+1].straight == 0) && (path[i+2].straight > 0) ) {
			if( (path[i].type == turn_90) && (path[i+1].type == turn_90) && (path[i].direction == path[i+1].direction) ) {
				path[i].straight--;
				path[i].type = turn_180;
				path[i+2].straight--;
				for( j = i + 1; j < NUM_PATH; j++ ) {
					if( path[j].type == goal ) {
						break;
					} else;

					path[j].straight = path[j+1].straight;
					path[j].direction = path[j+1].direction;
					path[j].type = path[j+1].type;
				}
			} else;
		} else;
	}
}

/* ----------------------------------------------------------------------------------
	斜め侵入型45度ターンのパス判定
-----------------------------------------------------------------------------------*/
int8_t Path_IsTurn45in( uint8_t i )
{
	if( (path[i].straight > 0) && (path[i+1].straight == 0 )) {
		if( (path[i].type == turn_90) && (path[i+1].type == turn_90)) {
			return( path[i].direction != path[i+1].direction );
		} else;
	}else;
	return(false);
}

/* ----------------------------------------------------------------------------------
	斜め脱出型45度ターンのパス判定
-----------------------------------------------------------------------------------*/
int8_t Path_IsTurn45out( uint8_t i )
{
	if( (path[i].straight == 0) && (path[i+1].straight > 0) ) {
		if( (path[i-1].type == turn_90) && (path[i].type == turn_90) ) {
			return( path[i-1].direction != path[i].direction );
		} else if( (path[i-1].type == turn_45in) && (path[i].type == turn_90) ) {
			return( path[i-1].direction != path[i].direction );
		} else;
	} else;
	return( false );
}

/* ----------------------------------------------------------------------------------
	斜め侵入型135度ターンのパス判定
-----------------------------------------------------------------------------------*/
int8_t Path_IsTurn135in( uint8_t i )
{
	if( (path[i].straight > 0) && (path[i+1].straight == 0) && (path[i+2].straight == 0) ) {
		if( (path[i].type == turn_90) && (path[i+1].type == turn_90) ) {
			return( path[i].direction == path[i+1].direction );
		} else;
	} else;
	return( false );
}

/* ----------------------------------------------------------------------------------
	斜め脱出型135度ターンのパス判定
-----------------------------------------------------------------------------------*/
int8_t Path_IsTurn135out( uint8_t i )
{
	if( (path[i].straight == 0) && (path[i+1].straight == 0) && (path[i+2].straight > 0) ) {
		if( (path[i].type == turn_90) && (path[i+1].type == turn_90) ) {
				return( path[i].direction == path[i+1].direction );
		} else;
	} else;
	return( false );
}

/* ----------------------------------------------------------------------------------
	90度ターンのパス判定
-----------------------------------------------------------------------------------*/
int8_t Path_IsTurn90v( uint8_t i )
{
	if( (path[i].straight == 0) && (path[i+1].straight == 0) && (path[i+2].straight == 0) ) {
		if( (path[i].type == turn_90) && (path[i+1].type == turn_90) ) {
			return( path[i].direction == path[i+1].direction );
		} else;
	} else;
	return( false );
}

/* ----------------------------------------------------------------------------------
	90度ターンのパス変換
-----------------------------------------------------------------------------------*/
void Path_ConvertTurn90v( uint8_t i )
{
	uint8_t j;

	path[i].type = turn_90v;
	for( j = i + 1; j < NUM_PATH-1; j++ ) {
		if( path[j].type == goal ) {
			break;
		} else;

		path[j].straight = path[j+1].straight;
		path[j].direction = path[j+1].direction;
		path[j].type = path[j+1].type;
	}
}

/* ----------------------------------------------------------------------------------
	斜め侵入型45度ターンのパス変換
-----------------------------------------------------------------------------------*/
void Path_ConvertTurn45in( uint8_t i )
{
	path[i].straight--;
	path[i].type = turn_45in;
}

/* ----------------------------------------------------------------------------------
	斜め脱出型45度ターンのパス変換
-----------------------------------------------------------------------------------*/
void Path_ConvertTurn45out( uint8_t i )
{
	path[i+1].straight--;
	path[i].type = turn_45out;
}

/* ----------------------------------------------------------------------------------
	斜め侵入型135度ターンのパス変換
-----------------------------------------------------------------------------------*/
void Path_ConvertTurn135in( uint8_t i )
{
	uint8_t j;

	path[i].straight--;
	path[i].type = turn_135in;
	for( j = i + 1; j < NUM_PATH-1; j++ ) {
		if( path[j].type == goal ) {
			break;
		} else;

		path[j].straight = path[j+1].straight;
		path[j].direction = path[j+1].direction;
		path[j].type = path[j+1].type;
	}
}

/* ----------------------------------------------------------------------------------
	斜め脱出型135度ターンのパス変換
-----------------------------------------------------------------------------------*/
void Path_ConvertTurn135out( uint8_t i )
{
	uint8_t j;

	path[i+2].straight--;
	path[i].type = turn_135out;
	for( j = i + 1; j < NUM_PATH-1; j++ ) {
		if( path[j].type == goal ) {
			break;
		} else;

		path[j].straight = path[j+1].straight;
		path[j].direction = path[j+1].direction;
		path[j].type = path[j+1].type;
	}
}

/* ----------------------------------------------------------------------------------
	斜めのパス変換
-----------------------------------------------------------------------------------*/
void Path_ConvertDiagonal( void )
{
	volatile uint8_t i, j, k;

	for( i = 0; i < NUM_PATH-1; i++ ) {
		if( path[i].type == goal ) {
			break;
		} else;

		if( (Path_IsTurn45in(i) == true) || (Path_IsTurn135in(i) == true) ) {
			for( j = i; j < NUM_PATH-1; j++ ) {
				if( path[j].type == goal ) {
					break;
				} else;
				if( (Path_IsTurn45out(j) == true) || (Path_IsTurn135out(j) == true) ) break;
			}
			if( path[j].type != goal ) {
				if( Path_IsTurn45in(i) == true )	Path_ConvertTurn45in(i);
				if( Path_IsTurn45out(j) == true )	Path_ConvertTurn45out(j);
				if( Path_IsTurn135out(j) == true )	Path_ConvertTurn135out(j);
				if( Path_IsTurn135in(i) == true )	Path_ConvertTurn135in(i);
				for( k = i + 1; k < j; k++ ) {
					if(Path_IsTurn90v(k) == true ) {
						Path_ConvertTurn90v(k);
						j--;
					} else;
				}
			} else;
		} else;
	}

	for( i = 0; i < NUM_PATH-1; i++ ) {
		if( path[i].type == goal ) {
			num = i + 1;
			break;
		} else;

		if( path[i].type >= turn_45in ) {
			i++;
			for( j = i; path[j].type < turn_45out; j++ ) {
				if( path[j].type == turn_90v ) {
					path[j].straight = j - i;
					for( k = i; k < NUM_PATH-1; k++ ) {
						if( path[k].type == goal ) {
							break;
						} else;
						path[k].straight	= path[k+j-i].straight;
						path[k].direction	= path[k+j-i].direction;
						path[k].type	 	= path[k+j-i].type;
					}
					j = i;
					i++;
				} else;
			}
			path[j].straight = j - i;
			for( k = i; k < NUM_PATH-1; k++ ) {
				if( path[k].type == goal ) {
					break;
				} else;
				path[k].straight	= path[k+j-i].straight;
				path[k].direction	= path[k+j-i].direction;
				path[k].type	 	= path[k+j-i].type;
			}
		} else;
	}
}

/* ----------------------------------------------------------------------------------
	拡張ターンのパス変換
-----------------------------------------------------------------------------------*/
void Path_ConvertAdvancedTurn( void )
{
	for( int16_t i = 0; i < NUM_PATH-1; i++ ) {
		if( path[i].type == goal ) {
			break;
		} else;

		if( path[i].type == turn_45out && path[i+1].straight == 0 && path[i+1].type == turn_45in ) {
			if( path[i].direction == path[i+1].direction ) {
				path[i].type = turn_kojima;
				for( int16_t j = i + 1; j < NUM_PATH-1; j++ ) {
					if( path[j].type == goal ) {
						break;
					} else;

					path[j].straight = path[j+1].straight;
					path[j].direction = path[j+1].direction;
					path[j].type = path[j+1].type;
				}
			} else;
		} else;
	}
}

/* ----------------------------------------------------------------------------------
	パスの表示
-----------------------------------------------------------------------------------*/
void Path_Display( void )
{
	uint8_t i;

	printf("\n\rStart!\n\r");
	for( i = 0; i < NUM_PATH-1; i++ ) {

		// 直線区間の表示
		if( path[i].straight != 0 ) {
			if( path[i].type >= turn_90v && path[i].type <= turn_135out ) {
				printf("Diagonal : %2d\n\r", path[i].straight);
			} else {
				printf("Straight : %2d\n\r", path[i].straight);
			}
		}

		// ターンの表示
		switch (path[i].type) {
			case turn_0:		printf("Turn_0");		break;
			case turn_90:		printf("Turn_90");		break;
			case turn_large:	printf("Turn_Large");	break;
			case turn_180:		printf("Turn_180");		break;
			case turn_90v:		printf("Turn_90v");		break;
			case turn_45in:		printf("Turn_45in");	break;
			case turn_135in:	printf("Turn_135in");	break;
			case turn_45out:	printf("Turn_45out");	break;
			case turn_135out:	printf("Turn_135out");	break;
			case turn_kojima:	printf("Turn_Kojima");	break;
			case goal:			printf("Goal!");		break;
			default:			printf("ERROR!\n\r");	break;
		}

		if( (path[i].type == turn_0) || (path[i].type == goal) ) {
			break;
		} else {
			if( path[i].direction == RIGHT ) 	 	printf("_RIGHT\r\n");
			else if( path[i].direction == LEFT )	printf("_LEFT\r\n");
			else								 	printf("_ERROR\r\n");
		}
	}
	printf("\n\r");
}
