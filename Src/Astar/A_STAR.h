#ifndef  A_STAR
#define A_STAR
#include <stdio.h> 

//player 
#define RED  1
#define BLUE  2 

//state
//#define OPEN 1
#define CLOSE 0
#define OPEN 2

//occupy
#define FREE 0
#define OCCUPY 1

//position
#define FORWARD 0
#define OBLIQUE 1
#define EMPTY NULL

//change
#define CHANGE 1
#define INVARIANT 0

//field state
#define WEAK 1
#define STRONG 2

typedef struct {
	int loc[2];
	double data[3];   //G H F
	int state;
	int occupy;
	int position;
	int change;
	int spr;
	int spb;
	int near;
}node_t;
	
extern void A_Star(int player_color);

#endif
