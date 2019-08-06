#ifndef  A_STAR
#define A_STAR
#include <stdio.h> 



typedef struct {
	int loc[2];
	float data[3];   //G H F
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
