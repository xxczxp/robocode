#ifndef  A_STAR
#define A_STAR
#include <stdio.h> 

//state
#define OPEN 1
#define CLOSE 0
#define OUT 2

//occupy
#define FREE 0
#define OCCUPY 1

//position
#define FORWARD 0
#define OBLIQUE 1
#define EMPTY NULL

typedef struct {
	int loc[2];
	int date[3];   //G H F
	int state;
	int occupy;
	int position;
}node_t;
	
extern void A_star(void);

#endif