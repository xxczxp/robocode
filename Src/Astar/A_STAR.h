#ifndef  A_STAR
#define A_STAR
#include <stdio.h> 

typedef struct {
	int loc[2];
	float date[3];   //G H F
	int state;
	int occupy;
	int position;
	int change;
	int spr;
	int spb;
	int near;
}node_t;
	
typedef struct{

}pair;
extern void A_star(void);

#endif
