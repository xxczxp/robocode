#include "A_STAR.h"
#include <stdio.h>
#include <math.h>

node_t space_A[9][7];
node_t start ;
node_t end;
int last_G;
int value_H = 0;
node_t open_node;
int list_long = 0;
node_t close[14];
node_t open[8];
node_t path_way[10];

//void create_node(int vertical, int transverse, node_t* end){

//node_t a;

//a.loc[0] = vertical;
//a.loc[1] = transverse;

//a.state = OUT;

//a.date[0] = last_G = 0;
//a.date[1] = abs(end -> loc[0]  -  a.loc[0]) + abs(end -> loc[1]  -  a.loc[1]);
//a.date[2] = a.date[0] + a.date[1];
//	
//}







void spot_Init(){
start.loc[0] = 0;
start.loc[1] = 0;
end.loc[0] = 3;
end.loc[1] = 4;
	}


void node_Init(){
for(int h = 0; h < 9; h++){
	for(int z = 0; z < 7; z++){
		node_t a;
		a.loc[0] = h;
		a.loc[1] = z;
		
		a.occupy = FREE;
		
		a.position = EMPTY;
		
		a.state = OUT;
		
		a.date[0] = a.date[1] = a.date[2] = 0;
		
	  space_A[h][z] = a; 
		}
	}
}

void change_space_state(int  vertical, int transverse){
if (space_A[vertical][transverse].state == OUT || space_A[vertical][transverse].state == OPEN){
 space_A[vertical][transverse].state = CLOSE;
}
else{
space_A[vertical][transverse].state = OUT;
}


}

void create_near_node(node_t *node){
int Lx = node -> loc[0];
int Ly = node -> loc[1];
int j = 0;	
	
//  create nearbyv node
int x[8] = {
Lx + 1, Lx - 1, Lx , Lx, Lx + 1, Lx + 1, Lx - 1, Lx - 1 
};

int y[8] = {
Ly, Ly, Ly + 1, Ly - 1, Ly + 1, Ly - 1, Ly + 1, Ly - 1
};

//add it to array
for(int i = 0; i < 4; i++){
	
	if(&space_A[ x [ i ] ] [ y [ i ] ] == NULL){
	 return;
	}
	else if (space_A[ x [ i ] ] [ y [ i ] ].state == CLOSE ){
		return;
	}
	
	else if (space_A[ x [ i ] ] [ y [ i ] ].state == OPEN ){
		return;
	}
	
	else if (space_A[ x [ i ] ] [ y [ i ] ].state == OUT ){
	 space_A[ x [ i ] ] [ y [ i ] ].position = FORWARD; 
	 space_A[ x [ i ] ] [ y [ i ] ].state =OPEN;
		
	 open[j] = space_A[ x [ i ] ] [ y [ i ] ];
		
	j ++;	
	}
	
	else {
	printf("I don't knew what is it!!!");
		return;
	}
}
	
for(int i = 4; i < 8; i++){
	
	if( &space_A[ x [ i ] ] [ y [ i ] ] == NULL){
	return;
	}
	else if (space_A[ x [ i ] ] [ y [ i ] ].state == CLOSE ){
		return;
	}
	
	else if (space_A[ x [ i ] ] [ y [ i ] ].state == OPEN ){
		return;
	}
	
	else if (space_A[ x [ i ] ] [ y [ i ] ].state == OUT ){
	 space_A[ x [ i ] ] [ y [ i ] ].position = FORWARD; 
	 space_A[ x [ i ] ] [ y [ i ] ].state =OPEN;
		
	 open[j] = space_A[ x [ i ] ] [ y [ i ] ];
		
	j ++;	
	}
	
	else {
	printf("I don't knew what is it!!!");
		return;
	}

}

}
//updata node data
void update(node_t* openlist){
	close[list_long] = open_node;
for(int i = 0; i < (sizeof(openlist) / sizeof(node_t)); i++){
	int value;
	if (openlist[i].position == FORWARD){
		value = 10;
	}
	else if(openlist[i].position == OBLIQUE){
		value = 14;
	}
	else{
		printf("position is empty!!!");
		return;
	}
	//assignment G F H
	openlist[i].date[0] = 0.5*(last_G + value);
	openlist[i].date[1] = abs(end.loc[0]-openlist->loc[0]) + abs(end.loc[1]-openlist->loc[1]) ;
	openlist[i].date[2] = openlist[i].date[1] + openlist[i].date[2];
	
	//compare smallest node
	if (openlist[i].date[2] < value_H){
			value_H = openlist[i].date[2];
		  open_node = openlist[i];
		}
	else{
		value_H = value_H;
		open_node =  open_node;
		}
	}
 value_H = 0;
open[list_long] = open_node;
list_long++;
}

//void compare()
void A_star(void){
node_Init();
change_space_state(0, 3);
change_space_state(2, 0);
change_space_state(2, 6);
change_space_state(4, 3);
change_space_state(6, 0);
change_space_state(6, 6);
change_space_state(8, 3);
void spot_Init();
open_node = start;
int breaksign =1;

while(breaksign == 1){
	 create_near_node(&open_node);
		update(open);
	printf("%d    %d    /n", open_node.loc[0], open_node .loc[1]);
		if(open_node.loc[0]  ==  end.loc[0] && open_node.loc[1]  ==  end.loc[1]){
			breaksign = 0;
			break;
		}
	}
}








	