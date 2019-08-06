//#include "A_STAR.h"
//#include "protocol.h"
//#include "referee.h" 
//#include <math.h>

////player 
//#define RED  1
//#define BLUE  2 

////state
////#define OPEN 1
//#define CLOSE 0
//#define OUT 2

////occupy
//#define FREE 0
//#define OCCUPY 1

////position
//#define FORWARD 0
//#define OBLIQUE 1
//#define EMPTY NULL

////change
//#define CHANGE 1
//#define RESET 0

////field state
//#define WEAK 1
//#define STRONG 2


//int player;
//int enemy;

//int last_G;
//int value_H = 10000;

//node_t open_node;
//node_t start ;
//node_t end;
//node_t space_A[9][7];
//node_t have_robot[3];
//node_t open[8];
//node_t path_way[10];

//int blue_Good_l [7][7];
//int red_Good_l  [7][7];

//extern summer_camp_info_t field_info;

//void get_player(int i){
//if (i == RED){
//player = RED;
//enemy = BLUE;
//}
//else{
//player = BLUE;
//enemy = RED;
//}
//}


//void spot_Init(){
//start.loc[0] = 0;
//start.loc[1] = 0;
//end.loc[0] = 3;
//end.loc[1] = 4;
//	}
//node_t placeholder;
//	
//	void placeholder_Init(){
//	placeholder.data[0]= 10000000;
//	placeholder.data[1]= 10000000;
//	placeholder.data[2]= 20000000;
//	}

//void node_Init(){
//for(int h = 0; h < 9; h++){
//	for(int z = 0; z < 7; z++){
//		node_t a;
//		a.loc[0] = h;
//		a.loc[1] = z;
//    
//		a.change = RESET;
//		
//		a.occupy = FREE;
//		
//		a.position = EMPTY;
//		
//		a.state = OUT;
//		
//		a.data[0] = a.data[1] = a.data[2] = 0;
//		
//		a.spb = 0;
//		a.spr = 0;
//		
//		a.near = 0;
//	  space_A[h][z] = a; 
//		}
//	}
//if (player == RED){
//space_A[3][0].near = 0;
//space_A[3][0].spr = 1;
//	
//	space_A[7][0].near = 1;
//space_A[7][0].spr = 1;
//	
//	space_A[0][4].near = 2;
//space_A[0][4].spr = 1;
//	
//	space_A[1][6].near = 5;
//space_A[1][6].spr = 1;
//	
//	space_A[4][4].near = 3;
//space_A[4][4].spr = 1;
//	
//	space_A[5][6].near = 6;
//space_A[5][6].spr = 1;
//	
//	space_A[8][2].near = 4;
//space_A[8][2].spr = 1;
//}
//else if (player == BLUE){
//space_A[1][0].near = 0;
//space_A[1][0].spr = 1;
//	
//	space_A[5][0].near = 1;
//space_A[5][0].spb = 1;
//	
//	space_A[0][2].near = 2;
//space_A[0][2].spb = 1;
//	
//	space_A[3][6].near = 5;
//space_A[3][6].spb = 1;
//	
//	space_A[4][2].near = 3;
//space_A[4][2].spb = 1;
//	
//	space_A[7][6].near = 6;
//space_A[7][6].spb = 1;
//	
//	space_A[8][4].near = 4;
//space_A[8][4].spb = 1;
//}
//else{
//return;
//}
//}

//void change_space_state(int  vertical, int transverse){
//	if (space_A[vertical][transverse].state == OUT ){
//		space_A[vertical][transverse].state = CLOSE;
//	}
//	else{
//		space_A[vertical][transverse].state = OUT;
//	}
//}

//void create_near_node(node_t *node){
//int Lx = node -> loc[0];
//int Ly = node -> loc[1];
//int j = 0;	
//	
////  create nearby node
//int x[8] = {
//Lx + 1, Lx - 1, Lx , Lx, Lx + 1, Lx + 1, Lx - 1, Lx - 1 
//};

//int y[8] = {
//Ly, Ly, Ly + 1, Ly - 1, Ly + 1, Ly - 1, Ly + 1, Ly - 1
//};

////add it to array
//for(int i = 0; i < 4; i++){
//	
//	if(&space_A[ x [ i ] ] [ y [ i ] ] == NULL || space_A[ x [ i ] ] [ y [ i ] ].state == CLOSE){
//	 open[j] = placeholder;
//	}
//	
//	else if (space_A[ x [ i ] ] [ y [ i ] ].state == OUT ){
//	 space_A[ x [ i ] ] [ y [ i ] ].position = FORWARD; 
//	 open[j] = space_A[ x [ i ] ] [ y [ i ] ];
//	}
//	
//	else {
//	printf("I don't knew what is it!!!");
//		return;
//	}
//	j++;
//}
//	
//for(int i = 4; i < 8; i++){
//	
//	if( &space_A[ x [ i ] ] [ y [ i ] ] == NULL || space_A[ x [ i ] ] [ y [ i ] ].state == CLOSE ){
//	open[i] = placeholder;
//	}
//	
//	else if (space_A[ x [ i ] ] [ y [ i ] ].state == OUT ){
//	 space_A[ x [ i ] ] [ y [ i ] ].position = OBLIQUE ; 
//	 open[j] = space_A[ x [ i ] ] [ y [ i ] ];	
//	j ++;	
//	}
//	
//	else {
//	printf("I don't knew what is it!!!");
//		return;
//	}

//}

//}

//void one_more_step(node_t* openlist){
//	//close[list_long] = open_node;
//for(int i = 0; i < 8; i++){
//	int value;
//	if (openlist[i].position == FORWARD){
//		value = 10;
//	}
//	else if(openlist[i].position == OBLIQUE){
//		value = 14;
//	}
//	else{
//		printf("position is empty!!!");
//		return;
//	}
//	//assignment G F H
//	// get G
//	if (player == RED){
//		if (openlist[i].spr == 1){
//		openlist[i].data[0] = 0.8*(last_G + value) - 11+ (field_info.castle_energy[openlist[i].near].energy[1] - field_info.castle_energy[openlist[i].near].energy[0]) / 4;
//		}
//	  else{
//		openlist[i].data[0] = 0.8*(last_G + value);
//		}
//	}
//	
//	if (player == BLUE){
//		if (openlist[i].spb == 1){
//		openlist[i].data[0] = 0.8*(last_G + value) - 11+ (field_info.castle_energy[openlist[i].near].energy[0] - field_info.castle_energy[openlist[i].near].energy[1])/4;
//		}
//	  else{
//		openlist[i].data[0] = 0.8*(last_G + value);
//		}
//	}
//	
//	openlist[i].data[1] = abs(end.loc[0]-openlist->loc[0]) + abs(end.loc[1]-openlist->loc[1]) ;
//	openlist[i].data[2] = openlist[i].data[1] + openlist[i].data[2];
//	
//	//compare smallest node
//	if (openlist[i].data[2] < value_H){
//			value_H = openlist[i].data[2];
//		  open_node = openlist[i];
//		}
//	else{
//		value_H = value_H;
//		open_node =  open_node;
//		}
//	}
// value_H = 10000;
//open_node.state = CLOSE;
//}

////reload field information
//void updata_data(summer_camp_info_t* field_info){
// int list_num = 0;
//	for(int i = 0;  i < 9;  i ++ ){
//	for(int j = 0; j < 7; j++){
//	  if(field_info -> region_occupy[i][j].have_robot == 1){
//			have_robot[list_num] = space_A[i][j];
//			space_A[i][j].state = CLOSE;
//		}	
//		if(field_info -> region_occupy[i][j].belong == player && space_A[i][j].state  != CLOSE  && space_A[i][j].change ==RESET){
//				space_A[i][j].data[0] +=4;
//			 	//space_A[i][j].change = CHANGE;
//		}
//		 if (field_info -> region_occupy[i][j].belong == enemy && space_A[i][j].state  != CLOSE  ){
//				if (field_info -> region_occupy[i][j].status == STRONG ){
//				space_A[i][j].data[0] += 10;
//				space_A[i][j].change = CHANGE;
//				}
//				
//				else{
//				space_A[i][j].data[0] += 6;
//				space_A[i][j].change =CHANGE;
//				 }
//			 }
//		 }
//	 }
//			
//		
// }



//void A_Star(int player_color){
//get_player(player_color);
//spot_Init();
//placeholder_Init();
//node_Init();
//change_space_state(0, 3);
//change_space_state(2, 0);
//change_space_state(2, 6);
//change_space_state(4, 3);
//change_space_state(6, 0);
//change_space_state(6, 6);
//change_space_state(8, 3);
//open_node = start;

//int breaksign =1;

//while(breaksign == 1){
//	 create_near_node(&open_node);
//		one_more_step(open);
//	printf("%d    %d    /n", open_node.loc[0], open_node .loc[1]);
//		if(open_node.loc[0]  ==  end.loc[0] && open_node.loc[1]  ==  end.loc[1]){
//			breaksign = 0;
//			break;
//		}
//	}
//}
