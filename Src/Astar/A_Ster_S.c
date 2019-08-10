#include "A_STAR.h"
#include "protocol.h"
#include "referee.h" 
#include <stdio.h>
#include <math.h>
#include "chassis_control.h"

int player = RED;
int enemy;
QueueHandle_t point_queue;
int last_G;
int value_F = 10000;
int way_n = 0;
int ball_num = 4;
int cup_num = 30;

node_t current_node;
node_t last_node;
node_t target_node;
node_t start;
node_t space_A[9][7];
node_t have_robot;
node_t open[8];
node_t path_way[15];
node_t placeholder;

extern summer_camp_info_t field_info;

void get_enemy(){
	if(player == RED){
		enemy = BLUE;
	}
	else{
		enemy = RED;
	}
}

void node_Init(){
	for(int h = 0; h < 9; h++ ){
		
		for(int z = 0; z < 7; z++){
			node_t a;
			
			a.change = INVARIANT;
			
			a.loc[0] = h;
			a.loc[1] = z;
			
			a.data[0] = a.data[1] = a.data[2] = 0;
			
			a.occupy = FREE;

			a.state = OPEN;
			
			a.spb = a.spr = 0;
			
			space_A[h][z] = a;
			
		}
	}
if (player == RED){
space_A[3][0].near = 0;
space_A[3][0].spr = 1;
	
space_A[7][0].near = 1;
space_A[7][0].spr = 1;
	
space_A[0][4].near = 2;
space_A[0][4].spr = 1;
	
space_A[1][6].near = 5;
space_A[1][6].spr = 1;
	
space_A[4][4].near = 3;
space_A[4][4].spr = 1;
	
space_A[5][6].near = 6;
space_A[5][6].spr = 1;
	
space_A[8][2].near = 4;
space_A[8][2].spr = 1;
}
else if (player == BLUE){
space_A[1][0].near = 0;
space_A[1][0].spr = 1;
	
space_A[5][0].near = 1;
space_A[5][0].spb = 1;
	
space_A[0][2].near = 2;
space_A[0][2].spb = 1;
	
space_A[3][6].near = 5;
space_A[3][6].spb = 1;
	
space_A[4][2].near = 3;
space_A[4][2].spb = 1;
	
space_A[7][6].near = 6;
space_A[7][6].spb = 1;
	
space_A[8][4].near = 4;
space_A[8][4].spb = 1;
}
else{
	return;
}

space_A[0][3].state = CLOSE;
space_A[2][0].state = CLOSE;
space_A[2][6].state = CLOSE;
space_A[4][3].state = CLOSE;
space_A[6][0].state = CLOSE;
space_A[6][6].state = CLOSE;
space_A[8][3].state = CLOSE;

placeholder.spb = placeholder.spr = 0;
placeholder.state = OPEN;
placeholder.data[2] = 1000000;
placeholder.occupy = FREE;

target_node.data[0] = 1000000;
target_node.loc[0] = 5;
target_node.loc[1] = 4;

}

void updata(summer_camp_info_t* field_info){
//	int list_num = 0;
	int x = field_info->car_location[enemy]>>4;
	int y = field_info->car_location[enemy] & 0xf;
	space_A[x][y].state = CLOSE;
	
	for(int i = 0;  i < 9;  i ++ ){
		for(int j = 0; j < 7; j++){
//			if(field_info -> region_occupy[i][j].have_robot == 1){
//				have_robot = space_A[i][j];
//				space_A[i][j].state = CLOSE;
//			}	
			
			if(field_info -> region_occupy[i][j].belong == player && space_A[i][j].change ==INVARIANT){
				space_A[i][j].data[0] +=2;
				space_A[i][j].change = CHANGE;
				switch(player){
					case RED : {
						if(space_A[i][j].spr == 1){
							space_A[i][j].data[0] = -13 + (field_info->castle_energy[space_A[i][j].near].energy[1] - field_info->castle_energy[space_A[i][j].near].energy[0]) / 4;
							space_A[i+1][j].data[0] = space_A[i][j].data[0] *0.8;
							space_A[i-1][j].data[0] = space_A[i][j].data[0] *0.8;
							space_A[i][j+1].data[0] = space_A[i][j].data[0] *0.8;
							space_A[i][j-1].data[0] = space_A[i][j].data[0] *0.8;
					  }
					  else{
							space_A[i][j].data[0] = 0;
						}
					}
					
					case BLUE : {
						if(space_A[i][j].spb == 1){
							space_A[i][j].data[0] = -13 + (field_info->castle_energy[space_A[i][j].near].energy[0] - field_info->castle_energy[space_A[i][j].near].energy[1]) / 4;
							space_A[i+1][j].data[0] = space_A[i][j].data[0] *1.2;
							space_A[i-1][j].data[0] = space_A[i][j].data[0] *1.2;
							space_A[i][j+1].data[0] = space_A[i][j].data[0] *1.2;
							space_A[i][j-1].data[0] = space_A[i][j].data[0] *1.2;
						 }
						else{
							space_A[i][j].data[0] = 0;							
			  	 }
					}
				 }
				}
			 else if(field_info -> region_occupy[i][j].belong == enemy ){
				if (field_info -> region_occupy[i][j].status == STRONG ){
					space_A[i][j].data[0] = 6;
						
					switch(player){
					case RED : {
						if(space_A[i][j].spr == 1){
							space_A[i][j].data[0] = -13 + (field_info->castle_energy[space_A[i][j].near].energy[1] - field_info->castle_energy[space_A[i][j].near].energy[0]) / 4;
							space_A[i+1][j].data[0] = space_A[i][j].data[0] *0.8;
							space_A[i-1][j].data[0] = space_A[i][j].data[0] *0.8;
							space_A[i][j+1].data[0] = space_A[i][j].data[0] *0.8;
							space_A[i][j-1].data[0] = space_A[i][j].data[0] *0.8;
							}
						else{
							space_A[i][j].data[0] = 0;
						}
					}
					
					case BLUE : {
						if(space_A[i][j].spb == 1){
							space_A[i][j].data[0] = -13 + (field_info->castle_energy[space_A[i][j].near].energy[0] - field_info->castle_energy[space_A[i][j].near].energy[1]) / 4;
							space_A[i+1][j].data[0] = space_A[i][j].data[0] *1.2;
							space_A[i-1][j].data[0] = space_A[i][j].data[0] *1.2;
							space_A[i][j+1].data[0] = space_A[i][j].data[0] *1.2;
							space_A[i][j-1].data[0] = space_A[i][j].data[0] *1.2;
							}
						else{
							space_A[i][j].data[0] = 0;							
					}
				}
			}
		}
				else if(field_info -> region_occupy[i][j].status == WEAK){
					space_A[i][j].data[0] = 4;
						
					switch(player){
						case RED : {
							if(space_A[i][j].spr == 1){
								space_A[i][j].data[0] = -13 + (field_info->castle_energy[space_A[i][j].near].energy[1] - field_info->castle_energy[space_A[i][j].near].energy[0]) / 4;
								space_A[i+1][j].data[0] = space_A[i][j].data[0] *0.8;
								space_A[i-1][j].data[0] = space_A[i][j].data[0] *0.8;
								space_A[i][j+1].data[0] = space_A[i][j].data[0] *0.8;
								space_A[i][j-1].data[0] = space_A[i][j].data[0] *0.8;
								}
						else{
							space_A[i][j].data[0] = 0;
						}
					}
					
						case BLUE : {
							if(space_A[i][j].spb == 1){
								space_A[i][j].data[0] = -13 + (field_info->castle_energy[space_A[i][j].near].energy[0] - field_info->castle_energy[space_A[i][j].near].energy[1]) / 4;
								space_A[i+1][j].data[0] = space_A[i][j].data[0] *1.2;
								space_A[i-1][j].data[0] = space_A[i][j].data[0] *1.2;
								space_A[i][j+1].data[0] = space_A[i][j].data[0] *1.2;
								space_A[i][j-1].data[0] = space_A[i][j].data[0] *1.2;
						}
						else{
							space_A[i][j].data[0] = 0;							
					}
				}
			}			
		}
				
			   else{
					 switch(player){
						 case RED : {
							 if(space_A[i][j].spr == 1){
								 space_A[i][j].data[0] = -13 + (field_info->castle_energy[space_A[i][j].near].energy[1] - field_info->castle_energy[space_A[i][j].near].energy[0]) / 4;
								 space_A[i+1][j].data[0] = space_A[i][j].data[0] *0.8;
								 space_A[i-1][j].data[0] = space_A[i][j].data[0] *0.8;
								 space_A[i][j+1].data[0] = space_A[i][j].data[0] *0.8;
								 space_A[i][j-1].data[0] = space_A[i][j].data[0] *0.8;
						   }
							else{
								space_A[i][j].data[0] = 0;
						 }
					 }
					
						 case BLUE : {
							 if(space_A[i][j].spb == 1){
								 space_A[i][j].data[0] = -13 + (field_info->castle_energy[space_A[i][j].near].energy[0] - field_info->castle_energy[space_A[i][j].near].energy[1]) / 4;
								 space_A[i+1][j].data[0] = space_A[i][j].data[0] *1.2;
								 space_A[i-1][j].data[0] = space_A[i][j].data[0] *1.2;
								 space_A[i][j+1].data[0] = space_A[i][j].data[0] *1.2;
								 space_A[i][j-1].data[0] = space_A[i][j].data[0] *1.2;
						  }
							 else{
								space_A[i][j].data[0] = 0;							
					 }
				  }
			   }
			  }
		   }
			
	     if (target_node.data[0] > space_A[i][j].data[0] && abs(current_node.loc[0] - target_node.loc[0]) + abs(current_node.loc[0] - target_node.loc[1]) < 12 ){
				 target_node = space_A[i][j];
   }
	}	 
 }
}


void take_a_step(node_t c_n){
	last_G = current_node.data[0];
	float time = field_info.round_remain_tick;
	int Lx = c_n.loc[0];
	int Ly = c_n.loc[1];
	
	current_node.state = CLOSE;
	last_node = current_node; 
	
	int x[8] = {
	Lx + 1, Lx - 1, Lx , Lx, Lx + 1, Lx + 1, Lx - 1, Lx - 1 
	};

	int y[8] = {
	Ly, Ly, Ly + 1, Ly - 1, Ly + 1, Ly - 1, Ly + 1, Ly - 1
	};
	
	int j = 0;
//add it to array
	for(int i = 0; i < 4; i++){
		j++;
	
		if(&space_A[ x [ i ] ] [ y [ i ] ] == NULL || space_A[ x [ i ] ] [ y [ i ] ].state == CLOSE){
			open[j] = placeholder;
		}
	
		else if (space_A[ x [ i ] ] [ y [ i ] ].state == OPEN ){
			space_A[ x [ i ] ] [ y [ i ] ].position = FORWARD; 
			open[j] = space_A[ x [ i ] ] [ y [ i ] ];
		}
	
		else {
			printf("I don't knew what is it!!!");
			return;
	 }
	}
	
	for(int i = 4; i < 8; i++){
		j++;
		
		if( &space_A[ x [ i ] ] [ y [ i ] ] == NULL || space_A[ x [ i ] ] [ y [ i ] ].state == CLOSE ){
			open[i] = placeholder;
		}
	
	else if (space_A[ x [ i ] ] [ y [ i ] ].state == OPEN ){
	  space_A[ x [ i ] ] [ y [ i ] ].position = OBLIQUE ; 
	  open[j] = space_A[ x [ i ] ] [ y [ i ] ];	
	 }
	
	else {
	  printf("I don't knew what is it!!!");
		return;
	}
}
	

	for(int i = 0; i < 8; i++){
		if (open[i].position == FORWARD){
			open[i].data[0] += 10;
		}
		else if(open[i].position == OBLIQUE){
			open[i].data[0] += 14;
		}
		else{
			printf("position is empty!!!");
			return;
		}
	open[i].data[0] += 0.8*last_G;
	open[i].data[1] =  abs(target_node.loc[0]-open[i].loc[0]) + abs(target_node.loc[1]-open[i].loc[1]);
	open[i].data[2] = open[i].data[0] + open[i].data[1];	

	if(open[i].data[2] < value_F){
		last_G = open[i].data[0];
		value_F = open[i].data[2];
		current_node = open[i];
	}
	else{
		last_G = last_G;
		value_F = value_F;
		current_node = current_node;
	}
}

	path_way[way_n] = current_node;
	way_n++;

	if (player == RED){
		if(current_node.spr == 1&& time > 5){
			auto_pack_t pack;
			pack.cmd = PUT_BALL_CMD;
			pack.target.x = current_node.loc[0];
			pack.target.y = current_node.loc[1];
			pack.target.w = 0;
			if (cup_num > (field_info.castle_energy[current_node.near].energy[1]-field_info.castle_energy[current_node.near].energy[0])/4 && ball_num > pack.ball_num > (field_info.castle_energy[current_node.near].energy[1]-field_info.castle_energy[current_node.near].energy[0])%4){
			pack.ball_num = (field_info.castle_energy[current_node.near].energy[1]-field_info.castle_energy[current_node.near].energy[0])%4;
			pack.cup_num = (field_info.castle_energy[current_node.near].energy[1]-field_info.castle_energy[current_node.near].energy[0])/4;
			}
			else if(cup_num > (field_info.castle_energy[current_node.near].energy[1]-field_info.castle_energy[current_node.near].energy[0])/4 && ball_num > pack.ball_num > field_info.castle_energy[current_node.near].energy[1]-field_info.castle_energy[current_node.near].energy[0]){
			pack.cup_num = cup_num;
			pack.ball_num =field_info.castle_energy[current_node.near].energy[1]-field_info.castle_energy[current_node.near].energy[0];
			
			}
			else{
			pack.cup_num = cup_num;
			pack.ball_num  = ball_num;
			}
			xQueueReceive(auto_queue,&pack,5);
		}
	
		else if(current_node.spr == 1 && time<5){
			auto_pack_t pack;
			pack.ball_num = 0;
			pack.cmd = MOVE_CMD;
			pack.cup_num = 0;
			pack.target.x = last_node.loc[0];
			pack.target.y = last_node.loc[1];
			pack.target.w = 0;
			xQueueReceive(auto_queue,&pack,5);
	}
		else if(current_node.spr != 1 && time>3){
			auto_pack_t pack;
			pack.ball_num = 0;
			pack.cmd = MOVE_CMD;
			pack.cup_num = 0;
			pack.target.x = current_node.loc[0];
			pack.target.y = current_node.loc[1];
			pack.target.w = 0;
			xQueueReceive(auto_queue,&pack,5);
	 }
		else{
			auto_pack_t pack;
			pack.ball_num = 0;
			pack.cup_num = 0;
			pack.cmd = MOVE_CMD;
			pack.target.x = last_node.loc[0];
			pack.target.y = last_node.loc[1];
			pack.target.w = 0;
			xQueueReceive(auto_queue,&pack,5);
		}
}
	else{
		if(current_node.spb == 1){
			auto_pack_t pack;
			pack.cmd = PUT_BALL_CMD;
			pack.target.x = current_node.loc[0];
			pack.target.y = current_node.loc[1];
			pack.target.w = 0;
			
			if (cup_num > (field_info.castle_energy[current_node.near].energy[0]-field_info.castle_energy[current_node.near].energy[1])/4 && ball_num > pack.ball_num > (field_info.castle_energy[current_node.near].energy[0]-field_info.castle_energy[current_node.near].energy[1])%4){
			pack.ball_num = (field_info.castle_energy[current_node.near].energy[0]-field_info.castle_energy[current_node.near].energy[1])%4;
			pack.cup_num = (field_info.castle_energy[current_node.near].energy[0]-field_info.castle_energy[current_node.near].energy[1])/4;
			}
			else if(cup_num > (field_info.castle_energy[current_node.near].energy[0]-field_info.castle_energy[current_node.near].energy[1])/4 && ball_num > pack.ball_num > field_info.castle_energy[current_node.near].energy[0]-field_info.castle_energy[current_node.near].energy[1]){
			pack.cup_num = cup_num;
			pack.ball_num =field_info.castle_energy[current_node.near].energy[0]-field_info.castle_energy[current_node.near].energy[1];
			
			}
			else{
			pack.cup_num = cup_num;
			pack.ball_num = ball_num;
			}
						
			xQueueReceive(auto_queue,&pack,5);
			}
		else if(current_node.spr == 1 && time<5){
			auto_pack_t pack;
			pack.ball_num = 0;
			pack.cmd = MOVE_CMD;
			pack.cup_num = 0;
			pack.target.x = last_node.loc[0];
			pack.target.y = last_node.loc[1];
			pack.target.w = 0;
			xQueueReceive(auto_queue,&pack,5);
			}
		else if(current_node.spr != 1 && time>3){
			auto_pack_t pack;
			pack.ball_num = 0;
			pack.cmd = MOVE_CMD;
			pack.cup_num = 0;
			pack.target.x = current_node.loc[0];
			pack.target.y = current_node.loc[1];
			pack.target.w = 0;
			xQueueReceive(auto_queue,&pack,5);
	}
		else{
			auto_pack_t pack;
			pack.ball_num = 0;
			pack.cup_num = 0;
			pack.cmd = MOVE_CMD;
			pack.target.x = last_node.loc[0];
			pack.target.y = last_node.loc[1];
			pack.target.w = 0;
			xQueueReceive(auto_queue,&pack,5);
		}
	}
}


void deInit(){
	for(int i = 0; i < 9; i++){
		for(int j = 0; j < 7; j++){
		space_A[i][j].state = OPEN;
		space_A[i][j].state = INVARIANT;
		}
	}
	space_A[0][3].state = CLOSE;
	space_A[2][0].state = CLOSE;
	space_A[2][6].state = CLOSE;
	space_A[4][3].state = CLOSE;
	space_A[6][0].state = CLOSE;
	space_A[6][6].state = CLOSE;
	space_A[8][3].state = CLOSE;
}

void A_star(){
	get_enemy();
	node_Init();
	if(field_info.round_remain_cnt > 0){
	updata(&field_info);
	take_a_step(current_node);
	deInit();
  updata(&field_info);
  }
	else{
		auto_pack_t pack;
		pack.cmd = STOP;
		xQueueReceive(auto_queue,&pack,5);
	}
}

