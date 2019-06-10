#include "robonaut_config.h"

#define NODES_NUM 8
#define NODES_DATA_NUM 13
#define MAX_NOT_DECISION_CROSS 8
#define MAX NODES_NUM

uint8_t data_process(uint8_t irany,int32_t x,int32_t y,uint32_t distance,int8_t atsorolas);
uint8_t next_direction(uint8_t irany);
uint8_t follow_the_flow(void);
uint8_t dijkstra(int G[MAX][MAX],int n,int startnode, int atsorolas);
int8_t atsorolashoz(void);
uint8_t ismeretlen_utak(void);
int8_t path_99(void);
void make_cost_table_99(uint8_t goal, uint8_t from);
uint32_t dijkstra_99(int G[MAX][MAX],int n,int startnode, int atsorolas);
void corr_x_y(uint8_t curr_cross, int32_t x,int32_t y);
void make_cost_table(void);
void print_table(void);
void UART2_message2(char *debug_data,int32_t size);
void print_dist(void);
void path_100(void);

