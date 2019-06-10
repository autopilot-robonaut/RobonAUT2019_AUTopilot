#include "labirintus.h"

#define max_get_message_size 2200

int32_t data_table[NODES_NUM][NODES_DATA_NUM]={{-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},{-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},{-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},{-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},{-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},{-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},{-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},{-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1}};
uint8_t irany=0,konwncross=0; //utolso keresztezodes azert 2 elemu mert az is fontos h honna jovunk
int8_t decision_list[MAX_NOT_DECISION_CROSS][2]={{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}};// azokat a kereesztezodeseket taroljuk, amikneken atjottunk az utolso dontesi keresztezodésen
uint8_t newcross,current_cross,flow;
int8_t last_back_decision=-1;
int8_t last_cross[2]={-1,-1};
int32_t G[MAX][MAX]={{-1,-1,-1,-1,-1,-1,-1,-1},{-1,-1,-1,-1,-1,-1,-1,-1},{-1,-1,-1,-1,-1,-1,-1,-1},{-1,-1,-1,-1,-1,-1,-1,-1},{-1,-1,-1,-1,-1,-1,-1,-1},{-1,-1,-1,-1,-1,-1,-1,-1},{-1,-1,-1,-1,-1,-1,-1,-1},{-1,-1,-1,-1,-1,-1,-1,-1},};
int32_t G_99[MAX][MAX]={{-1,-1,-1,-1,-1,-1,-1,-1},{-1,-1,-1,-1,-1,-1,-1,-1},{-1,-1,-1,-1,-1,-1,-1,-1},{-1,-1,-1,-1,-1,-1,-1,-1},{-1,-1,-1,-1,-1,-1,-1,-1},{-1,-1,-1,-1,-1,-1,-1,-1},{-1,-1,-1,-1,-1,-1,-1,-1},{-1,-1,-1,-1,-1,-1,-1,-1},};
uint8_t ismeretlen_utak_szama=0;
uint8_t next_dir;
uint32_t min_dist=99999999;
uint8_t cross_count=0;
int8_t cross_99 = -1;
	
char debug_data2[64];
char debug_message2[2000];
uint32_t msg_index2;
	
int32_t last_lab_x,last_lab_y;
uint32_t dist;
uint32_t dist_array[7]={0,0,0,0,0,0,0};
uint8_t dist_ind=0;


uint8_t data_process(uint8_t irany,int32_t x,int32_t y,uint32_t distance,int8_t atsorolas)
{
	// voltunk már ebben a keresztezodésben?
	newcross=1;
	min_dist=99999999;
	last_lab_x = x;
	last_lab_y = y;
	for(uint8_t i=0;i<konwncross;i++)
	{
		if(abs(data_table[i][0]-x)<700 && abs(data_table[i][1]-y)<700 && pow(data_table[i][0]-x,2)+pow(data_table[i][1]-y,2)< min_dist ) 
		{
			min_dist=pow(data_table[i][0]-x,2)+pow(data_table[i][1]-y,2);
			newcross=0;current_cross=i;
		}
	}
	if(newcross) // ha meg nem volt meg ilyen keresztezodes akkor hozzáfuzzük 
	{
		current_cross=konwncross;
		konwncross++;
	}
	else
	{
		corr_x_y(current_cross, x, y);
	}
	data_table[current_cross][12]=cross_count;
	cross_count++;
	if(konwncross<9)
	{				
		// keresztezodések koordinátájának frissiteése
		if(newcross) {data_table[current_cross][0]=x; data_table[current_cross][1]=y;}
		else{data_table[current_cross][0]=(data_table[current_cross][0]+x)/2; data_table[current_cross][1]=(data_table[current_cross][1]+y)/2;}
		
		data_table[current_cross][irany+2]=last_back_decision;
		if(irany==0) // ha normál irányból jövünk és van döntési leheoségünk
		{
			//az olyan keresztezodések amiken az elozo dontesi ota keresztüljöttünk azoknak is a következo döntésije ez
			uint8_t decision_ind=0;
			while(decision_list[decision_ind][0]!=-1 && decision_ind<MAX_NOT_DECISION_CROSS)
			{
				data_table[decision_list[decision_ind][0]][decision_list[decision_ind][1]+2]=current_cross;
				decision_ind++;
			}
			// majd -1-et  írunk a tömbelemek helyére
			for(decision_ind=0;decision_ind<MAX_NOT_DECISION_CROSS;decision_ind++)
			{
				decision_list[decision_ind][0]=-1;decision_list[decision_ind][1]=-1;
			}
			// de az elso helyre egybol visszatöltjük a mostani keresztezodést
			decision_list[0][0]=current_cross;	
			decision_list[0][1]=next_direction(irany);
		}
		else
		{
			// ha nem tudunk dönteni akkor hozzáfuzzük az elobb említett tömbhöz
			uint8_t decision_ind=0;
			while(decision_list[decision_ind][0]!=-1 && decision_ind<MAX_NOT_DECISION_CROSS) decision_ind++;
			decision_list[decision_ind][0]=current_cross;decision_list[decision_ind][1]=0;
			last_back_decision=current_cross;		
		}
		if(atsorolas!=-1)
		{
			if(atsorolas==2) // ahova erkezunk onnan kell jöjjünk
			{
				if(irany!=0)
				{
					data_table[current_cross][11]=irany;
				}
				else
				{
					data_table[current_cross][11]=99;
				}
				
			}
			else // ahonnan jöttünk oda kell menjunk
			{
				if(last_cross[0]!=-1)
				{
					if(last_back_decision!=last_cross[0]) // ha az elozo keresztezodesbe kell eljussunk
					{
						data_table[last_cross[0]][11]=last_cross[1];
					}
					else //ha nem az elozobe, akkor majd a végén kitaláljuk
					{
						data_table[last_cross[0]][11]=99;
					}
				}
				else
				{
					if(irany==0) data_table[current_cross][11]=100;
					else if(irany==1) data_table[current_cross][11]=101;
					else data_table[current_cross][11]=102;
				}
				
			}
		}
		
		// kitöltjük a NEM döntési részeket
		if(last_cross[0] !=-1) // csak azért kell h a legelso ne fusson hibára
		{
			data_table[last_cross[0]][last_cross[1]+5]=current_cross;
			data_table[current_cross][irany+5]=last_cross[0];	
			data_table[last_cross[0]][last_cross[1]+8]=distance;
			data_table[current_cross][irany+8]=distance;	
		}	
		last_cross[0]=current_cross;
		last_cross[1]=next_direction(irany);

		ismeretlen_utak();
		if(ismeretlen_utak_szama==0)
		{
			path_100();
			cross_99=path_99();
			if(cross_99 !=-1)
			{
				make_cost_table_99(data_table[cross_99][2],cross_99);
				uint32_t min_dist=99999,min_ind;
				for(uint8_t i_99=0;i_99<8;i_99++)
				{
					if(i_99!=data_table[cross_99][2])
					{
						dist=dijkstra_99( G_99, 8, i_99, data_table[cross_99][2]);
						dist_array[dist_ind]=dist;
						dist_ind++;
						if(min_dist>dist){min_dist=dist;min_ind=i_99;}						
					}
				}
				print_dist();
				
				//make_cost_table();					
				//data_table[cross_99][11]=-1;				
				data_table[min_ind][11]=dijkstra(G_99,8, min_ind,data_table[cross_99][2]);	
			}
			int8_t csp=atsorolashoz();
			if(csp==current_cross && irany==0) next_dir=data_table[csp][11];
			else
			{
				if(irany!=0)
				{
					next_dir=0;
				}
				else
				{
					make_cost_table();
					next_dir=dijkstra(G,8, current_cross,csp);
				}

			}

		}
		else
		{
			next_dir=next_direction(irany);
		}
		return next_dir;
	}
	else
	{
		konwncross--;
		if(irany!=0) return 0;
		else return 2;
	}
	

}

uint8_t follow_the_flow(void)
{
	// ha van olyan aminek tudjuk h mi  szomszédja és ennek a szomszédnak tudjuk, hogy mi a következo döntésije
	for(uint8_t i=0;i<NODES_NUM;i++)
	{
		for(uint8_t j=0;j<3;j++)
		{
			if(data_table[i][j+2]==-1 && data_table[i][j+5]!=-1 && data_table[data_table[i][j+5]][2] != -1)
			{
				data_table[i][j+2]=data_table[data_table[i][j+5]][2];
				return 1;
			}				
		}
	}
	return 0;
}

uint8_t next_direction(uint8_t irany)
{
	uint8_t next_dir;
	if(irany!=0)next_dir=0;
	else if(data_table[current_cross][6]==-1) next_dir=1;
	else if(data_table[current_cross][7]==-1) next_dir=2;
	else 
	{
		make_cost_table();
		next_dir=dijkstra(G,8, current_cross,-1);
	}	
	return next_dir;
}


void make_cost_table(void)
{
	// az útkereso algoritmusnak kell egy költségfv, ennek az eloállítása
	for(uint8_t i=0;i<NODES_NUM;i++)
	{
		//jobbra megyunk
		uint32_t distance=data_table[i][9];
		int8_t next_cross=data_table[i][6];
		uint8_t cost_table_count=0;
		if(data_table[i][3]!=-1)
		{
			while(data_table[i][3]!=next_cross && cost_table_count<7)
			{
				distance+=data_table[next_cross][8];next_cross=data_table[next_cross][5];cost_table_count++;
			}
			if(G[i][data_table[i][3]]==-1 || G[i][data_table[i][3]]>distance) G[i][data_table[i][3]]=distance;		
		}
		//balra megyunk
		distance=data_table[i][10];
		next_cross=data_table[i][7];
		cost_table_count=0;
		if(data_table[i][4]!=-1)
		{
			while(data_table[i][4]!=next_cross && cost_table_count<7)
			{
				distance+=data_table[next_cross][8];next_cross=data_table[next_cross][5];cost_table_count++;
			}
			if(G[i][data_table[i][4]]==-1 || G[i][data_table[i][4]]>distance) G[i][data_table[i][4]]=distance;
		}		
	}
}
uint8_t dijkstra(int G[MAX][MAX],int n,int startnode, int atsorolas)
{
	//algoritmus az útvonalkereséshez
	
	uint32_t cost[MAX][MAX],distance[MAX],pred[MAX];
	uint32_t visited[MAX],count,mindistance,nextnode,i,j,last_node;
	
	for(i=0;i<n;i++)
		for(j=0;j<n;j++)
			if(G[i][j]==-1)
				cost[i][j]=9999999;
			else
				cost[i][j]=G[i][j];
	
	for(i=0;i<n;i++)
	{
		distance[i]=cost[startnode][i];
		pred[i]=startnode;
		visited[i]=0;
	}
	
	distance[startnode]=0;
	visited[startnode]=1;
	count=1;
	
	while(count<n-1)
	{
		mindistance=9999999;
		for(i=0;i<n;i++)
			if(distance[i]<mindistance&&!visited[i])
			{
				mindistance=distance[i];
				nextnode=i;
			}		
			visited[nextnode]=1;
			for(i=0;i<n;i++)
				if(!visited[i])
					if(mindistance+cost[nextnode][i]<distance[i])
					{
						distance[i]=mindistance+cost[nextnode][i];
						pred[i]=nextnode;
					}
		count++;
	}
	
	//msg_index=0;
	uint32_t min_dist=100000,direct,directout,index=0;
	for(i=0;i<n;i++)
	{
		if((index!=startnode && (data_table[index][6]==-1 || data_table[index][7]==-1 ) && atsorolas==-1)|| atsorolas==index)
		{			
			j=index;
			do
			{
				last_node=j;
				j=pred[j];
			}while(j!=startnode);
			if(distance[index]!=9999999)
			{
				if(distance[index]<min_dist)
				{
					min_dist=distance[index];
					direct=last_node;
				}
								
			}
		}	
		index++;
	}
	if(data_table[startnode][3]==direct){directout=1;}
	else{directout=2;}	
	return directout;
}
int8_t atsorolashoz(void)
{
	for(uint8_t i=0;i<8;i++)
	{
		if(data_table[i][11]==1 || data_table[i][11]==2) return i;
	}
	return -1;
}

uint8_t ismeretlen_utak(void)
{
	ismeretlen_utak_szama=0;
	for(uint8_t i=0;i<8;i++)
	{
		for(uint8_t j=0;j<3;j++)
		{
			if(data_table[i][j+5]==-1)  ismeretlen_utak_szama++;
		}
	}
	ismeretlen_utak_szama=ismeretlen_utak_szama/2;
	return (ismeretlen_utak_szama);
}
void print_table(void)
{	
	
	msg_index2=0;
	//xSemaphoreTake(xMutexUART, portMAX_DELAY);
	for(uint8_t i=0;i<NODES_NUM;i++)
	{
		for(uint8_t j=0;j<NODES_DATA_NUM;j++)
		{			
				UART2_message(debug_data, sprintf(debug_data, "T %d %d %d \n",i,j, data_table[i][j]));			
		}		
	}
	//UART2_message2("\r",1);
	//HAL_UART_Transmit_DMA(&huart2, (uint8_t*)debug_message2,  msg_index2);
	//HAL_UART_Transmit(&huart2, (uint8_t*)debug_message2,  msg_index2,portMAX_DELAY);	
	//xSemaphoreGive(xMutexUART);
}
void print_dist(void)
{	
	
	msg_index2=0;
	//xSemaphoreTake(xMutexUART, portMAX_DELAY);
	for(uint8_t i=0;i<NODES_NUM;i++)
	{
		for(uint8_t j=0;j<NODES_DATA_NUM;j++)
		{			
				UART2_message(debug_data, sprintf(debug_data2, "D %d %d \n",i, dist_array[i]));			
		}		
	}
	//UART2_message2("\r",1);
	//HAL_UART_Transmit_DMA(&huart2, (uint8_t*)debug_message2,  msg_index2);
	//HAL_UART_Transmit(&huart2, (uint8_t*)debug_message2,  msg_index2,portMAX_DELAY);	
	//xSemaphoreGive(xMutexUART);
}

void UART2_message2(char *debug_data,int32_t size)
{
	for(int16_t i = 0;i<size;i++)
	{
		debug_message2[msg_index2] = debug_data[i];
		msg_index2++;
	}
}
int8_t path_99(void)
{
	int8_t cross=-1;
	for(uint8_t i=0;i<8;i++)
	{
		if(data_table[i][11]==99)
		{
			cross=i;
			break;
		}
	}
	return cross;
}
void path_100(void)
{
	for(uint8_t i=0;i<8;i++)
	{
		if(data_table[i][11]>99)
		{
			uint8_t cross_99_ind=data_table[i][data_table[i][11]-100+5];
			if(data_table[cross_99_ind][5]==i)
			{
				data_table[cross_99_ind][11]=99;
			}
			else if(data_table[cross_99_ind][6]==i)
			{
				data_table[cross_99_ind][11]=1;
			}
			else
			{
				data_table[cross_99_ind][11]=2;
			}
			data_table[i][11]=-1;
			break;
		}
	}
	
}
void make_cost_table_99(uint8_t goal, uint8_t from)
{

	// az útkereso algoritmusnak kell egy költségfv, ennek az eloállítása
	for(uint8_t i=0;i<NODES_NUM;i++)
	{
		//jobbra megyunk
		uint32_t distance=data_table[i][9];
		int8_t next_cross=data_table[i][6];
		uint8_t cost_table_count=0;
		if(data_table[i][3]!=-1)
		{
			while(data_table[i][3]!=next_cross && cost_table_count<7 )
			{
				distance+=data_table[next_cross][8];next_cross=data_table[next_cross][5];cost_table_count++;
			}
			if(G_99[i][data_table[i][3]]==-1 || G_99[i][data_table[i][3]]>distance) G_99[i][data_table[i][3]]=distance;		
		}
		//balra megyunk
		distance=data_table[i][10];
		next_cross=data_table[i][7];
		cost_table_count=0;
		if(data_table[i][4]!=-1)
		{
			while(data_table[i][4]!=next_cross && cost_table_count<7)
			{
				distance+=data_table[next_cross][8];next_cross=data_table[next_cross][5];cost_table_count++;
			}
			if(G_99[i][data_table[i][4]]==-1 || G_99[i][data_table[i][4]]>distance) G_99[i][data_table[i][4]]=distance;
		}		
	}
	for(uint8_t i=0;i<NODES_NUM;i++)
	{
		if(i!=from && G_99[i][goal] < G_99[from][goal]) //asdasd változott
		{
			G_99[i][goal]=-1;
		}
	}
}
uint32_t dijkstra_99(int G[MAX][MAX],int n,int startnode, int atsorolas)
{
	//algoritmus az útvonalkereséshez
	
	uint32_t cost[MAX][MAX],distance[MAX],pred[MAX];
	uint32_t visited[MAX],count,mindistance,nextnode,i,j;
	
	for(i=0;i<n;i++)
		for(j=0;j<n;j++)
			if(G[i][j]==-1)
				cost[i][j]=999999;
			else
				cost[i][j]=G[i][j];
	
	for(i=0;i<n;i++)
	{
		distance[i]=cost[startnode][i];
		pred[i]=startnode;
		visited[i]=0;
	}
	
	distance[startnode]=0;
	visited[startnode]=1;
	count=1;
	
	while(count<n-1)
	{
		mindistance=999999;
		for(i=0;i<n;i++)
			if(distance[i]<mindistance&&!visited[i])
			{
				mindistance=distance[i];
				nextnode=i;
			}		
			visited[nextnode]=1;
			for(i=0;i<n;i++)
				if(!visited[i])
					if(mindistance+cost[nextnode][i]<distance[i])
					{
						distance[i]=mindistance+cost[nextnode][i];
						pred[i]=nextnode;
					}
		count++;
	}	
	return distance[atsorolas];
}
void corr_x_y(uint8_t curr_cross, int32_t x,int32_t y)
{
	int32_t x_err=data_table[curr_cross][0]-x;
	int32_t y_err=data_table[curr_cross][1]-y;
	for(uint8_t i=0;i<NODES_NUM;i++)
	{
		if(data_table[i][0]!=-1 && data_table[curr_cross][12]>data_table[i][12])
		{
			data_table[i][0]=data_table[i][0]-x_err;
			data_table[i][1]=data_table[i][1]-y_err;
		}
		
	}
	
}

