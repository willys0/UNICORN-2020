//
// Created by Lana on 04/2017.
//
#pragma once
#include "MinSearchHeap.h"
#define CONNECT4 5  //4+1 (END FLAG)
#define BUFFER_SIZE 16000000 //For a map of size 4000x4000
# define INF 0x3f3f3f3f  
using namespace std;

class ThetaStar4Grid {
//Use to store information of grid
int *index_;
int *mapping_;
float *vertice_w_;
int *edge_;
float *edge_w_;
int *close_heap;
unsigned char* cost_map_;
MinSearchHeap *open_heap;
//Configuration of grid
int ns; //number of vertices of the grid
int map_height,map_width; //height and width of the cost map
int grid_width, grid_height; //height and width of the grid
int res;
public:
	float *g_score_;
	int *parent_list_;

	ThetaStar4Grid();
	~ThetaStar4Grid();
	void SetGridStep(int);
	int GetNodeIndex(int,int );
	void GetNodeCoordinate(int,int &,int &);
	void IntializeMap(unsigned char*,int,int);
    bool MakeGrid();
	bool LoS_Check(int,int);
	void Theta(int,int);
	void LazyTheta(int,int);
	float Cost(int, int);
};

