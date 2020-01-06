//
// Created by Lana on 04/2017.
//
#pragma once
#include "thetastar_planner/MinSearchHeap.h"
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#define CONNECT4 5  //4+1 (END FLAG)
#define BUFFER_SIZE 16000000 //For a map of size 4000x4000
#define INF 0x3f3f3f3f  

class ThetaStar4Grid {
//Use to store information of grid
	private:
		MinSearchHeap *open_heap = NULL;
		float *edge_w_ = NULL;
		float *g_score_ = NULL;
		int *parent_list_ = NULL;
		int *real_parent_list_ = NULL;
		int *edge_ = NULL;
		int *index_ = NULL;
		int *mapping_ = NULL;
		int *close_heap = NULL;

		//Configuration of grid
		int ns; 					//number of vertices of the grid
		int map_height, map_width; 	//height and width of the cost map
		int grid_width, grid_height;//height and width of the grid
		int res;

		unsigned char* cost_map_ = NULL;

	public:
		ThetaStar4Grid();
		~ThetaStar4Grid();

		void IntializeMap(unsigned char* cost_img, int h, int w);
		void SetGridStep(int s);
		void GetNodeCoordinate(int ix,int &x,int &y);
		bool Theta(int start,int goal);
		bool LazyTheta(int start,int goal);

		int GetNodeIndex(int x, int y);
		int getParent(int index);
		int getRealParent(int index);

		bool MakeGrid();
		bool LoS_Calc(int xs, int xg, int ys, int yg, int dy, int dx, int width, int threshold, unsigned char *ptr, int ptr_sign);
		bool LoS_Check(int start, int goal);
		
		float Cost(int start, int goal);
		float GetGCost(int index);
		
};

