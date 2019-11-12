//
// Created by Lana on 04/2017.
//
#include "thetastar_planner/ThetaStar4Grid.h"
#include <queue>
#include <functional>
#include <time.h>
#include <math.h>
#include <ros/ros.h>

ThetaStar4Grid::ThetaStar4Grid()
{ 
    ns = 0;
	res = 1;
	open_heap = new MinSearchHeap(BUFFER_SIZE);
	index_= new int[BUFFER_SIZE];
	mapping_= new int[BUFFER_SIZE];
	edge_ = new int[BUFFER_SIZE*CONNECT4];
	edge_w_ = new float[BUFFER_SIZE*CONNECT4];
	close_heap = new int[BUFFER_SIZE];
	parent_list_ = new int[BUFFER_SIZE];
	real_parent_list_ = new int[BUFFER_SIZE];
	g_score_ = new float[BUFFER_SIZE];
}

ThetaStar4Grid::~ThetaStar4Grid()
{
	delete open_heap;
	delete index_;
	delete mapping_;
	delete edge_;
	delete edge_w_;
	delete close_heap;
	delete parent_list_;
	delete g_score_;
}

void ThetaStar4Grid::SetGridStep(int s)
{
	res = s;
}

int ThetaStar4Grid::GetNodeIndex(int x, int y)
{
	x = x / res;
	y = y / res;
	return mapping_[y*(map_width/res) + x];
}

void ThetaStar4Grid::IntializeMap(unsigned char* cost_img, int h, int w)
{
	cost_map_ = cost_img;
	map_width = w;
	map_height = h;
}


bool ThetaStar4Grid::MakeGrid()
{
	float *weight_ptr = edge_w_;
	int i, ix, x, y;
	int n = 0;
	int ys = 0;
	grid_width = map_width / res;
	grid_height = map_height / res;
	int grid_size = grid_width * grid_height;
	int *edge_ptr = edge_;
	
	ns = 0;

	if (grid_size >= BUFFER_SIZE) {
		return false;
	}

	for (i = 0; i < grid_size; i++) {
		x = res * (i % grid_width);
		y = res * (i / grid_width);

		// Check if an obstacle is in the current costmap index (This value is gradient)
		// Check if the space is either free or unknown
		// 253 == LETHAL, 255 == UNKNOWN
		if (cost_map_[y * map_width + x] <= 128 || cost_map_[y * map_width + x] == 255) { // Higher value == pathings is able to get closer to obstacles
			index_[ns] = i;
			mapping_[i] = ns;
			ns++;
		}

		else {
			mapping_[i] = 0;
		}
	}

	for (i = 0; i < ns; i++) {
			ix = index_[i] ;
			x = ix % grid_width;
			y = ix / grid_width;

			ys = y * grid_width;

			n = 0;

			if (mapping_[ys + grid_width + x] && y < grid_height) {
				edge_ptr[n] = mapping_[ys + grid_width + x];
				weight_ptr[n] = 1;
				n++;
			}

			if (mapping_[ys - grid_width + x] && y > 0) {
				edge_ptr[n] = mapping_[ys - grid_width + x];
				weight_ptr[n] = 1;
				n++;
			}

			if (mapping_[ys + x - 1] && x > 0) {
				edge_ptr[n] = mapping_[ys + x - 1];
				weight_ptr[n] = 1;
				n++;
			}

			if (mapping_[ys + x + 1] && x < grid_width - 1) {
				edge_ptr[n] = mapping_[ys + x + 1];
				weight_ptr[n] = 1;
				n++;
			}

			edge_ptr[n] =- 1;
			edge_ptr += CONNECT4;
			weight_ptr += CONNECT4;
		}

	return true;
}

inline bool ThetaStar4Grid::LoS_Calc(int xs, int xg, int ys, int yg, int dy, int dx, int width, int threshold, unsigned char *ptr, int ptr_sign)
{
	int f = 0;
	if (dx > dy) {
		for (int x = xs; x <xg; x++, ptr += ptr_sign) {
			f += dy;
			if (f >= dx) {
				ptr += width;
				f -= dx;
			}
			if (*ptr > threshold && *ptr != 255) return false; // No line of sight
		}
	}
	else {
		for (int y = ys; y <yg; y++, ptr += width) {
			f += dx;
			if (f >= dy) {
				ptr += ptr_sign;
				f -= dy;
			}
			if (*ptr > threshold && *ptr != 255) return false; // No line of sight
		}
	}

	return true; // Line of sight
}

inline bool ThetaStar4Grid::LoS_Check(int start, int goal)
{

	int threshold = 160; // LoS threshold, that is, how close to an object you have LoS (Lower == farther)
	unsigned char *ptr = cost_map_;
	int xs, xg, ys, yg;

	GetNodeCoordinate(start,xs,ys);
	GetNodeCoordinate(goal,xg,yg);

	int dx = xg - xs;  
	int dy = yg - ys;  

	ptr += map_width * ys + xs; 

	if (dx >= 0 && dy >= 0) {
		return LoS_Calc(xs, xg, ys, yg, dy, dx, map_width, threshold, ptr, 1);
	}

	if (dx >= 0 && dy < 0) {
		return LoS_Calc(xs, xg, -ys, -yg, -dy, dx, -map_width, threshold, ptr, 1);
	}

	if (dx < 0 && dy >= 0) {
		return LoS_Calc(-xs, -xg, ys, yg, dy, -dx, map_width, threshold, ptr, -1);
	}

	if (dx < 0 && dy < 0) {
		return LoS_Calc(-xs, -xg, -ys, -yg, -dy, -dx, -map_width, threshold, ptr, -1);
	}

	return true;
}

/*
inline bool ThetaStar4Grid::LoS_Check_F(int start, int goal) {

	int threshold = 160; // 128		// LoS threshold, that is, how close to an object you have LoS (Lower == farther)
	unsigned char *ptr = cost_map_;
	int xs, xg, ys, yg;

	GetNodeCoordinate(start,xs,ys);
	GetNodeCoordinate(goal,xg,yg);

	int dx = xg - xs;  
	int dy = yg - ys;  
	int f = 0;
	int ptr_sign = 0;
	int w = 0;
	int ptr_add, ptr_it;
	int is, ig, f_add, f_comp;

	ptr += map_width * ys + xs; 

	ptr_sign = 1;
	w = map_width;

	if (dx < 0) {
		ptr_sign = -1;
		xs = -xs;
		xg = -xg;
		dx = -dx;
	}

	if (dy < 0) {
		w = -map_width;
		ys = -ys;
		yg = -yg;
		dy = -dy;
	}

	if (dx > dy) {
		ptr_it = ptr_sign;
		ptr_add = w;
		is = xs;
		ig = xg;
		f_add = dy;
		f_comp = dx;
	}
	else {
		ptr_it = w;
		ptr_add = ptr_sign;
		is = ys;
		ig = yg;
		f_add = dx;
		f_comp = dy;
	}

	for (int i = is; i <ig; i++, ptr += ptr_it) {
		f += f_add;
		if (f >= f_comp) {
			ptr += ptr_add;
			f -= f_comp;
		}

		if (*ptr > threshold) return false; //if the cost is equal to threshold == no line of sight
	}

	return true;
}
*/

inline float ThetaStar4Grid::Cost(int start, int goal)
{
	int xs, ys, xg, yg;
	int s = index_[start];
	xs = (s % grid_width)*res;
	ys = (s / grid_width)*res;
	s = index_[goal];
	xg = (s % grid_width)*res;
	yg = (s / grid_width)*res;
	int x = xs - xg;
	int y = ys - yg;
	return sqrt(x*x + y*y);
} 

bool ThetaStar4Grid::Theta(int start,int goal)
{
	HeapElement tmp;
	float gold, w;
	int i, u,parent_list_of_s, tries = 0;
	
	open_heap->Clear();

	for (u = 0; u < ns; u++) {
		close_heap[u] = 1; 
		g_score_[u] = INF; 
	}

	g_score_[start] = 0;
	parent_list_[start] = start;
	tmp.key = 0;
	tmp.data = start;
	open_heap->Insert(tmp);

	while (!open_heap->IsEmpty()) {

		if(tries++ == 100000) {
			std::cout << "No path" << std::endl;
			return false;
		}

		tmp = open_heap->SearchMin(); 
		int s = tmp.data;

		if (s == goal) {
			std::cout << "Path found" << std::endl;
			return true;
		}
			

		if (!close_heap[s]) continue;

		close_heap[s] = 0; 
		parent_list_of_s = parent_list_[s];

		for (i = CONNECT4*s; edge_[i] > 0; i++) {
			u = edge_[i];

			if (close_heap[u]) { 
				gold = g_score_[u];

				// Save the real parent
				real_parent_list_[u] = s;

				// Has LoS
				if (LoS_Check(parent_list_of_s, u)) {
					w = Cost(parent_list_of_s,u); 
					if (g_score_[u] > g_score_[parent_list_of_s] + w) {
							parent_list_[u] = parent_list_of_s;
							g_score_[u] = g_score_[parent_list_of_s] + w;
					}
				}

				// No LoS
				else {
					w = edge_w_[i]; 
					if (g_score_[u] > g_score_[s] + w) {
						parent_list_[u] = s;
						g_score_[u] = g_score_[s] + w;
					}
				}

				// Add node
				if (g_score_[u] < gold) {
					tmp.key = g_score_[u];
					tmp.data = u;
					open_heap->Insert(tmp);
				}
			}
		}
	}

	std::cout << "No path" << std::endl;
	return false;
}

void ThetaStar4Grid::GetNodeCoordinate(int ix,int &x,int &y)
{
	int s = index_[ix];
	x = (s % grid_width) * res;
	y = (s / grid_width) * res;
}

float ThetaStar4Grid::GetGCost(int index)
{
	return g_score_[index];
}
int ThetaStar4Grid::getParent(int index)
{
	return parent_list_[index];
}
