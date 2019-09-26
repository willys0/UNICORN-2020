//
// Created by Lana on 04/2017.
//
#include "thetastar_planner/ThetaStar4Grid.h"
#include <queue>
#include <functional>
#include <time.h>
#include <math.h>
#include <ros/ros.h>
using namespace std;
ThetaStar4Grid::ThetaStar4Grid() { 
    ns=0;
	res = 1;
	open_heap = new MinSearchHeap(BUFFER_SIZE);
	index_=new int[BUFFER_SIZE];
	mapping_=new int[BUFFER_SIZE];
	vertice_w_=new float[BUFFER_SIZE];
	edge_=new int[BUFFER_SIZE*CONNECT4];
	edge_w_=new float[BUFFER_SIZE*CONNECT4];
	close_heap=new int[BUFFER_SIZE];
	parent_list_ = new int[BUFFER_SIZE];
	g_score_=new float[BUFFER_SIZE];
}

ThetaStar4Grid::~ThetaStar4Grid() {
	delete open_heap;
	delete index_;
	delete mapping_;
	delete vertice_w_;
	delete edge_;
	delete edge_w_;
	delete close_heap;
	delete parent_list_;
	delete g_score_;
}


void ThetaStar4Grid::SetGridStep(int s) {
	res = s;
}

int ThetaStar4Grid::GetNodeIndex(int x, int y) {
	x = x / res;
	y = y / res;
	return mapping_[y*(map_width/res) + x];
}

void ThetaStar4Grid::IntializeMap(unsigned char* cost_img, int h, int w) {
	
	//ROS_ERROR("\n Hello\n");
	cost_map_ = cost_img;
	map_width = w;
	map_height = h;
}


bool ThetaStar4Grid::MakeGrid(){
	int i, ix, x,y;
	grid_width = map_width / res;
	grid_height = map_height / res;
	int grid_size = grid_width*grid_height;


	if (grid_size >= BUFFER_SIZE) {
		ns = 0;
		return false;
	}

	for (i = 0; i < grid_size; i++) {
		x = res*(i%grid_width);
		y = res*(i/grid_width);
		unsigned char *ptr;
		ptr=cost_map_;
		if (!ptr[y*map_width + x]) {
			vertice_w_[i] = 1;
		}
		else {
			vertice_w_[i] = 0;
			mapping_[i] = 0;
		}
	}

	ns = 0;
	for (i = 0; i < grid_size; i++ ) {
		if(vertice_w_[i]){
				x = res*(i%grid_width);
				y = res*(i/grid_width);
				index_[ns] = i;
				mapping_[i] = ns;
				ns++;
		}
	}

	int *edge_ptr = edge_;
	float *weight_ptr = edge_w_;

		for (i = 0; i < ns; i++) {
				ix = index_[i] ;
				x = ix%grid_width;
				y = ix/ grid_width;
				int n = 0,ys= y *grid_width;
				if (mapping_[ys + grid_width + x ] && y<grid_height) {
					edge_ptr[n]= mapping_[ys + grid_width + x];
					weight_ptr[n] = 1;
					n++;
				}

				if (mapping_[ys - grid_width + x] && y>0) {
					edge_ptr[n] = mapping_[ys - grid_width + x];
					weight_ptr[n] = 1;
					n++;
				}

				if (mapping_[ys + x - 1] && x>0) {
					edge_ptr[n] = mapping_[ys + x - 1];
					weight_ptr[n] = 1;
					n++;
				}

				if (mapping_[ys + x + 1] && x<grid_width-1  ) {
					edge_ptr[n] = mapping_[ys + x + 1];
					weight_ptr[n] = 1;
					n++;
				}
			edge_ptr[n]=-1;
			edge_ptr += CONNECT4;
			weight_ptr+=CONNECT4;
			}
return true;
}


inline bool ThetaStar4Grid::LoS_Check(int start, int goal) {
	unsigned char *ptr = cost_map_;
	int width = map_width;
	int xs, xg, ys, yg;
	GetNodeCoordinate(start,xs,ys);
	GetNodeCoordinate(goal,xg,yg);
	int dx = xg - xs;  
	int dy = yg - ys;  
	int f = 0, x, y;
	ptr += width*ys + xs; 
	if (dx >= 0 && dy >= 0) {
		if (dx>dy) {
			for (x = xs; x <xg; x++, ptr++) {
				f += dy;
				if (f >= dx) {
					ptr += width;
					f -= dx;
				}
				if (*ptr) return false;
			}
		}
		else {
			for (y = ys; y <yg; y++, ptr += width) {
				f += dx;
				if (f >= dy) {
					ptr++;
					f -= dy;
				}
				if (*ptr) return false;
			}
		}
		return true;
	}

	if (dx >= 0 && dy < 0) {
		dy = -dy;
		if (dx>dy) {
			for (x = xs; x <xg; x++, ptr++) {
				f += dy;
				if (f >= dx) {
					ptr -= width;
					f -= dx;
				}
				if (*ptr) return false;
			}
		}
		else {
			for (y = ys; y > yg; y--, ptr -= width) {
				f += dx;
				if (f >= dy) {
					ptr++;
					f -= dy;
				}
				if (*ptr) return false; 
			}
		}
		return true;
	}

	if (dx < 0 && dy >= 0) {
		dx = -dx;
		if (dx>dy) {
			for (x = xs; x > xg; x--, ptr--) {
				f += dy;
				if (f >= dx) {
					ptr += width;
					f -= dx;
				}
				if (*ptr) return false;
			}
		}
		else {
			for (y = ys; y < yg; y++, ptr += width) {
				f += dx;
				if (f >= dy) {
					ptr--;
					f -= dy;
				}
				if (*ptr) return false;
			}
		}
		return true;
	}

	if (dx < 0 && dy < 0) {
		dx = -dx;
		dy = -dy;
		if (dx>dy) {
			for (x = xs; x >xg; x--, ptr--) {
				f += dy;
				if (f >= dx) {
					ptr -= width;
					f -= dx;
				}
				if (*ptr) return false;
			}
		}
		else {
			for (y = ys; y >yg; y--, ptr -= width) {
				f += dx;
				if (f >= dy) {
					ptr--;
					f -= dy;
				}
				if (*ptr) return false;
			}
		}
		return true;
	}

	return true;
}


inline float ThetaStar4Grid::Cost(int start, int goal) {
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

void ThetaStar4Grid::Theta (int start,int goal) {
	HeapElement tmp;
	int i, u,parent_list_of_s;
	float gold, w;
	open_heap->Clear();
	for (u = 0;u < ns;u++) {
		close_heap[u] = 1; 
		g_score_[u] = INF; 
	}

	g_score_[start] = 0;
	parent_list_[start] = start;
	tmp.key = 0;
	tmp.data = start;
	open_heap->Insert(tmp);
	while (!open_heap->IsEmpty()) {
		tmp = open_heap->SearchMin(); 
		int s = tmp.data;
		if (s == goal)
			break;

		if (!close_heap[s]) continue;
		close_heap[s] = 0; 
		parent_list_of_s = parent_list_[s];
		for (i = CONNECT4*s; edge_[i]>0;i++) {
			u = edge_[i];
			if (close_heap[u]) { 
				gold = g_score_[u];
				if (LoS_Check(parent_list_of_s, u)) {
					w = Cost(parent_list_of_s,u); 
					if (g_score_[u] > g_score_[parent_list_of_s] + w) {
							parent_list_[u] = parent_list_of_s;
							g_score_[u] = g_score_[parent_list_of_s] + w;
					}
				}
				else {
					w = edge_w_[i]; 
					if (g_score_[u] > g_score_[s] + w) {
						parent_list_[u] = s;
						g_score_[u] = g_score_[s] + w;
					}
				}

				if (g_score_[u] < gold) {
					tmp.key = g_score_[u];
					tmp.data = u;
					open_heap->Insert(tmp);
				}
			}
		}
	}
}

void ThetaStar4Grid::LazyTheta(int start,int goal) {
	HeapElement tmp;
	int i, u, parent_list_of_s;
	float gold, w,min_gs,umin;
	open_heap->Clear();
	for (u = 0;u < ns;u++) {
		close_heap[u] = 1; 
		g_score_[u] = INF; 
	}

	g_score_[start] = 0;
	parent_list_[start] = start;
	tmp.key = 0;
	tmp.data = start;
	open_heap->Insert(tmp);
	while (!open_heap->IsEmpty()) {
		tmp = open_heap->SearchMin(); 
		int s = tmp.data;
		
		if (!close_heap[s]) continue;
		close_heap[s] = 0; 
		parent_list_of_s = parent_list_[s];
		if (!LoS_Check(parent_list_of_s, s)) {
			min_gs = INF;
			for (i = CONNECT4*s; edge_[i] > 0;i++) {
				u = edge_[i];w = edge_w_[i];
				if ((!close_heap[u]) && (g_score_[u] + w < min_gs)) {
					min_gs = g_score_[u] + w;
					umin = u;
				}
			}
			parent_list_[s]=umin;
			g_score_[s] = min_gs;
		}

		if (s == goal)
			break; 

		for (i = CONNECT4*s; edge_[i]>0;i++) {
			u = edge_[i];
			if (close_heap[u]) { 
				gold = g_score_[u];
					w = Cost(parent_list_of_s, u); 
					if (g_score_[u] > g_score_[parent_list_of_s] + w) {
						parent_list_[u] = parent_list_of_s;
						g_score_[u] = g_score_[parent_list_of_s] + w;
					}
				if (g_score_[u] < gold) {
					tmp.key = g_score_[u];
					tmp.data = u;
					open_heap->Insert(tmp);
				}
			}
		}
	}
}


void ThetaStar4Grid::GetNodeCoordinate(int ix,int &x,int &y) {
int s = index_[ix];
x = (s % grid_width)*res;
y= (s / grid_width)*res;
}

