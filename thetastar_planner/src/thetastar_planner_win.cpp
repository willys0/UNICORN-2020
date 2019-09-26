#include "thetastar_planner/thetastar_planner.h"

namespace thetastar_planner {
	ThetaStarPlanner::ThetaStarPlanner()
		: initialized_(false), unknown_(true), lethal_cost_(253), neutral_cost_(50), convert_offset_(0.5) {}


	void ThetaStarPlanner::initialize(int nx_, int ny_, unsigned char *cost_map) {
		nx = nx_;
		ny = ny_;
		ns = nx*ny;
		cost_array_ = cost_map;
	}

	void ThetaStarPlanner::outlineMap(unsigned char* costarr, int nx, int ny, unsigned char value) {
		unsigned char* pc = costarr;
		for (int i = 0; i < nx; i++)
			*pc++ = value;
		pc = costarr + (ny - 1) * nx;
		for (int i = 0; i < nx; i++)
			*pc++ = value;
		pc = costarr;
		for (int i = 0; i < ny; i++, pc += nx)
			*pc = value;
		pc = costarr + nx - 1;
		for (int i = 0; i < ny; i++, pc += nx)
			*pc = value;
	}


	bool ThetaStarPlanner::calculatePotentials(unsigned char* costs, double start_x, double start_y, double end_x, double end_y,
		int cycles, float* potential) {
		ROS_ERROR("Wow, LA  calculate Potential \n");
		queue_.clear();
		int start_i = start_x + nx*start_y;
		queue_.push_back(Index(start_i, 0));
		ROS_ERROR("Wow, LA star searching a goal -step 1 \n");
		int ns_ = nx * ny;
		std::fill(potential, potential + ns_, POT_HIGH);
		ROS_ERROR("Wow, LA star searching a goal -step 2, %d \n", start_i);
		potential[start_i] = 0;

		int goal_i = end_x + nx*end_y;
		int cycle = 0;
		ROS_ERROR("Wow, LA star searching a goal %d \n", goal_i);
		while (queue_.size() > 0 && cycle < cycles) {
			Index top = queue_[0];
			std::pop_heap(queue_.begin(), queue_.end(), greater1());
			queue_.pop_back();

			int i = top.i;
			if (i == goal_i) {
				ROS_ERROR("Wow, LA found a goal \n");
				return true;
			}

			add(costs, potential, potential[i], i + 1, end_x, end_y);
			add(costs, potential, potential[i], i - 1, end_x, end_y);
			add(costs, potential, potential[i], i + nx, end_x, end_y);
			add(costs, potential, potential[i], i - nx, end_x, end_y);
			cycle++;
		}

		return false;
	}


	void ThetaStarPlanner::add(unsigned char* costs, float* potential, float prev_potential, int next_i, int end_x,
		int end_y) {
		if (next_i < 0 || next_i >= ns)
			return;

		if (potential[next_i] < POT_HIGH)
			return;

		// if(costs[next_i]>=lethal_cost_ && !(unknown_ && costs[next_i]==costmap_2d::NO_INFORMATION))
		if (costs[next_i] >= lethal_cost_ && !(unknown_ && costs[next_i] == 255))
			return;

		potential[next_i] = calculatePotential(potential, costs[next_i] + neutral_cost_, next_i, prev_potential);
		int x = next_i % nx, y = next_i / nx;
		float distance = abs(end_x - x) + abs(end_y - y);

		queue_.push_back(Index(next_i, potential[next_i] + distance * neutral_cost_));
		std::push_heap(queue_.begin(), queue_.end(), greater1());
	}

	bool ThetaStarPlanner::getPath(float* potential, double start_x, double start_y, double end_x, double end_y, std::vector<std::pair<float, float> >& path) {
		std::pair<float, float> current;
		current.first = end_x;
		current.second = end_y;
		int start_index = start_x + nx*start_y;
		path.push_back(current);
		int c = 0;
		int ns = nx * ny;

		while (current.first + nx*current.second != start_index) {
			float min_val = 1e10;
			int min_x = 0, min_y = 0;
			for (int xd = -1; xd <= 1; xd++) {
				for (int yd = -1; yd <= 1; yd++) {
					if (xd == 0 && yd == 0)
						continue;
					int x = current.first + xd, y = current.second + yd;
					int index = x + nx*y;
					if (potential[index] < min_val) {
						min_val = potential[index];
						min_x = x;
						min_y = y;
					}
				}
			}
			if (min_x == 0 && min_y == 0)
				return false;
			current.first = min_x;
			current.second = min_y;
			path.push_back(current);
			
			if (c++ > ns * 4) {
				return false;
			}

		}
		return true;
	}


	std::vector<std::pair<float, float>> ThetaStarPlanner::makePlan(int start_x, int start_y, int goal_x, int goal_y) {

		std::vector<std::pair<float, float>> path;
		ROS_ERROR("LA got a start: %d %d, and a goal: %d, %d in (%d %d)", start_x, start_y, goal_x, goal_y,nx,ny);
		ns = nx*ny;
		potential_array_ = new float[nx * ny];
		outlineMap(cost_array_, nx, ny, 254);
		bool found_legal = calculatePotentials(cost_array_, start_x, start_y, goal_x, goal_y,
			nx * ny * 2, potential_array_);

		getPath(potential_array_, start_x, start_y, goal_x, goal_y, path);

		delete potential_array_;
		return path;
	};

	int step = MAX_DIST;
	inline void ThetaStarPlanner::FillLinePotential(int xs,int ys,int xg,int yg,float *ptr) {
		int width = nx;
		int dx = xg - xs;   // the delta of x
		int dy = yg - ys;   // the delta of y 
		int f = 0, x ,y;
		float *ptr_end= ptr+width*yg + xg;
		ptr += width*ys + xs; //shift the pointer to the coordinate of the pixel
		if (dx >= 0 && dy >= 0) {
			if (dx>dy) {
				for (x = xs; x <xg; x++, ptr++) {
					f += dy;
					if (f >= dx) {
						ptr += width;
						f -= dx;
					}
					*ptr = step--;
				}
			}
			else {
				for (y = ys; y <yg; y++, ptr += width) {
					f += dx;
					if (f >= dy) {
						ptr++;
						f -= dy;
					}
					*ptr = step--;
				}
			}
			*ptr_end = step--;
			return;
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
					*ptr = step--;
				}
			}
			else {
				for (y = ys; y > yg; y--, ptr -= width) {
					f += dx;
					if (f >= dy) {
						ptr++;
						f -= dy;
					}
					*ptr = step--;
				}
			}
			return;
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
					*ptr = step--;
				}
			}
			else {
				for (y = ys; y < yg; y++, ptr += width) {
					f += dx;
					if (f >= dy) {
						ptr--;
						f -= dy;
					}
					*ptr = step--;
				}
			}
			return;
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
					*ptr = step--;
				}
			}
			else {
				for (y = ys; y >yg; y--, ptr -= width) {
					f += dx;
					if (f >= dy) {
						ptr--;
						f -= dy;
					}
					*ptr = step--;
				}
			}
			return;
		}

		return;
	}


	std::vector<std::pair<float, float>> ThetaStarPlanner::makeThetaStarPlan(int start_x, int start_y, int goal_x, int goal_y) {

		grid_graph_.SetGridStep(GRID_SIZE);
		grid_graph_.IntializeMap(cost_array_, nx, ny);
		if (!grid_graph_.MakeGrid())
			printf("\n Cannot create the graph\n");
		int src, dst;
		src = grid_graph_.GetNodeIndex(start_x, start_y);
		dst = grid_graph_.GetNodeIndex(goal_x, goal_y);
		grid_graph_.Theta(src, dst);
		ns = nx*ny;
		
		vector<std::pair<int, int>> shortest_path;

		int x, y, u = dst;
		std::pair<int, int> current;
		current.first = goal_x;current.second = goal_y;
		shortest_path.push_back(current);
		grid_graph_.GetNodeCoordinate(u, x, y);
		current.first = x;current.second = y;
		shortest_path.push_back(current);

		while (u != src && grid_graph_.g_score_[u] != INF) {
			grid_graph_.GetNodeCoordinate(grid_graph_.parent_list_[u], x, y);
			current.first = x;current.second = y;
			shortest_path.push_back(current);
			u = grid_graph_.parent_list_[u];
		}
		current.first = start_x;current.second = start_y;
		shortest_path.push_back(current);

		potential_array_ = new float[ns];
		for (int i = 0;i < ns;i++) potential_array_[i] = POT_HIGH;
		int i;
		for (i = 0;i < shortest_path.size() - 1;i++) 
			FillLinePotential(shortest_path[i].first, shortest_path[i].second, shortest_path[i + 1].first, shortest_path[i + 1].second, potential_array_);

		
		std::vector<std::pair<float, float>> path;
		path.clear();
		getPath(potential_array_, start_x,start_y, goal_x, goal_y, path);
		delete potential_array_;
		return path;
	}
};