#include <thetastar_planner/ThetaStar4Grid.h>
#include <thetastar_planner/thetastar_planner.h>
#include <pluginlib/class_list_macros.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(thetastar_planner::ThetaStarPlanner, nav_core::BaseGlobalPlanner)

namespace thetastar_planner {

  ThetaStarPlanner::ThetaStarPlanner()
  : costmap_ros_(NULL), initialized_(false),unknown_(true), lethal_cost_(253), neutral_cost_(50),convert_offset_(0.5){ try_=0;}

  ThetaStarPlanner::ThetaStarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  : costmap_ros_(NULL), initialized_(false),unknown_(true), lethal_cost_(253), neutral_cost_(50),convert_offset_(0.5){try_=0;
    initialize(name, costmap_ros);
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


inline void ThetaStarPlanner::FillLinePotential(int xs,int ys,int xe,int ye,float *ptr) {
		int width = nx;
        //ROS_ERROR("\n The width of the map %d\n",width);
		int dx = xe - xs;   
		int dy = ye - ys;    
		int f = 0, x ,y;
		float *ptr_end= ptr+width*ye + xe;
		ptr += width*ys + xs; //shift the pointer to the coordinate of the pixel
		if (dx >= 0 && dy >= 0) {
			if (dx>dy) {
				for (x = xs; x <xe; x++, ptr++) {
					f += dy;
					if (f >= dx) {
						ptr += width;
						f -= dx;
					}
					*ptr = step--;
				}
			}
			else {
				for (y = ys; y <ye; y++, ptr += width) {
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
				for (x = xs; x <xe; x++, ptr++) {
					f += dy;
					if (f >= dx) {
						ptr -= width;
						f -= dx;
					}
					*ptr = step--;
				}
			}
			else {
				for (y = ys; y > ye; y--, ptr -= width) {
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

		if (dx < 0 && dy >= 0) {
			dx = -dx;
			if (dx>dy) {
				for (x = xs; x > xe; x--, ptr--) {
					f += dy;
					if (f >= dx) {
						ptr += width;
						f -= dx;
					}
					*ptr = step--;
				}
			}
			else {
				for (y = ys; y < ye; y++, ptr += width) {
					f += dx;
					if (f >= dy) {
						ptr--;
						f -= dy;
					}
					*ptr = step--;
				}
			}
			*ptr_end = step--;
			return;
		}

		if (dx < 0 && dy < 0) {
			dx = -dx;
			dy = -dy;
			if (dx>dy) {
				for (x = xs; x >xe; x--, ptr--) {
					f += dy;
					if (f >= dx) {
						ptr -= width;
						f -= dx;
					}
					*ptr = step--;
				}
			}
			else {
				for (y = ys; y >ye; y--, ptr -= width) {
					f += dx;
					if (f >= dy) {
						ptr--;
						f -= dy;
					}
					*ptr = step--;
				}
			}
			*ptr_end = step--;
			return;
		}

		return;
	}



bool ThetaStarPlanner::ThetaPotentials(unsigned char* costs, double start_x, double start_y, double end_x, double end_y,
                                        float* potential) {
        //ROS_ERROR("\n Inside Theta Potentials %d %d\n",nx,ny);
		grid_graph_.SetGridStep(GRID_SIZE);
		grid_graph_.IntializeMap(costs, nx, ny);
        if (!grid_graph_.MakeGrid()) {
            ROS_ERROR("\n Cannot create the graph\n");
        }

		int src, dst;
		src = grid_graph_.GetNodeIndex(start_x, start_y);
		dst = grid_graph_.GetNodeIndex(end_x, end_y);
        //ROS_ERROR("\n I am here to start running Theta Star\n");
		grid_graph_.Theta(src, dst);
        //ROS_ERROR("\n I am here\n");

		ns = nx*ny;
		
		vector<std::pair<int, int> > shortest_path;

		int x, y, u = dst;
		std::pair<int, int> current;

		current.first = end_x;current.second = end_y;
                shortest_path.push_back(current);

		grid_graph_.GetNodeCoordinate(u, x, y);
		/*current.first = x;current.second = y;
		shortest_path.push_back(current);*/

		while (u != src && grid_graph_.g_score_[u] != INF) {
			grid_graph_.GetNodeCoordinate(grid_graph_.parent_list_[u], x, y);
			current.first = x;current.second = y;
			shortest_path.push_back(current);
			u = grid_graph_.parent_list_[u];
		}

		current.first = start_x;current.second = start_y;
                shortest_path.push_back(current);

		for (int i = 0;i < ns;i++) potential[i] = POT_HIGH;
		
		reset_step();
		for (int i = 0;i < shortest_path.size() - 1;i++) {
			//ROS_ERROR("Line from %d %d to %d %d",shortest_path[i].first, shortest_path[i].second, shortest_path[i + 1].first, shortest_path[i + 1].second);
			FillLinePotential(shortest_path[i].first, shortest_path[i].second, shortest_path[i + 1].first, shortest_path[i + 1].second, potential);
		}
		//ROS_ERROR("Potential at %d %d is %f",(int)start_x,(int)start_y,potential[(int) (start_x+nx*start_y)]);

		return true;
}

std::vector< std::pair<float, float> > ThetaStarPlanner::makeThetaStarPlan(int start_x, int start_y, int goal_x, int goal_y) {

		ROS_ERROR("Start Theta %d %d to %d %d",start_x,start_y,goal_x,goal_y);

		grid_graph_.SetGridStep(GRID_SIZE);
		grid_graph_.IntializeMap(cost_array_, nx, ny);
		if (!grid_graph_.MakeGrid())
			printf("\n Cannot create the graph\n");
		int src, dst;
		src = grid_graph_.GetNodeIndex(start_x, start_y);
		dst = grid_graph_.GetNodeIndex(goal_x, goal_y);
		grid_graph_.Theta(src, dst);
		ns = nx*ny;
		
		vector<std::pair<int, int> > shortest_path;

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
		reset_step();
		for (i = 0;i < shortest_path.size() - 1;i++) {
			ROS_ERROR("Line from %d %d to %d %d",shortest_path[i].first, shortest_path[i].second, shortest_path[i + 1].first, shortest_path[i + 1].second);
			FillLinePotential(shortest_path[i].first, shortest_path[i].second, shortest_path[i + 1].first, shortest_path[i + 1].second, potential_array_);

		}

		int x1, y1, x2, y2;
		grid_graph_.GetNodeCoordinate(src, x1, y1);
		grid_graph_.GetNodeCoordinate(dst, x2, y2);
		std::vector<std::pair<float, float> > path;
		path.clear();
		getPath(potential_array_, x1, y1, x2, y2, path);

		delete potential_array_;
		return path;
	}




bool ThetaStarPlanner::calculatePotentials(unsigned char* costs, double start_x, double start_y, double end_x, double end_y,
                                        int cycles, float* potential) {

    //ROS_ERROR("Wow, LA  calculate Potential");
    queue_.clear();
    int start_i = start_x+nx*start_y;
    //ROS_ERROR("start %f %f --> %d",start_x,start_y,start_i);
    queue_.push_back(Index(start_i, 0));
    //ROS_ERROR("Wow, LA star searching a goal -step 1");
    int ns_ = nx * ny;
    std::fill(potential, potential + ns_, POT_HIGH);
    //ROS_ERROR("Wow, LA star searching a goal -step 2, %d",start_i);
    potential[start_i] = 0;

    int goal_i = end_x+nx*end_y;
    //ROS_ERROR("goal %f %f --> %d",end_x,end_y,goal_i);
    int cycle = 0;
    //ROS_ERROR("Wow, LA star searching a goal %d",goal_i);
    while (queue_.size() > 0 && cycle < cycles) {
        Index top = queue_[0];
        std::pop_heap(queue_.begin(), queue_.end(), greater1());
        queue_.pop_back();

        int i = top.i;
        if (i == goal_i) {
            //ROS_ERROR("Wow, LA found a goal");
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

    if(costs[next_i]>=lethal_cost_ && !(unknown_ && costs[next_i]==costmap_2d::NO_INFORMATION))
        return;

    potential[next_i] = calculatePotential(potential, costs[next_i] + neutral_cost_, next_i, prev_potential);
    int x = next_i % nx, y = next_i / nx;
    float distance = abs(end_x - x) + abs(end_y - y);

    queue_.push_back(Index(next_i, potential[next_i] + distance * neutral_cost_));
    std::push_heap(queue_.begin(), queue_.end(), greater1());
}

void ThetaStarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    if(!initialized_){
      costmap_ros_ = costmap_ros;
      costmap_ = costmap_ros_->getCostmap();
      frame_id_ = costmap_ros->getGlobalFrameID();
      ros::NodeHandle private_nh("~/" + name);
      private_nh.param("step_size", step_size_, costmap_->getResolution());
      private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.10);
      world_model_ = new base_local_planner::CostmapModel(*costmap_); 
      plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
      //make_plan_srv_ = private_nh.advertiseService("make_plan", &ThetaStarPlanner::makePlanService, this);
      initialized_ = true;
    }
    else
      ROS_WARN("This planner has already been initialized... doing nothing");
  }

  //we need to take the footprint of the robot into account when we calculate cost to obstacles
  double ThetaStarPlanner::footprintCost(double x_i, double y_i, double theta_i){
    if(!initialized_){
      ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
      return -1.0;
    }

    std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();
    //if we have no footprint... do nothing
    if(footprint.size() < 3)
      return -1.0;

    //check if the footprint is legal
    double footprint_cost = world_model_->footprintCost(x_i, y_i, theta_i, footprint);
    return footprint_cost;
  }


bool ThetaStarPlanner::getPlanFromPotential(double start_x, double start_y, double goal_x, double goal_y,
                                      const geometry_msgs::PoseStamped& goal,
                                       std::vector<geometry_msgs::PoseStamped>& plan) {
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }

    std::string global_frame = frame_id_;

    //clear the plan, just in case
    plan.clear();

    std::vector<std::pair<float, float> > path;

    if (!getPath(potential_array_, start_x, start_y, goal_x, goal_y, path)) {
        ROS_ERROR("NO PATH!");
        return false;
    }

    ros::Time plan_time = ros::Time::now();
    for (int i = path.size() -1; i>=0; i--) {
        std::pair<float, float> point = path[i];
        //convert the plan to world coordinates
        double world_x, world_y;
        mapToWorld(point.first, point.second, world_x, world_y);
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = plan_time;
        pose.header.frame_id = global_frame;
        pose.pose.position.x = world_x;
        pose.pose.position.y = world_y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
        plan.push_back(pose);
    }
plan.push_back(goal);

    /*if(old_navfn_behavior_){
            plan.push_back(goal);
    }*/
    return !plan.empty();
}


bool ThetaStarPlanner::getPath(float* potential, double start_x, double start_y, double end_x, double end_y, std::vector<std::pair<float, float> >& path) {
    //ROS_ERROR("ROS::Inside getPath functions");
    std::pair<float, float> current;
    current.first = (int) end_x;
    current.second = (int) end_y;

    int start_index = (int)start_x+nx*(int)start_y;

    //ROS_ERROR("Get path from %f %f (%d) to %f %f (%d %d)",start_x,start_y,start_index,end_x,end_y,(int)end_x,(int)end_y);


    path.push_back(current);
    int c = 0;
    int ns = nx * ny;
    
    while (current.first+nx*current.second != start_index) {
        float min_val = 1e10;
        int min_x = 0, min_y = 0;
        for (int xd = -1; xd <= 1; xd++) {
            for (int yd = -1; yd <= 1; yd++) {
                if (xd == 0 && yd == 0)
                    continue;
                int x = current.first + xd, y = current.second + yd;
                int index = x+nx*y;
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
        
        if(c++>ns*4){
            return false;
        }

    }
    return true;
}


void ThetaStarPlanner::mapToWorld(double mx, double my, double& wx, double& wy) {
    wx = costmap_->getOriginX() + (mx+convert_offset_) * costmap_->getResolution();
    wy = costmap_->getOriginY() + (my+convert_offset_) * costmap_->getResolution();
}

bool ThetaStarPlanner::worldToMap(double wx, double wy, double& mx, double& my) {
    double origin_x = costmap_->getOriginX(), origin_y = costmap_->getOriginY();
    double resolution = costmap_->getResolution();

    if (wx < origin_x || wy < origin_y)
        return false;

    mx = (wx - origin_x) / resolution - convert_offset_;
    my = (wy - origin_y) / resolution - convert_offset_;

    if (mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY())
        return true;

    return false;
}


void ThetaStarPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) {
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    //create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    gui_path.header.frame_id = frame_id_;
    gui_path.header.stamp = ros::Time::now();

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++) {
        gui_path.poses[i] = path[i];
    }

    plan_pub_.publish(gui_path);
}


bool ThetaStarPlanner::makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp) {
    makePlan(req.start, req.goal, resp.plan.poses);

    resp.plan.header.stamp = ros::Time::now();
    resp.plan.header.frame_id = frame_id_;

    return true;
}



  bool ThetaStarPlanner::makePlan(const geometry_msgs::PoseStamped& start, 
      const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){

    if(!initialized_){
      ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
      return false;
    }

    //ROS_ERROR("LA got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);
    //ROS_ERROR("LA is here for testing");


    plan.clear();
    costmap_ = costmap_ros_->getCostmap();

    if(goal.header.frame_id != costmap_ros_->getGlobalFrameID()){
      ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.", 
          costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
      return false;
    }

    tf::Stamped<tf::Pose> goal_tf;
    tf::Stamped<tf::Pose> start_tf;

    poseStampedMsgToTF(goal,goal_tf);
    poseStampedMsgToTF(start,start_tf);

    double useless_pitch, useless_roll, goal_yaw, start_yaw;
    start_tf.getBasis().getEulerYPR(start_yaw, useless_pitch, useless_roll);
    goal_tf.getBasis().getEulerYPR(goal_yaw, useless_pitch, useless_roll);

    //we want to step back along the vector created by the robot's position and the goal pose until we find a legal c

    double wx = start.pose.position.x;
    double wy = start.pose.position.y;
    double start_x, start_y, goal_x, goal_y;
    worldToMap(wx, wy, start_x, start_y);
    wx = goal.pose.position.x;
    wy = goal.pose.position.y;
    worldToMap(wx, wy, goal_x, goal_y);

    nx = costmap_->getSizeInCellsX();
    ny = costmap_->getSizeInCellsY();
    ns=nx*ny;
    potential_array_ = new float[nx * ny];
    outlineMap(costmap_->getCharMap(), nx, ny, costmap_2d::LETHAL_OBSTACLE);

    /*bool found_legal = calculatePotentials(costmap_->getCharMap(), start_x, start_y, goal_x, goal_y,
                                                    nx * ny * 2, potential_array_);  */  

    bool found_legal =ThetaPotentials(costmap_->getCharMap(), start_x, start_y, goal_x, goal_y, potential_array_);

    //ROS_ERROR("LA save a start: %.2f, %.2f, and a goal: %.2f, %.2f", start_x, start_y, goal_x, goal_y);
    try_++;
 
    /*FILE *fp;
    char filename[256];
    sprintf(filename,"/home/lana/%d.bin",try_);
    fp=fopen(filename,"wb");  
    fwrite(&nx,sizeof(int),1,fp);
    fwrite(&ny,sizeof(int),1,fp);
    fwrite(&start_x,sizeof(double),1,fp);
    fwrite(&start_y,sizeof(double),1,fp);
    fwrite(&goal_x,sizeof(double),1,fp);
    fwrite(&goal_y,sizeof(double),1,fp);
    fwrite(costmap_->getCharMap(),sizeof(unsigned char),nx*ny,fp);
    fwrite(potential_array_,sizeof(float),nx*ny,fp);
    fclose(fp);*/
   
    if (found_legal) {
        //extract the plan
	    //ROS_ERROR("Start to retrieve the plan from start to goal");
        if (getPlanFromPotential(start_x, start_y, goal_x, goal_y, goal, plan)) {
            //make sure the goal we push on has the same timestamp as the rest of the plan
            geometry_msgs::PoseStamped goal_copy = goal;
            goal_copy.header.stamp = ros::Time::now();
            plan.push_back(goal_copy);
        } else {
            ROS_ERROR("Failed to get a plan from potential when a legal potential was found. This shouldn't happen.");
        }
    }else{
        ROS_ERROR("Failed to get a plan.");
    }

    publishPlan(plan);
    delete potential_array_;
    return !plan.empty();
  }

};
