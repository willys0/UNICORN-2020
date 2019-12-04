#include <thetastar_planner/ThetaStar4Grid.h>
#include <thetastar_planner/ThetaPath.h>
#include <thetastar_planner/thetastar_planner.h>
#include <pluginlib/class_list_macros.h>

#include <iostream>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(thetastar_planner::ThetaStarPlanner, nav_core::BaseGlobalPlanner);
namespace thetastar_planner {

    ThetaStarPlanner::ThetaStarPlanner() : 
        costmap_ros_(NULL), initialized_(false),convert_offset_(0.5){}

    ThetaStarPlanner::ThetaStarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros) : 
        costmap_ros_(NULL), initialized_(false),convert_offset_(0.5){
        initialize(name, costmap_ros);
    }

    ThetaStarPlanner::~ThetaStarPlanner(){}

    void ThetaStarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    {
        // Initialize parameters
        if(!initialized_){
            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros_->getCostmap();
            ros::NodeHandle private_nh("~/" + name);
            private_nh.param("step_size", step_size_, costmap_->getResolution());
            private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.10);
            plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
            make_plan_srv_ = private_nh.advertiseService("make_plan", &ThetaStarPlanner::makePlanService, this);
            initialized_ = true;
            frame_id_ = costmap_ros->getGlobalFrameID();
            recalc_old_goal_ = false;
        }
        else{
            ROS_WARN("This planner has already been initialized... doing nothing");
            return;
        }

        // Initialize the grid graph
        grid_graph_.SetGridStep(GRID_SIZE);
        grid_graph_.IntializeMap(costmap_->getCharMap(), costmap_->getSizeInCellsY(), costmap_->getSizeInCellsX());
        if(!grid_graph_.MakeGrid()) {
            std::cout << "Failed to make grid (grid_graph_)" << std::endl;
            return;
        } else {
            std::cout << "Make grid (grid_graph_), ok!" << std::endl;
        }
    }

    // Create sub-points between all the points in the path since the local planner needs the path to look like that
    void ThetaStarPlanner::createSubPoints(int theta_ret, geometry_msgs::PoseStamped& current_goal, std::vector<geometry_msgs::PoseStamped>& plan)
    {
        Types::Point2f heading_vector;
        ros::Time plan_time = ros::Time::now();
        geometry_msgs::PoseStamped new_goal;
        new_goal.pose.position.z = 0.0;
        new_goal.pose.orientation.x = 0.0;
        new_goal.pose.orientation.y = 0.0;
        new_goal.pose.orientation.z = 0.0;
        new_goal.pose.orientation.w = 1.0;

        // stamp and frame_id needs to be set in order for the msg to be correct
        new_goal.header.stamp = plan_time;
        new_goal.header.frame_id = frame_id_;

        int divide_path_number = 0;
        float length_of_segment = 0;
        double next_world_x = 0, next_world_y = 0, world_x = 0, world_y = 0;
        
        for(int i = 0; i < theta_path_.getNumOfSegment(); i++){

	    std::cout << (int)costmap_->getCost((unsigned int)theta_path_.getX(i), (unsigned int)theta_path_.getY(i)) << std::endl;

            // Get the current point in the path
            mapToWorld(theta_path_.getX(i), theta_path_.getY(i), world_x, world_y);

            // Get next point
            if(i < theta_path_.getNumOfSegment() - 1){
                mapToWorld(theta_path_.getX(i + 1), theta_path_.getY(i + 1), next_world_x, next_world_y);
            }
            else
            {
                // Goal is removed, save old goal so we can recalculate it next call if we need to
                if (theta_ret == 1) {
                    old_goal_ = current_goal;
                    recalc_old_goal_ = true;
                    break;
                }
                else if (theta_ret == 0) {
                    recalc_old_goal_ = false;
                }

                next_world_x = current_goal.pose.position.x;
                next_world_y = current_goal.pose.position.y;
            }

            // Calculate the length of the segment in order to split it into smaller parts
            length_of_segment = std::sqrt(pow(world_x - next_world_x, 2) + pow(world_y - next_world_y, 2));
            divide_path_number = round(length_of_segment * 100 / 10); // Get length in cm
            
            // Calculate the heading of the sub-point to correctly orient the robot
            heading_vector.x = next_world_x - world_x;
            heading_vector.y = next_world_y - world_y;
            if (theta_ret == 1)
                new_goal.pose.orientation.z = atan2(heading_vector.y,heading_vector.x);
            else
                new_goal.pose.orientation.z = current_goal.pose.orientation.z;          
            // Add points to the path
            for (int j = 0; j < divide_path_number; j++)
            {            
                new_goal.pose.position.x = world_x + (heading_vector.x / divide_path_number) * j;
                new_goal.pose.position.y = world_y + (heading_vector.y / divide_path_number) * j;
                plan.push_back(new_goal);
            }
        }
    }

    bool ThetaStarPlanner::goalIsInObstacle(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, double& mx, double& my)
    {
        int testcost = 0;
        double test_x = 0, test_y = 0, t_x = 0, t_y = 0;
        double scale = 0.0, dScale = 0.01;

        // Calculate difference
        double diff_x = start.pose.position.x - goal.pose.position.x;
        double diff_y = start.pose.position.y - goal.pose.position.y;

        while(1) {
            // Calculate a new position
            test_x = goal.pose.position.x + (scale * diff_x);
            test_y = goal.pose.position.y + (scale * diff_y);

            // Check the cost in the new point
            worldToMap(test_x, test_y, t_x, t_y);
            testcost = (int)costmap_->getCost((unsigned int)t_x, (unsigned int)t_y);

            // Make sure the cost in the new point is zero and that the node in the grid graph
            // is a valid node
            if ((testcost == 0) && (grid_graph_.GetNodeIndex(t_x, t_y) != 0)) {
                break;
            }
            // Could not find a valid new goal position
            if(scale > 1) {
                return false;
            }

            scale += dScale;
        }

        mx = test_x;
        my = test_y;

        return true;
    }

    bool ThetaStarPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
    {
        if (!initialized_) {
            ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
            return false;
        }

        geometry_msgs::PoseStamped current_goal;
        double start_x = 0, start_y = 0, goal_x = 0, goal_y = 0;
        double world_x = 0, world_y = 0;
        int goal_cost = 0;
        int theta_ret_ = 0;

        plan.clear();
        current_goal = goal;

        // Dummy goal if the robot has reached the goal
        if(current_goal.pose.position.x <= -2400000.0 && current_goal.pose.position.y <= -2400000.0) 
        {
            // The original goal was not reached, because it was in unknown space
            if(recalc_old_goal_) {
                current_goal = old_goal_;
                recalc_old_goal_ = false;
            } 
            // The original goal was reached, publish empty plan
            else {
                plan.push_back(start);
                publishPlan(plan);
                return true;
            }        
        }
        grid_graph_.IntializeMap(costmap_->getCharMap(), costmap_->getSizeInCellsY(), costmap_->getSizeInCellsX());

	// Update the grid since the cost map might have been updated
        if(!grid_graph_.MakeGrid()) {
            std::cout << "Failed to make grid (grid_graph_)" << std::endl;
        }


        // Convert postions
        worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y);
        worldToMap(current_goal.pose.position.x, current_goal.pose.position.y, goal_x, goal_y);

        // Check if start position is in an obstacle
        int start_cost = (int)costmap_->getCost((unsigned int)start_x, (unsigned int)start_y);
        if (start_cost >= 128) {
            std::cout << "Moving start pos" << std::endl;
            // New start position
            double rec_x = 0;
            double rec_y = 0;

            bool legal = goalIsInObstacle(goal, start, rec_x, rec_y);

            // No valid start could be found
            if(!legal) {
                std::cout << "No new goal pos can be found" << std::endl;
                plan.push_back(start);
                publishPlan(plan);
                return true;
            }

            // Valid start, update the start position
            worldToMap(rec_x,rec_y, start_x, start_y);
        }

        
        // If the current goal is inside an obstacle, calculate the closes legal goal position
        // in the direction of the start point
        goal_cost = (int)costmap_->getCost((unsigned int)goal_x, (unsigned int)goal_y);

        if (goal_cost >= 128 && goal_cost <= 254) {
            // New goal position
            double rec_x = 0;
            double rec_y = 0;

            bool legal = goalIsInObstacle(start, goal, rec_x, rec_y);

            // No valid goal could be found
            if(!legal) {
                std::cout << "No new goal pos can be found" << std::endl;
                plan.push_back(start);
                publishPlan(plan);
                return true;
            }

            // Valid goal, update the goal position
            current_goal.pose.position.x = rec_x;
            current_goal.pose.position.y = rec_y;
            worldToMap(rec_x,rec_y, goal_x, goal_y);
        }

        // Calculate path
        theta_path_.setInitPath(&grid_graph_);
        theta_ret_ = theta_path_.InitFlowField(Types::Point(start_x, start_y), Types::Point(goal_x, goal_y), costmap_);

        // No path found
        if(theta_ret_ == -1) {
            plan.push_back(start);
            publishPlan(plan);
            return true;
        }

        // Path found, create subpoints
        createSubPoints(theta_ret_, current_goal, plan);
        publishPlan(plan);

        return true;
    }

    void ThetaStarPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path)
    {
        if (!initialized_) {
            ROS_ERROR(
                    "This planner has not been initialized yet, but it is being used, please call initialize() before use");
            return;
        }

        // Create a message for the plan
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


    bool ThetaStarPlanner::makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp)
    {
        makePlan(req.start, req.goal, resp.plan.poses);

        resp.plan.header.stamp = ros::Time::now();
        resp.plan.header.frame_id = frame_id_;

        return true;
    }
    //Convert map coordinates to world coordinates
    void ThetaStarPlanner::mapToWorld(double mx, double my, double& wx, double& wy)
    {
        wx = costmap_->getOriginX() + (mx+convert_offset_) * costmap_->getResolution();
        wy = costmap_->getOriginY() + (my+convert_offset_) * costmap_->getResolution();
    }

    //Convert world coordinates to map coordinates
    bool ThetaStarPlanner::worldToMap(double wx, double wy, double& mx, double& my)
    {
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

    //Create a barrier around the map
    void ThetaStarPlanner::outlineMap(unsigned char* costarr, int nx, int ny, unsigned char value)
    {
        unsigned char* pc = costarr;
        
        for (int i = 0; i < nx; i++) { *pc++ = value; }

        pc = costarr + (ny - 1) * nx;
        for (int i = 0; i < nx; i++) { *pc++ = value; }

        pc = costarr;
        for (int i = 0; i < ny; i++, pc += nx) { *pc = value; }

        pc = costarr + nx - 1;
        for (int i = 0; i < ny; i++, pc += nx) { *pc = value; }
    }
}

