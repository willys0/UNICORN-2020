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
            world_model_ = new base_local_planner::CostmapModel(*costmap_);
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
        } else {
            std::cout << "Make grid (grid_graph_), ok!" << std::endl;
        }
    }

    bool ThetaStarPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
    {
        ros::Time plan_time = ros::Time::now();
        geometry_msgs::PoseStamped new_goal;
        new_goal.pose.position.z = 0.0;
        new_goal.pose.orientation.x = 0.0;
        new_goal.pose.orientation.y = 0.0;
        new_goal.pose.orientation.z = 0.0;
        new_goal.pose.orientation.w = 1.0;
        new_goal.header.stamp = plan_time;
        new_goal.header.frame_id = frame_id_;

        geometry_msgs::PoseStamped current_goal;
        double start_x, start_y, goal_x, goal_y;
        double world_x, world_y;
        double next_world_x, next_world_y;
        float length_of_segment;
        int goal_cost;
        bool goal_is_removed;

        current_goal = goal;

        std::vector<geometry_msgs::Point> fp = costmap_ros_->getRobotFootprint();

        std::cout << "FP Size: " << fp.size();

        /*for (int i = 0; i < fp.size(); i++) {
            std::cout << "FP(" << i << "): " << fp.at(i) << std::endl;
        }*/

        // The robot has reached the goal
        if(current_goal.pose.position.x <= -2400000.0 && current_goal.pose.position.y <= -2400000.0) 
        {
            // The original goal was not reached, because it was in unknown space
            if(recalc_old_goal_) {
                current_goal = old_goal_;
                recalc_old_goal_ = false;
            } 
            // The original goal was reached, publish empty plan
            else {
                plan.clear();
                plan.push_back(start);
                publishPlan(plan);
                return true;
            }        
        }

        // Convert postions
        worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y);
        worldToMap(current_goal.pose.position.x, current_goal.pose.position.y, goal_x, goal_y);

        // Find the costmap index of the current goal position
        goal_cost = (int)costmap_->getCost((unsigned int)goal_x, (unsigned int)goal_y);

        //std::cout << "Cost: " << goal_cost << std::endl;
        // Current goal point is inside an obstacle, push empty plan
        if (goal_cost >= 250 && goal_cost <= 254) {
            plan.clear();
            plan.push_back(start);
            publishPlan(plan);
            return true;
        }
        // Update the grid since the cost map might have been updated
        if(!grid_graph_.MakeGrid()) {
            std::cout << "Failed to make grid (grid_graph_)" << std::endl;
        }

        plan.clear();
        
        // std::cout << "Startcell XY: " << start_x << " " << start_y << std::endl;
        // std::cout << "goalCell XY: " << goal_x << " " << goal_y << std::endl;

        // Calculate path
        theta_path_.setInitPath(&grid_graph_);
        int theta_ret_ = 0;
        theta_ret_ = theta_path_.InitFlowField(Types::Point(start_x, start_y), Types::Point(goal_x, goal_y), costmap_);

        // No path found
        if(theta_ret_ == -1) {
            plan.push_back(start);
            publishPlan(plan);
            return true;
        }

        // Create sub-points in the path since the local planner needs the path to look like that
        for(int i = 0; i < theta_path_.getNumOfSegment(); i++){

            //mapToWorld(theta_path_.x[i], theta_path_.y[i], world_x, world_y);
            mapToWorld(theta_path_.getX(i), theta_path_.getY(i), world_x, world_y);
            //std::cout << "X: " << world_x << std::endl;
            //std::cout << "Y: " << world_y << std::endl;

            if(i < theta_path_.getNumOfSegment() - 1)
                mapToWorld(theta_path_.getX(i + 1), theta_path_.getY(i + 1), next_world_x, next_world_y);

            else
            {
                // Goal is removed, save old goal so we can recalculate it next call if we need to
                if (theta_ret_ == 1) {
                    old_goal_ = current_goal;
                    recalc_old_goal_ = true;
                    break;
                }
                else if (theta_ret_ == 0) {
                    recalc_old_goal_ = false;
                }

                next_world_x = current_goal.pose.position.x;
                next_world_y = current_goal.pose.position.y;
            }

            // Calculate the length of the segment in order to split it into smaller parts
            length_of_segment = std::sqrt(pow(world_x-next_world_x,2)+pow(world_y-next_world_y,2));
            int divide_path_number = round(length_of_segment * 100 / 10); // Get length in cm
            Types::Point2f heading_vector;
            heading_vector.x = next_world_x - world_x;
            heading_vector.y = next_world_y - world_y;
            new_goal.pose.orientation.z = atan2(heading_vector.y,heading_vector.x);
            std::cout << "Orientation: " << new_goal.pose.orientation.z;
            for (int j = 0; j < divide_path_number; j++)
            {            
                new_goal.pose.position.x = world_x + (heading_vector.x/divide_path_number)*j;
                new_goal.pose.position.y = world_y + (heading_vector.y/divide_path_number)*j;
                plan.push_back(new_goal);
            }
        }

        publishPlan(plan);

        return true;
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
    //Convert map coordinates to world coordinates
    void ThetaStarPlanner::mapToWorld(double mx, double my, double& wx, double& wy) {
        wx = costmap_->getOriginX() + (mx+convert_offset_) * costmap_->getResolution();
        wy = costmap_->getOriginY() + (my+convert_offset_) * costmap_->getResolution();
    }

    //Convert world coordinates to map coordinates
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

    //Create a barrier around the map
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
}

