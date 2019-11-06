#include <thetastar_planner/ThetaStar4Grid.h>
#include <thetastar_planner/ThetaPath.h>
#include <thetastar_planner/thetastar_planner.h>
#include <pluginlib/class_list_macros.h>

#include <iostream>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(thetastar_planner::ThetaStarPlanner, nav_core::BaseGlobalPlanner);
namespace thetastar_planner {

    ThetaStarPlanner::ThetaStarPlanner() : 
        costmap_ros_(NULL), initialized_(false),unknown_(true), lethal_cost_(253), neutral_cost_(50),convert_offset_(0.5){ try_=0;}

    ThetaStarPlanner::ThetaStarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros) : 
        costmap_ros_(NULL), initialized_(false),unknown_(true), lethal_cost_(253), neutral_cost_(50),convert_offset_(0.5){
        try_=0;
        initialize(name, costmap_ros);
    }


    void ThetaStarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    {
        std::cout << "Init." << std::endl;
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
        }
        else{
            ROS_WARN("This planner has already been initialized... doing nothing");
            return;
        }

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
        std::cout << "makePlan" << std::endl;

        costmap_ = costmap_ros_->getCostmap();
        grid_graph_.SetGridStep(GRID_SIZE);
        grid_graph_.IntializeMap(costmap_->getCharMap(), costmap_->getSizeInCellsY(), costmap_->getSizeInCellsX());
        std::cout << "Total size: " << costmap_->getSizeInCellsY() * costmap_->getSizeInCellsX() << std::endl;
        if(!grid_graph_.MakeGrid()) {
            std::cout << "Failed to make grid (grid_graph_)" << std::endl;
        } else {
            std::cout << "Make grid (grid_graph_), ok!" << std::endl;
        }

        plan.clear();

        Types::Point startCell;
        Types::Point goalCell;

        double start_x, start_y, goal_x, goal_y;
        worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y);
        worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y);

        startCell.x = start_x;
        startCell.y = start_y;
        goalCell.x = goal_x;
        goalCell.y = goal_y;
        
        std::cout << "Startcell XY: " << start_x << " " << start_y << std::endl;

        std::cout << "goalCell XY: " << goal_x << " " << goal_y << std::endl;

        theta_path_.setInitPath(&grid_graph_);

        theta_path_.InitFlowField(startCell, goalCell);

        // Create a mock up plan (for now)
        //std::cout << "Current plan" << std::endl;
        //std::cout << "Num segment: " << theta_path_.num_of_segment << std::endl;

        ros::Time plan_time = ros::Time::now();

        double world_x, world_y;
        double next_world_x, next_world_y;

        //std::vector<float> length_of_segment;
        float length_of_segment;

        geometry_msgs::PoseStamped new_goal = goal;
        new_goal.pose.position.z = 0.0;
        new_goal.pose.orientation.x = 0.0;
        new_goal.pose.orientation.y = 0.0;
        new_goal.pose.orientation.z = 0.0;
        new_goal.pose.orientation.w = 1.0;

        for(int i = 0; i < theta_path_.num_of_segment; i++){

            mapToWorld(theta_path_.x[i], theta_path_.y[i], world_x, world_y);

            std::cout << "X: " << world_x << std::endl;
            std::cout << "Y: " << world_y << std::endl;

            if(i < theta_path_.num_of_segment - 1)
                mapToWorld(theta_path_.x[i+1], theta_path_.y[i+1], next_world_x, next_world_y);
            else
            {
                next_world_x = goal.pose.position.x;
                next_world_y = goal.pose.position.y;
            }

                length_of_segment = std::sqrt(pow(world_x-next_world_x,2)+pow(world_y-next_world_y,2));
                int divide_path_number = round(length_of_segment*100 / 10); // Get length in cm
                Types::Point2f heading_vector;
                heading_vector.x = next_world_x - world_x;
                heading_vector.y = next_world_y - world_y;
                std::cout << "Lenght segment: " << length_of_segment << " Divide_path_number: " << divide_path_number << std::endl;
                std::cout << "Heading vector: " << heading_vector.x << " " << heading_vector.y << std::endl; 
                std::cout << "Start:" << start_x << " " << start_y << " Goal " << goal_x << " " << goal_y << std::endl;

                for (int j = 0; j < divide_path_number; j++)
                {            
                    new_goal.header.stamp = plan_time;
                    new_goal.header.frame_id = frame_id_;
                    new_goal.pose.position.x = world_x + (heading_vector.x/divide_path_number)*j;
                    new_goal.pose.position.y = world_y + (heading_vector.y/divide_path_number)*j;
                    
                    //std::cout << "Heading vector: " << heading_vector.x << " " << heading_vector.y << std::endl; 
                    std::cout << "Added: " << new_goal.pose.position.x << " " << new_goal.pose.position.y << std::endl;
                    plan.push_back(new_goal);
                }
        }

        plan.push_back(goal);
        publishPlan(plan);

        return true;
    }

    void ThetaStarPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) {
        std::cout << "publishPlan" << std::endl;
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
}

