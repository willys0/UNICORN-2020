#pragma once
#include <ros/ros.h>
#include <costmap_2d/cost_values.h> 
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <nav_msgs/GetPlan.h>
namespace thetastar_planner{
    #define MAX_DIST 1.0e4
    #define GRID_SIZE 4 

    class ThetaStarPlanner : public nav_core::BaseGlobalPlanner {
        public:
            ThetaStarPlanner();
            ThetaStarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
            ~ThetaStarPlanner();
            void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
            bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

            bool makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp);
            void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);

            void mapToWorld(double mx, double my, double& wx, double& wy);
            bool worldToMap(double wx, double wy, double& mx, double& my);

            void outlineMap(unsigned char* costarr, int nx, int ny, unsigned char value);
    
        private:
            ThetaStar4Grid grid_graph_;
            ThetaPath theta_path_;

            ros::ServiceServer make_plan_srv_;
            costmap_2d::Costmap2DROS* costmap_ros_;
            costmap_2d::Costmap2D* costmap_;
            base_local_planner::WorldModel* world_model_;
            geometry_msgs::PoseStamped old_goal_;
            ros::Publisher plan_pub_;
            std::string frame_id_;
            
            double step_size_;
            double min_dist_from_robot_;
            float convert_offset_;
            bool initialized_;   
            bool recalc_old_goal_; 
    };
};  

