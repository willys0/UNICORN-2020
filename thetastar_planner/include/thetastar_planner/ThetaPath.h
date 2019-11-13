#pragma once
#include <vector>
#include <thetastar_planner/ThetaStar4Grid.h>
#include <thetastar_planner/types.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>

class ThetaPath
{
	private:
		ThetaStar4Grid *grid_graph_;
		std::vector<Types::Point> path_;

	public:
		ThetaPath();
		~ThetaPath();

		void setInitPath(ThetaStar4Grid *g);
		int InitFlowField(Types::Point s,Types::Point d, costmap_2d::Costmap2D* costmap_);
		int getNumOfSegment();
		int getX(int index);
		int getY(int index);
};

 