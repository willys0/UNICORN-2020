#include <thetastar_planner/ThetaPath.h>
#include <cmath>
#include <algorithm>
#include <iostream>

ThetaPath::ThetaPath() 
{
    grid_graph_ = NULL;
}

ThetaPath::~ThetaPath() 
{
}

void ThetaPath::setInitPath(ThetaStar4Grid *g) 
{
	grid_graph_ = g;
}

int ThetaPath::InitFlowField(Types::Point s, Types::Point d, costmap_2d::Costmap2D* costmap_) 
{
	int src, dst, x, y, u;

	path_.clear();

	src = grid_graph_->GetNodeIndex(s.x, s.y);
	u = dst = grid_graph_->GetNodeIndex(d.x, d.y);
	// std::cout << "src (X,Y,VAL): " << s.x << " " << s.y << " " << src << std::endl;
	// std::cout << "dst (X,Y,VAL): " << d.x << " " << d.y << " " << dst << std::endl;

	if(!grid_graph_->Theta(src, dst)){
		return -1; // No path
	}

	// Push back goal
	grid_graph_->GetNodeCoordinate(u, x, y);
	path_.push_back(Types::Point(x,y));

	// Check if goal is in unknown
	bool goalIsInUnknown = ((int)costmap_->getCost((unsigned int)x, (unsigned int)y) == 255);
	bool unkownDone = !goalIsInUnknown;

	// Find shortest path
	while (u != src && grid_graph_->GetGCost(u) != INF) {
		// Checks if the goal node is in unknown, if it is remove all the nodes in
		// unkown so the goal position will be on the edge of unkown.
		// All other nodes in the path that are in unknown will be considered in LoS,
		// this is to avoid the path sticking to unknown regions.
		if(!unkownDone) {
			grid_graph_->GetNodeCoordinate(grid_graph_->getRealParent(u),x,y);

			// Check if node is in unknown (255)
			if((int)costmap_->getCost((unsigned int)x, (unsigned int)y) == 255) {
				u = grid_graph_->getRealParent(u);
				continue;
			}

			// Set unkownDone to true since our new goal pos is not in unknown
			unkownDone = true;
			
			// Add node with no LoS
			grid_graph_->GetNodeCoordinate(grid_graph_->getRealParent(u),x,y);
			path_.push_back(Types::Point(x,y));
			u = grid_graph_->getRealParent(u);

			continue;
		}

		// Add node with LoS
		grid_graph_->GetNodeCoordinate(grid_graph_->getParent(u),x,y);
		path_.push_back(Types::Point(x,y));
		u = grid_graph_->getParent(u);
		
	}

	// Reverse the path since we're building it from goal to start
	std::reverse(path_.begin(), path_.end());
	
	return (int)goalIsInUnknown;
}

int ThetaPath::getNumOfSegment()
{
	return path_.size() - 1;
}

int ThetaPath::getX(int index)
{
	return path_.at(index).x;
}

int ThetaPath::getY(int index)
{
	return path_.at(index).y;
}
