#pragma once
#include <vector>
#include <thetastar_planner/ThetaStar4Grid.h>
#include <thetastar_planner/types.h>

#define FLOWFIELD_SCOPE 20

class ThetaPath
{
	ThetaStar4Grid *grid_graph_;

public:
	float *x, *y;
	float *nx, *ny;
	float *segment_len;
	Types::Point2f end;
	int num_of_segment;
	ThetaPath();
	void BuildPath(std::vector<Types::Point> path_);
	Types::Point2f Theta2FlowForce(Types::Point2f s);
	void setInitPath(ThetaStar4Grid *g);
	void InitFlowField(Types::Point s,Types::Point d);
	~ThetaPath();
};

 