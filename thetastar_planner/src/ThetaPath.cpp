#include <thetastar_planner/ThetaPath.h>
#include <cmath>
#include <algorithm>
#include <iostream>

ThetaPath::ThetaPath() {
	x = y = nx = ny = segment_len = NULL;
	num_of_segment = 0;
    grid_graph_ = NULL;
}

void ThetaPath::BuildPath(std::vector<Types::Point> path_) {
	float s = 0;
	num_of_segment = path_.size() - 1;
	x = new float[num_of_segment];
	y = new float[num_of_segment];
	nx = new float[num_of_segment];
	ny = new float[num_of_segment];
	segment_len = new float[num_of_segment];
	std::reverse(path_.begin(), path_.end());
	end.x = (float)path_[num_of_segment].x;
	end.y = (float)path_[num_of_segment].y;

	for (int i = 0; i < num_of_segment; ++i) {
		x[i] = (float)path_[i].x;
		y[i] = (float)path_[i].y;
		nx[i]= (float)path_[i + 1].x - (float)path_[i].x;
		ny[i] = (float)path_[i + 1].y - (float)path_[i].y;
		s = nx[i] * nx[i] + ny[i] * ny[i];
		s = sqrt(s);
		segment_len[i] = s;
		nx[i] /= s;
		ny[i] /= s;
	}
}

Types::Point2f ThetaPath:: Theta2FlowForce(Types::Point2f s) {
	float min_dist,dist,tmp,alpha;
	Types::Point2f return_force,force,follow_force;

	min_dist = sqrt((s.x - end.x)*(s.x - end.x) + (s.y - end.y)*(s.y - end.y)) + 1e-12;

	return_force.x = -(s.x - end.x) / min_dist;
	return_force.y = -(s.y - end.y) / min_dist;
	follow_force = return_force;

    int firstIf = 0;
    int secIf = 0;
    int i = 0;

	for (i = 0; i < num_of_segment; ++i) {
		force.x = x[i] - s.x;
		force.y = y[i] - s.y;

		tmp = force.x * nx[i] + force.y * ny[i];

		force.x = force.x - tmp * nx[i];
		force.y = force.y - tmp * ny[i];

		dist = sqrt(force.x*force.x + force.y*force.y)+1e-12;

		if (dist < min_dist && tmp<=0 && abs(tmp)<=segment_len[i]) {
            firstIf++;
			min_dist = dist;
			return_force.x = force.x / dist;
			return_force.y = force.y / dist;
			follow_force.x = nx[i];
			follow_force.y = ny[i];
		}

		dist = sqrt((s.x - x[i])*(s.x - x[i]) + (s.y - y[i])*(s.y - y[i])) + 1e-12;

		if (dist < min_dist) {
            secIf++;
			return_force.x = -(s.x - x[i]) / dist;
			return_force.y = -(s.y - y[i]) / dist;
			follow_force.x = nx[i];
			follow_force.y = ny[i];
			min_dist = dist;
		}

	}

    if (min_dist < FLOWFIELD_SCOPE) {
		float k1 = 1.0 / 100, k2 = 1.0;
		//alpha = e^(-k1d(p,ai))
		alpha = exp(-min_dist *k1);
		//return_force = ((ai - p) - ((ai - p) * ni)ni + k2 * alpha * ni
		//return_force = (1 - alpha)*return_force + k2*alpha*follow_force;
        return_force.x = (1 - alpha) * return_force.x + k2 * alpha * follow_force.x;
        return_force.y = (1 - alpha) * return_force.y + k2 * alpha * follow_force.y;
	}
	else {
		return_force.x = 0;
		return_force.y = 0;
	}
		
	return return_force;
}

ThetaPath::~ThetaPath() {
	if (x != NULL) delete x;
	if (y != NULL) delete y;
	if (nx != NULL) delete nx;
	if (ny != NULL) delete ny;
	if (segment_len != NULL) delete segment_len;
	num_of_segment = 0;
}

void ThetaPath::setInitPath(ThetaStar4Grid *g) {
	grid_graph_ = g;
}

void ThetaPath::InitFlowField(Types::Point s, Types::Point d) {
	if (x != NULL) delete x;
	if (y != NULL) delete y;
	if (nx != NULL) delete nx;
	if (ny != NULL) delete ny;
	if (segment_len != NULL) delete segment_len;
	num_of_segment = 0;

	int src, dst;
	src = grid_graph_->GetNodeIndex(s.x, s.y);

	std::cout << "src (X,Y,VAL): " << s.x << " " << s.y << " " << src << std::endl;

	dst = grid_graph_->GetNodeIndex(d.x, d.y);

	std::cout << "dst (X,Y,VAL): " << d.x << " " << d.y << " " << dst << std::endl;

	grid_graph_->Theta(src, dst);
	int x,y,u = dst;
	std::vector<Types::Point> shortest_path;
	Types::Point current;
	grid_graph_->GetNodeCoordinate(u, x, y);
	current.x = x;current.y = y;
	shortest_path.push_back(current);

	std::cout << "u: " << u << " src: " << src << std::endl;
	std::cout << "g_score: " << grid_graph_->g_score_[u] << std::endl;
	while (u != src && grid_graph_->g_score_[u] != INF) {
		grid_graph_->GetNodeCoordinate(grid_graph_->parent_list_[u],x,y);
		current.x = x;
        current.y = y;

		std::cout << "BOOP" << std::endl;
		std::cout << "X Y " << current.x << " " << current.y << std::endl;
		shortest_path.push_back(current);
		u = grid_graph_->parent_list_[u];
	}
	BuildPath(shortest_path);

}
