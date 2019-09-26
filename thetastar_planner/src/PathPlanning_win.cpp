#include <stdio.h>
#include <conio.h>
#include <opencv2/opencv.hpp>
#include "thetastar_planner.h"

using namespace cv;
using namespace thetastar_planner;


int main() {
	Mat map;
    std::string file_path = "D:\\Backup from Laptop\\Lana\\ICRA2019\\workspace\\data\\thetastar_map.bin";
	//std::string file_path = "D:\\Backup from Laptop\\Lana\\ICRA2019\\workspace\\data\\0.bin";
	FILE *fp;
	fp = fopen(file_path.c_str(), "rb");
	int nx, ny,s;
		double start_x, start_y, goal_x, goal_y;
	unsigned char *costmap;
	fread(&nx, sizeof(int), 1, fp);
	fread(&ny, sizeof(int), 1, fp);
	fread(&start_x, sizeof(double), 1, fp);
	fread(&start_y, sizeof(double), 1, fp);
	fread(&goal_x, sizeof(double), 1, fp);
	fread(&goal_y, sizeof(double), 1, fp);
	printf("start %.2f %.2f goal %.2f %.2f", start_x, start_y, goal_x, goal_y);
	getchar();
	goal_x = 1500;
	goal_y = 1500;
	
	cv::Mat cost_img(nx, ny, CV_8UC1),color_map(nx,ny,CV_8UC3);
	unsigned char *ptr = cost_img.data,*pc=color_map.data;
	unsigned char *cost_map;
	cost_map = new unsigned char[nx*ny];
	fread(ptr, sizeof(unsigned char), nx*ny, fp);
	memcpy(cost_map, ptr, nx*ny);
	float *potential_array_;
	potential_array_ = new float[nx*ny];
	fread(potential_array_, sizeof(float), nx*ny, fp);

	ThetaStarPlanner *planner_;
	s = nx*ny;
	for (int i = 0;i < s;i++, ptr++) {
		*pc++ = *ptr;
		*pc++ = *ptr;
		*pc++ = *ptr;
	}

	fclose(fp);
	planner_= new  ThetaStarPlanner();
	planner_->initialize(nx,ny, cost_map);
	//std::vector<std::pair<float, float>> path;
	//planner_->getPath(potential_array_, start_x, start_y, goal_x, goal_y, path);
	std::vector<std::pair<float, float>> path=planner_->makePlan(start_x, start_y, goal_x, goal_y);

	for (int i = 0;i < path.size()-1;i++) {
		std::pair<float, float> p1 = path[i];
		std::pair<float, float> p2 = path[i+1];
		line(color_map, Point((int)p1.first, (int)p1.second), Point((int)p2.first, (int)p2.second), cv::Scalar(255, 0, 0), 2);
	}

	cv::circle(color_map, cv::Point((int)start_x, (int)start_y), 10, cv::Scalar(0, 0, 255), 2);
	cv::circle(color_map, cv::Point((int)goal_x, (int)goal_y), 10, cv::Scalar(0, 0, 255), 2);
	cv::resize(color_map, color_map,Size(),0.25,0.25);
	imshow("Astar",color_map);
	//this is for Theta*
	cv::waitKey();

	printf("Start Theta-star to search for the shortest path fro binary map");
	
	Mat color_map_inv(nx, ny, CV_8UC3);
	ptr = cost_img.data;
	pc = color_map_inv.data;
	s = nx*ny;
	for (int i = 0;i < s;i++, ptr++) {
		*pc++ = *ptr;
		*pc++ = *ptr;
		*pc++ = *ptr;
	}

	std::vector<std::pair<float, float>> path_theta=planner_->makeThetaStarPlan(start_x, start_y, goal_x, goal_y);
	/*int src, dst;
	src = planner_->grid_graph_.GetNodeIndex(start_x, start_y);
	dst = planner_->grid_graph_.GetNodeIndex(goal_x, goal_y);
	int u = dst;
	vector<Point> shortest_path;
	int x, y;
	planner_->grid_graph_.GetNodeCoordinate(u, x, y);
	shortest_path.push_back(Point(x,y));

	while (u != src && planner_->grid_graph_.g_value[u] != INF) {
		planner_->grid_graph_.GetNodeCoordinate(planner_->grid_graph_.parent[u],x,y);
		shortest_path.push_back(Point(x, y));
		u = planner_->grid_graph_.parent[u];
	}
	printf("shortest path size %d", shortest_path.size());

	for(int i=0;i<shortest_path.size()-1;i++)
		line(color_map_inv, Point(shortest_path[i]), Point(shortest_path[i+1]), cv::Scalar(255, 0, 0), 2);*/
	
	
	for (int i = 0;i < path_theta.size() - 1;i++) {
		std::pair<float, float> p1 = path_theta[i];
		std::pair<float, float> p2 = path_theta[i + 1];
		line(color_map_inv, Point((int)p1.first, (int)p1.second), Point((int)p2.first, (int)p2.second), cv::Scalar(255, 0, 0), 2);
	}

	cv::circle(color_map_inv, cv::Point((int)start_x, (int)start_y), 10, cv::Scalar(0, 0, 255), 2);
	cv::circle(color_map_inv, cv::Point((int)goal_x, (int)goal_y), 10, cv::Scalar(0, 0, 255), 2);
	cv::resize(color_map_inv, color_map_inv, Size(), 0.25, 0.25);
	imshow("Theta Star", color_map_inv);
	//imwrite("d:/test.png",color_map_inv);
	cv::waitKey();
	delete planner_;
	delete cost_map;
	return -1;
}