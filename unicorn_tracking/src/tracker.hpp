
#ifndef _tracker_H_
#define _tracker_H_



#include <iostream>
#include <vector>
#include <stdio.h>

#include "kalman.hpp"

#define  MAXTRACKS 5  // Max number of tracks



class tracker
{
public:
   tracker();
  std::vector<KalmanFilter> KFT;
  int activeTracks = 0;  // Number of current tracks
  Eigen::VectorXd x_hatOut;


  //void activateTrack(int trackNum,double t0,Eigen::VectorXd& x0);
  //void deactivateTrack(int trackNum);
  void createTrack(double t0, const Eigen::VectorXd& x0);
  void deleteTrack(int position);
  void updateTracks(int trackID, Eigen::VectorXd y);
  Eigen::VectorXd estimateTracks(int trackID);
  void OutputTracker(int trackID, Eigen::VectorXd& y, int i);


private:



};
#endif
