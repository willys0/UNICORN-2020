#include <iostream>
#include <vector>
#include <Eigen/Dense>

#include "kalman.hpp"

void tracking_lidar::createTrack(std::vector<KalmanFilter>& KFtracker, double t0, const Eigen::VectorXd& x0){
  double dt = 1.0/30; // Time step
  int n = 3; // Number of states
  int m = 1; // Number of measurements
  double dt = 1.0/30; // Time step


  Eigen::MatrixXd A(n, n); // System dynamics matrix
  Eigen::MatrixXd C(m, n); // Output matrix
  Eigen::MatrixXd Q(n, n); // Process noise covariance
  Eigen::MatrixXd R(m, m); // Measurement noise covariance
  Eigen::MatrixXd P(n, n); // Estimate error covariance

  // Discrete LTI projectile motion, measuring position only
  A << 1, dt, 0, 0, 1, dt, 0, 0, 1;
  C << 1, 0, 0;

  // Reasonable covariance matrices
  Q << .05, .05, .0, .05, .05, .0, .0, .0, .0;
  R << 5;
  P << .1, .1, .1, .1, 10000, 10, .1, 10, 100;

//  std::cout << "A: \n" << A << std::endl;
//  std::cout << "C: \n" << C << std::endl;
//  std::cout << "Q: \n" << Q << std::endl;
//  std::cout << "R: \n" << R << std::endl;
//  std::cout << "P: \n" << P << std::endl;

  // Construct the filter
  KalmanFilter kf(A, C, Q, R, P);


  kf.init(t0,x0);
  KFtracker.push_back(kf) ;

}


void tracking_lidar::deleteTrack(std::vector<KalmanFilter>& KFtracker, int position){

  KFtracker.erase(position)
}

Eigen::MatrixXd tracking_lidar::updateTrackers(std::vector<KalmanFilter>& KFtracker, Eigen::VectorXd& y, int numTracks ){
KalmanFilter* pKF = kfKFtracker.data();
Eigen::VectorXd x_hat;
//Eigen::VectorXd x_hatJoined;
Eigen::MatrixXd x_hatAll

for (int i = 0; i < numTracks; i++) {
  *pKF[i].update(y);
  x_hat =  *pKF[i].state();
  X_hatAll.row(i) = x_hat
}
return x_hat;
}

Eigen::MatrixXd tracking_lidar::get estimateTracks(std::vector<KalmanFilter>& KFtracker,int numTracks) {
  KalmanFilter* pKF = kfKFtracker.data();
  Eigen::VectorXd x_hat
  //Eigen::VectorXd x_hatJoined;
  Eigen::MatrixXd x_hatAll

  for (int i = 0; i < numTracks; i++) {
  x_hat =  *pKF[i].state();
  X_hatAll.row(i) = x_hat
  }
  return x_hat;
}


void tracking_lidar::tracker(something, something){
int maxTracks= 14;  // Max number of tracks
int numTracks = 0;  // Number of current tracks
int Time = 0;       // guess what? time... current time
Eigen::VectorXd initialState; // for initial state
Eigen::VectorXd y;  // measurements

//Vector of kalman fiters
std::vector<KalmanFilter> KFtracker;



//Create New Track
if(newtracker && numTracks < maxTracks){
  createTrack(KFtracker, time, initialPos,));
  numTracks++
}

//deleteTrack
if(trackinactive){
  deleteTrack(KFtracker, ID );
  numTracks--
}

//update all Tracks and outputs Track estimates
updateTrackers(KFtrackers,y,numTracks);

//outputs Track estimates without updating tracks
estimateTracks(KFtrackers,numTracks);


/**Concatenate Vectors
VectorXd vec_joined(vec1.size() + vec2.size());
vec_joined << vec1, vec2;
*/


}
