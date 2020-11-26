
#include "tracker.hpp"
using namespace std;
tracker::tracker(){}
/*
void tracker::activateTrack(int trackNum,double t0,Eigen::VectorXd& x0){
KFT[trackNum].init(t0,x0);
activeTracks++
}

void tracker::deactivateTrack(int trackNum){
KFT[trackNum].deactivate();
activeTracks--
}
*/

void tracker::createTrack( double t0, const Eigen::VectorXd& x0){
  if (activeTracks < MAXTRACKS) {
    double dt = 1.0/30; // Time step
    int n = 3; // Number of states
    int m = 1; // Number of measurements

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

    std::cout << "A: \n" << A << std::endl;
    std::cout << "C: \n" << C << std::endl;
    std::cout << "Q: \n" << Q << std::endl;
    std::cout << "R: \n" << R << std::endl;
    std::cout << "P: \n" << P << std::endl;

    // Construct the filter
    KalmanFilter kf(dt,A, C, Q, R, P);
    kf.init(t0, x0);
    KFT.push_back(kf);
    activeTracks++;
  }


  else return;

}


void tracker::deleteTrack(int position){

  KFT.erase(KFT.begin()+position);
  activeTracks--;

  //return KFT;
}


void tracker::updateTracks(int trackID, Eigen::VectorXd y){
  //  Eigen::MatrixXd Plocal = 0,0,0,0,0,0,0,0,0;
    //std::cout<<KFT[trackID].x_hat<< std::endl; // error output
    //KFT[trackID].update(y);


    //cant "live" update vector, have to use std::vector.assign
    //make local varibles , assign all new values to them then update vector


    KFT[trackID].x_hat_new = KFT[trackID].A * KFT[trackID].x_hat;
    //std::cout<<"X_hat_new:"<<KFT[trackID].x_hat_new<< std::endl; // error output

    KFT[trackID].P = KFT[trackID].A*KFT[trackID].P*KFT[trackID].A.transpose() + KFT[trackID].Q;
    //std::cout<<"P:"<<KFT[trackID].P<< std::endl; // error output

    KFT[trackID].K = KFT[trackID].P*KFT[trackID].C.transpose()*(KFT[trackID].C*KFT[trackID].P*KFT[trackID].C.transpose() + KFT[trackID].R).inverse();
    //std::cout<<"K:"<<KFT[trackID].K<< std::endl; // error output

    KFT[trackID].x_hat_new += KFT[trackID].K * (y - KFT[trackID].C*KFT[trackID].x_hat_new);
    //std::cout<<"X_hat_new:"<<KFT[trackID].x_hat_new<< std::endl; // error output

//Problem starts here, assigning new value to old varibles in vector
//std::cout << "P: \n" << KFT[trackID].P << std::endl;
//std::cout << "I: \n" << KFT[trackID].I << std::endl;
//std::cout << "K: \n" << KFT[trackID].K << std::endl;
//std::cout << "C: \n" << KFT[trackID].C << std::endl;

    KFT[trackID].P = (KFT[trackID].I - KFT[trackID].K*KFT[trackID].C)*KFT[trackID].P;
    //std::cout<<"P:"<<KFT[trackID].P<< std::endl; // error output

    KFT[trackID].x_hat = KFT[trackID].x_hat_new;
    //std::cout<<"x_hat:"<<KFT[trackID].x_hat<< std::endl; // error output

    KFT[trackID].t += KFT[trackID].dt;
    //std::cout<<"t:"<<KFT[trackID].t<< std::endl; // error output



}

Eigen::VectorXd tracker::estimateTracks(int trackID) {
  //KalmanFilter* pKF = KFT.data();
  //Eigen::VectorXd x_hatJoined;
  Eigen::VectorXd x_hatOut;
  x_hatOut = KFT[trackID].state();
  //Eigen::VectorXd x_hat =  *pKF[trackID].state();

  return x_hatOut;
}
void tracker::OutputTracker(int trackID, Eigen::VectorXd& y,int i){
  int t;
  std::cout << "t = " << t << ", " << "x_hat[0]: " << KFT[trackID].state().transpose() << std::endl;

  std::cout << "t = " << t << ", " << "y[" << i << "] = " << y.transpose()
    << ", x_hat[" << i << "] = " << KFT[trackID].state().transpose() << std::endl;


}
