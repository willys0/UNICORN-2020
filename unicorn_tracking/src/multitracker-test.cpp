


#include "tracker.hpp"


using namespace std;
int main(int argc, char* argv[]) {
  double Time = 0;       // guess what? time... current time
  double dt = 1.0/30; // Time step
  Eigen::VectorXd initialPos(3); // for initial state
  Eigen::VectorXd y(1);  // measurements
  int trackID = 1;
  Eigen::VectorXd x_hatOut1;
  Eigen::VectorXd x_hatOut2;


  std::vector<double> measurements = {
      1.04202710058, 1.10726790452, 1.2913511148, 1.48485250951, 1.72825901034,
      1.74216489744, 2.11672039768, 2.14529225112, 2.16029641405, 2.21269371128,
      2.57709350237, 2.6682215744, 2.51641839428, 2.76034056782, 2.88131780617,
      2.88373786518, 2.9448468727, 2.82866600131, 3.0006601946, 3.12920591669,
      2.858361783, 2.83808170354, 2.68975330958, 2.66533185589, 2.81613499531,
      2.81003612051, 2.88321849354, 2.69789264832, 2.4342229249, 2.23464791825,
      2.30278776224, 2.02069770395, 1.94393985809, 1.82498398739, 1.52526230354,
      1.86967808173, 1.18073207847, 1.10729605087, 0.916168349913, 0.678547664519,
      0.562381751596, 0.355468474885, -0.155607486619, -0.287198661013, -0.602973173813
  }; /// dummy data
initialPos << measurements[0],0,-9.81;

  //Vector of kalman fiters
  tracker multitracker;


  //Create New Track
  while(multitracker.activeTracks < MAXTRACKS){
    multitracker.createTrack(Time, initialPos);
  //  std::cout << "track:" << multitracker.activeTracks << '\n' <<std::endl;

  };

  //deleteTrack
  if(!MAXTRACKS){
    multitracker.deleteTrack(3);
  };

  //update all Tracks and outputs Track estimates
  //updateTracks(KFT,2,y,numTracks);


  //x_hatOut1 = multitracker.KFT[0].state();
  std::cout << "t = " << Time << ", " << "x_hat[0]: " << multitracker.KFT[1].x_hat << std::endl;
  for(int i = 0; i < measurements.size(); i++) {
    Time += dt;
    y << measurements[i];
  //  std::cout<<multitracker.KFT[0].x_hat.transpose << std::endl; // error output
      multitracker.updateTracks(1,y); // update track
       //multitracker.KFT[1].update(y);
      //std::cout << y << '\n';
      //Eigen::VectorXd x_hatOut2 = multitracker.KFT[0].state().transpose;
      //std::cout << multitracker.estimateTracks(0) << '\n'<< std::endl;

      std::cout << "t = " << Time << ", " << "y[" << i << "] = " << y
       << ", x_hat[" << i << "] = " << multitracker.KFT[1].x_hat << std::endl;
      //multitracker.OutputTracker(0,y,i);
  };



  //outputs Track estimates without updating tracks
  //estimateTracks(KFTs,numTracks);


  /**Concatenate Vectors
  VectorXd vec_joined(vec1.size() + vec2.size());
  vec_joined << vec1, vec2;
  */

return 0;
}
