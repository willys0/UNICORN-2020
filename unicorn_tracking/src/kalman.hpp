/**
* Kalman filter header file.
*
* @author: Dhruv Shah, Hayk Martirosyan
* @date: 07/03/2018
*/

#include <Eigen/Dense>


#define STATES 4 // Number of states
#define MEAS_AMOUNT 4 // Number of measurements
#define CONTROL_INPUTS 0 // Number of control inputs

class KalmanFilter {

public:

	/**
	* Create a Kalman filter with the specified matrices.
	*   A - System dynamics matrix
	*   B - Input matrix
	*   C - Output matrix
	*   Q - Process noise covariance
	*   R - Measurement noise covariance
	*   P - Estimate error covariance
	*/  

  /*
	KalmanFilter();*/

	/**
	* Initialize the filter with initial states as zero.
	*/
	void init();

	/**
	* Initialize the filter with a guess for initial states.
	
	void init(const Eigen::VectorXd& x0);
	*/
	/**
	* Update the prediction based on control input.
	*/
	void predict();

	/**
	* Update the estimated state based on measured values.
	*/
	void update(const Eigen::VectorXd& y);

	/**
	* Update the dynamics matrix.
	*/
	void update_dynamics(const Eigen::MatrixXd A);

	/**
	* Update the output matrix.
	*/
	void update_output(const Eigen::MatrixXd C);

	/**
	* Return the current state.
	*/
	//Eigen::VectorXd state() { return x_hat; };


	Eigen::MatrixXd A{STATES, STATES}; // System dynamics matrix
	//Eigen::MatrixXd B{STATES, CONTROL_INPUTS};  // Input control matrix
	Eigen::MatrixXd C{MEAS_AMOUNT, STATES}; // Output matrix
	Eigen::MatrixXd Q{STATES, STATES}; // Process noise covariance
	Eigen::MatrixXd R{MEAS_AMOUNT, MEAS_AMOUNT}; // Measurement noise covariance
	Eigen::MatrixXd P{STATES, STATES}; // Estimate error covariance
	Eigen::MatrixXd P0{STATES, STATES}; // initial error covariance
	Eigen::MatrixXd I{STATES, STATES};  // n-size identity
	Eigen::VectorXd x_hat{STATES};	// Estimated states
	Eigen::VectorXd y{MEAS_AMOUNT}; //
  // Matrices for computation
	//Eigen::MatrixXd A, B, C, Q, R, P, K, P0;


	Eigen::MatrixXd K;
 
	// Is the filter initialized?
	bool initialized = false;

private:


	// System dimensions
	//int m, n, c;


	
};