/**
* Implementation of KalmanFilter class.
*
* @author: Dhruv Shah, Hayk Martirosyan
* @date: 07/03/2018
*/

#include <iostream>
#include <iostream>

#include "kalman.hpp"

/*
void KalmanFilter::init(const Eigen::VectorXd& x0) {

	I.setIdentity();
	x_hat = x0;
	P = P0;
	initialized = true;
}*/

void KalmanFilter::init() {

	I.setIdentity();
	x_hat.setZero();
	P = P0;
	initialized = true;
}

void KalmanFilter::predict() {

	if(!initialized) {
		std::cout << "Filter is not initialized! Initializing with trivial state.";
		init();
	}

	//x_hat_new = A*x_hat + B*u;
	x_hat = A*x_hat;
	P = A*P*A.transpose() + Q;
}

void KalmanFilter::update(const Eigen::VectorXd& y) {

	K = P*C.transpose()*(C*P*C.transpose() + R).inverse();
	x_hat += K * (y - C*x_hat);
	P = (I - K*C)*P;
}

void KalmanFilter::update_dynamics(const Eigen::MatrixXd A) {

	this->A = A;
}

void KalmanFilter::update_output(const Eigen::MatrixXd C) {

	this->C = C;
}