#include "unicorn/pid_controller.h"

PidController::PidController(float Kp, float Ki, float Kd, float tolerance)
	: Kp_(Kp), Ki_(Ki), Kd_(Kd), tolerance_(tolerance)
{
	total_error_ = 0;
	previous_error_ = 0;
}

void PidController::setLimit(double lower, double upper)
{
	lower_limit_ = lower;
	upper_limit_ = upper;
}

float PidController::limit(float term)
{
	if (term < lower_limit_)
	{
		return lower_limit_;
	}
	else if (term > upper_limit_)
	{
		return upper_limit_;
	}
	return term;
}

void PidController::control(float &var, float error)
{
	float pidterm;
	if (error < tolerance_)
	{
		total_error_ = 0;
	}
	total_error_ += error;
	pidterm = error * Kp_ + total_error_ * Ki_ + (error - previous_error_) * Kd_;
	previous_error_ = error;
	var = limit(pidterm);
}
