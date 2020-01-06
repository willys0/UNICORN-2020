class PidController
{
public:
	/** @brief Constructor to initialize PID gains and error tolerance.
	*	
	*	@param Kp 	Proportional gain.
	*	@param Ki 	Integral gain.
	*	@param Kd 	Derivative gain.
	*	@param tolerance 	error tolerance.
	*/
	PidController(float Kp, float Ki, float Kd, float tolerance);
	/** @brief Calculate pidterm and control variable
	*	
	*	@param error 	error of output response
	*	@param var 	new input into pid loop	
	*/
	void control(float &var, float error);
	void setLimit(double lower, double upper);
	float limit(float term);

private:
	const float Kp_, Ki_, Kd_, tolerance_;
	float previous_error_;
	float total_error_;
	double lower_limit_, upper_limit_;
};