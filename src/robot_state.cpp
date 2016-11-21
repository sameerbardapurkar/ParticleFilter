#include <robot_state.h>

namespace rs{

RobotState::RobotState(double x, double y, double theta)
:
x_(x),
y_(y),
theta_(theta)
{
	utils_ = new utils::UtilFunctions();
}

void RobotState::x(double x){
	x_ = x;
}

void RobotState::y(double y){
	y_ = y;
}

void RobotState::theta(double theta){
	theta_ = theta;
}

double RobotState::x(){
	return x_;
}

double RobotState::y(){
	return y_;
}

double RobotState::theta(){
	return theta_;
}

RobotState RobotState::rotate(double theta){

	RobotState trans_state;
	
	Eigen::Vector2f vec(x_, y_);	
	Eigen::Vector2f trans_vec = utils_->rotateVector(vec, theta);

	trans_state.x(trans_vec[0]);
	trans_state.y(trans_vec[1]);
	trans_state.theta(theta_);

	return trans_state;
}

} //namepsace