#include <particle_state.h>

using namespace rs;

namespace ps{

  ParticleState::ParticleState(RobotState robot_state, double weight)
    :
      x_(robot_state.x()),
      y_(robot_state.y()),
      theta_(robot_state.theta()),
      robot_state_(robot_state),
      weight_(weight)
  {
    utils_ = new utils::UtilFunctions();
    //ranges_ = new std::vector<int> ();
    //ranges_->clear();
  }

  ParticleState::ParticleState(double x, double y, double theta, double weight)
    :
      x_(x),
      y_(y),
      theta_(theta),
      weight_(weight)
  {
    utils_ = new utils::UtilFunctions();

    robot_state_.x(x_);
    robot_state_.y(y_);
    robot_state_.theta(theta_);
    //ranges_ = new std::vector<int> ();
    //ranges_->clear();


  }

  void ParticleState::x(double x){
    x_ = x;
  }

  void ParticleState::y(double y){
    y_ = y;
  }

  void ParticleState::theta(double theta){
    theta_ = theta;
  }

  void ParticleState::robot_state(RobotState robot_state){
    robot_state_ = robot_state;
    x_ = robot_state.x();
    y_ = robot_state.y();
    theta_ = robot_state.theta();
  }

  void ParticleState::weight(double weight){
    weight_ = weight;
  }

  double ParticleState::x(){
    return x_;
  }

  double ParticleState::y(){
    return y_;
  }

  double ParticleState::theta(){
    return theta_;
  }

  RobotState ParticleState::robot_state(){
    return robot_state_;
  }

  double ParticleState::weight(){
    return weight_;
  }

  ParticleState ParticleState::rotate(double theta){

    ParticleState trans_state;
    trans_state.robot_state(robot_state_.rotate(theta));
    trans_state.weight(weight_);

    return trans_state;	

  }

  

  void ParticleState::setRayTips(int max_range, double angle_min, double angle_max,
                                 double angle_increment, int num_scans) {
    Eigen::Vector2d origin(62,0);
    ray_tips_.push_back(origin);
    for(int i = 0; i<num_scans; i++) {
      double theta = angle_min + i*angle_increment;
      double x = origin(0) + max_range*cos(theta);
      double y = origin(1) + max_range*sin(theta);
      std::cout<<"x: "<<x<<"\t"<<"y :"<<y<<std::endl;
      Eigen::Vector2d point(x,y);
      ray_tips_.push_back(point);
    }
  }

} //namepsace
