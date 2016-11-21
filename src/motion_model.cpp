#include <motion_model.h>

using namespace utils;
using namespace data;

namespace mm{

MotionModel::MotionModel(data::Log *log, double mm_std_xy, double mm_std_theta)
:
std_xy_(mm_std_xy),
std_theta_(mm_std_theta)
{
	utils_ = new UtilFunctions();
	log_ = log;
}

void MotionModel::applyMotionModel(std::vector<ps::ParticleState>& particles,
                                   double time, double next_time){

	odom* odom_val_current = log_->getOdom(time);
	odom* odom_val_next = log_->getOdom(next_time);
	
	double x0 = odom_val_current->x;
	double y0 = odom_val_current->y;
	double th0 = odom_val_current->theta;

	double x1 = odom_val_next->x;
	double y1 = odom_val_next->y;
	double th1 = odom_val_next->theta;

	// Change
	double x_diff = x1 - x0;
	double y_diff = y1 - y0;
	double theta_diff = th1 - th0;

	Eigen::Vector2f diff_vec(x_diff, y_diff);

	Eigen::Vector2f robot_diff_vec = utils_->rotateVector(diff_vec, -th0); //check the angle of rotation

	// the main loop of the motion model
	for(int i = 0; i < particles.size(); i++){
		
		double rot = particles[i].theta();
		Eigen::Vector2f p_diff = utils_->rotateVector(robot_diff_vec, rot);

		Eigen::Vector3f s_p_diff = sampleNormalDist(p_diff, theta_diff);

		particles[i].x(particles[i].x() + s_p_diff(0));
		particles[i].y(particles[i].y() + s_p_diff(1));
		particles[i].theta(particles[i].theta() + s_p_diff(2));

	}
}

Eigen::Vector3f MotionModel::sampleNormalDist(Eigen::Vector2f p, double theta){

	std::random_device rd;
    std::mt19937 gen(rd());
 
    std::normal_distribution<> d1(p(0), std_xy_);
    std::normal_distribution<> d2(p(1), std_xy_);
    std::normal_distribution<> d3(theta, std_theta_);

    Eigen::Vector3f norm_p(d1(gen), d2(gen), d3(gen));

    return norm_p;

}

}