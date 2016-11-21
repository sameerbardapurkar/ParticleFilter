#ifndef PARTICLE_STATE_H
#define PARTICLE_STATE_H

#include <standard_includes.h>
#include <util_functions.h>

#include <robot_state.h>

using namespace rs;

namespace ps{

	class ParticleState{

	public:

		ParticleState(){}
		ParticleState(double x, double y, double theta, double weight);
		ParticleState(RobotState robot_state, double weight);

		void x(double x);
		void y(double y);
		void theta(double theta);
		void robot_state(RobotState robot_state);
		void weight(double weight);
		double x();
		double y();
		double theta();		
		RobotState robot_state();
		double weight();
		std::vector<int> ranges() { return ranges_;}
		void setRangeVal(int index, int val){ ranges_[index] = val;}
		void setRanges() { ranges_.resize(180,0);}
		void setRayTips(double max_range);
		std::vector<Eigen::Vector2d> getRayTips() {return ray_tips_;}
		ParticleState rotate(double theta);

	private:

		RobotState robot_state_;
		double x_;
		double y_;
		double theta_;
		double weight_;

		utils::UtilFunctions *utils_;

		//Fill in ray ranges from ray-casting here
		std::vector<int> ranges_;

		//End points of rays
		std::vector<Eigen::Vector2d> ray_tips_;

	};
}

#endif