#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

#include <standard_includes.h>
#include <util_functions.h>

namespace rs{

	class RobotState{

	public:

		RobotState(){}
		RobotState(double x, double y, double theta);
		
		void x(double x);
		void y(double y);
		void theta(double theta);
		double x();
		double y();
		double theta();
		
		RobotState rotate(double theta);

	private:

		// coordinates in odom frame
		double x_;
		double y_;
		double theta_;

		utils::UtilFunctions *utils_;

	};

} //namespace

#endif