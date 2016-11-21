#ifndef UTIL_FUNCTIONS_H
#define UTIL_FUNCTIONS_H

#include <standard_includes.h>
#include <eigen3/Eigen/Geometry> 

namespace utils{
	
	class UtilFunctions{
		
		public:

			UtilFunctions();
			Eigen::Matrix3d getRotationMatrix(double roll, double yaw, double pitch);
			Eigen::Vector2f rotateVector(Eigen::Vector2f vec, double yaw);
			Eigen::Vector2f transformVector(Eigen::Vector2f vec, double x, double y, double yaw);
	};


} //namespace

#endif