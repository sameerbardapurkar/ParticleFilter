#include <util_functions.h>

namespace utils{

UtilFunctions::UtilFunctions()
{

}

Eigen::Matrix3d UtilFunctions::getRotationMatrix(double roll, double yaw, double pitch){

	Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitZ());
	Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());

	Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;

	Eigen::Matrix3d rotationMatrix = q.matrix();

	return rotationMatrix;
}

Eigen::Vector2f UtilFunctions::rotateVector(Eigen::Vector2f vec, double yaw){

	Eigen::Rotation2Df t(yaw);
    Eigen::Matrix2f rot_mat = t.toRotationMatrix();
    return rot_mat*vec;
}

Eigen::Vector2f UtilFunctions::transformVector(Eigen::Vector2f vec, double x, double y, double yaw){

	Eigen::Rotation2Df t(yaw);

	Eigen::Vector2f trans(x, y);
    Eigen::Matrix2f rot_mat = t.toRotationMatrix();
    return rot_mat*vec + trans;
}

}