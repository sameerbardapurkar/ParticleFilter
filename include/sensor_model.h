//Sensor model
#ifndef SENSOR_MODEL_H
#define SENSOR_MODEL_H

#include <standard_includes.h>
#include <map.h>
#include <log.h>

namespace sensor_model {
	class LidarModel {

		public:
		//public functions
		
		LidarModel(double max_range, double std_dev, double z_hit,
			       double z_short, double lambda_short, double z_max,
			       double z_rand, double bracket);
		
		void updateWeight(ps::ParticleState* particle,
						  data::lidar* lidar);

		private:

		//Private variables

		//Maximum range of the sensor
		double max_range_;

		//Standard deviation of the measurements
		double std_dev_;

		//Weight for correct hits
		double z_hit_;
		double bracket_;

		//Weight for transient obstacles and the intrinsic parameter to tune
		//it
		double z_short_;
		double lambda_short_;

		//Weight for max_range readings
		double z_max_;

		//Weight for random issues
		double z_rand_;

		//Private functions

		//get p_hit
		double getPHit(int ideal_range, int lidar_range, int norm = 1);

		//get p_short
		double getPShort(int ideal_range, int lidar_range);

		//get p_max
		double getPMax(int lidar_range);

		//get p_rand
		double getPRand(int lidar_range);

		//Map to cache CDFs
		std::map<double, double> normal_cdf_;

	};
}

#endif
