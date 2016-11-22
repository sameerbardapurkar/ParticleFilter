#include <log.h>

namespace data {
    
    Log::Log(std::string odom_filename, std::string laser_filename) {
    	std::ifstream laser_fin(laser_filename);
        std::ifstream odom_fin(odom_filename);
    	laserCount_ = 0;
    	odomCount_ = 0;
        max_range_ = 0;
        time_stamps_.clear();

        if(laser_fin.good()) {
        	std::string line_raw;
            int count = 0;
            while (getline(laser_fin, line_raw)) {
                count++;
                if(count == 1) {
                    continue; //Ignore the first line
                }
                std::istringstream iss(line_raw);
                std::string token;
                std::vector<std::string> lidar_vals;
                lidar* lidar_val = new lidar;
                while(getline(iss, token, ',')) {
                    lidar_vals.push_back(token);
                }
                
                lidar_val->t = (unsigned long long int) (NSECS_TO_SEC*stoull(lidar_vals[0]));
                lidar_val->angle_min = stod(lidar_vals[4]);
                lidar_val->angle_max = stod(lidar_vals[5]);
                lidar_val->angle_increment = stod(lidar_vals[6]);

                lidar_val->range_min = M_TO_CM*stod(lidar_vals[9]);
                lidar_val->range_max = M_TO_CM*stod(lidar_vals[10]);

                lidar_val->scan_size = (int)((lidar_val->angle_max - 
                                     lidar_val->angle_min)/lidar_val->angle_increment) + 1;

                angle_min_ = lidar_val -> angle_min;
                angle_max_= lidar_val -> angle_max;
                angle_increment_ = lidar_val -> angle_increment;
        
                range_min_ = lidar_val -> range_min;
                range_max_ = lidar_val -> range_max;

                scan_size_ = lidar_val->scan_size;

                lidar_val->ranges = new std::vector<int>;
                lidar_val->ranges->clear();

                lidar_val->intensities = new std::vector<double>;
                lidar_val->intensities->clear();

                //Now fill in the laser ranges
                int offset = 11;
                for(int i = 0; i < lidar_val->scan_size; i++) {
                    lidar_val->ranges->push_back(stod(lidar_vals[offset + i]));
                }
                //Now fill in the laser intensities
                offset = 553;
                for(int i = 0; i < lidar_val->scan_size; i++) {
                    lidar_val->intensities->push_back(stod(lidar_vals[offset + i]));
                }

                //Add the lidar reading to the hash table
                lidarScans_[lidar_val->t] = lidar_val;

                //Add the time step to the vector;
                time_stamps_.push_back(lidar_val->t);
                laserCount_ ++;
                }
            }
            laser_fin.close();
            printf("Read %d laser scans \n", laserCount_);
            unsigned long long int prev_t = 0;
            if(odom_fin.good()) {
                std::string line_raw;
                int count = 0;
                while(getline(odom_fin, line_raw)) {
                    count++;
                    if(count == 1) {
                        continue; //Ignore first line
                    }
                    char data_type = line_raw[0];
                    char* line = (char*) line_raw.c_str();
                    std::istringstream iss(line_raw);
                    std::string token;
                    std::vector<std::string> odom_vals;
                    odom* odom_val = new odom;
                    while(getline(iss, token, ',')) {
                        odom_vals.push_back(token);
                    }
                    odom_val->x = M_TO_CM*stod(odom_vals[5]);
                    odom_val->y = M_TO_CM*stod(odom_vals[6]);
                    odom_val->theta = 2*atan2(stod(odom_vals[10]), stod(odom_vals[11]));
                    odom_val->t = (unsigned long long int) (NSECS_TO_SEC*stoull(odom_vals[0]));
                    if(prev_t > 0) {
                        odomVals_[prev_t]->next_t = odom_val->t;
                    }
                    prev_t = odom_val->t;
                    //Add the time to known time stamps
                    time_stamps_.push_back(odom_val->t);
                    //Add it to the map
                    odomVals_[odom_val->t] = odom_val;
                    odomCount_++;
                }
            }
            odom_fin.close();
            printf("Read %d odom values \n", odomCount_);
                
    }

    lidar* Log::getLidar (double time) {
        
        if(lidarScans_.count(time) <= 0) {
            printf("[ERROR] Wrong access to lidar scans");
            return NULL;
        }
        return lidarScans_[time];
    }

    odom* Log::getOdom (double time) {
         if(odomVals_.count(time) <= 0) {
            printf("[ERROR] Wrong access to odom");
            return NULL;
        }
        return odomVals_[time];
    }

}