#include <log.h>

namespace data {
    
    Log::Log(std::string filename) {
    	std::ifstream fin_(filename);
    	laserCount_ = 0;
    	odomCount_ = 0;
        max_range_ = 0;
        time_stamps_.clear();

        if(fin_.good()) {
        	std::string line_raw;
            while (getline(fin_, line_raw)) {
            	char data_type = line_raw[0];
                char* line = (char*) line_raw.c_str();
                std::istringstream iss(line_raw);
                std::string token;
                iss >> token;
                if (data_type == 'O') {
                    std::vector<std::string> odom_val_string;
                    odom* odom_val = new odom;
                    while(iss >> token) {
                        odom_val_string.push_back(token);
                    }
                    odom_val->x = std::stod(odom_val_string[0]);
                    odom_val->y = std::stod(odom_val_string[1]);
                    odom_val->theta = std::stod(odom_val_string[2]);
                    odom_val->t = std::stod(odom_val_string[3]);
                    //Add the time to known time stamps
                    time_stamps_.push_back(odom_val->t);
                    //Add it to the map
                    odomVals_[odom_val->t] = odom_val;
                    odomCount_ ++;
                }

                else if (data_type == 'L') {
                    std::vector<std::string> lidar_val_string;
                    lidar* lidar_val = new lidar;
                    lidar_val->ranges = new std::vector<int>;
                    lidar_val->ranges->clear();
                    odom* odom_val = new odom;
                    while(iss>>token) {
                        lidar_val_string.push_back(token);
                    }
                    lidar_val->t = std::stod(lidar_val_string.back());
                    for(int i = 6; i < lidar_val_string.size()-1; i++) {
                        int range = std::stoi(lidar_val_string[i]);
                        if(range > max_range_) {
                            max_range_ = range;
                        }
                        lidar_val->ranges->push_back
                                         (range);
                    }
                    //Add the odom corresponding to the laser in the map
                    odom_val->x = std::stod(lidar_val_string[0]);
                    odom_val->y = std::stod(lidar_val_string[1]);
                    odom_val->theta = std::stod(lidar_val_string[2]);
                    odom_val->t = lidar_val->t;
                    odomVals_[odom_val->t] = odom_val;

                    //Add the lidar reading to the hash table
                    lidarScans_[lidar_val->t] = lidar_val;

                    //Add the time step to the vector;
                    time_stamps_.push_back(lidar_val->t);

                    laserCount_ ++;
                }
            }
        }
        printf("Read %d laser scans and %d odom vals for %zu time steps total \n",
                laserCount_, odomCount_, time_stamps_.size());
        fin_.close();
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