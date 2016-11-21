#ifndef LOG_H_
#define LOG_H_
#include <standard_includes.h>

#ifndef NSECS_TO_SEC
#define NSECS_TO_SEC 1e-9
#endif 

#ifndef M_TO_CM
#define M_TO_CM 100
#endif

namespace data {
    
    struct odom {
        double x;
        double y;
        double theta;
        unsigned long long int t; //time
    };

    struct lidar {
        unsigned long long int t; //time
        double angle_min; //minimum angle
        double angle_max; //maximum angle
        double angle_increment; //resolution

        double range_min; //minimum range
        double range_max; //maximum range

        double time_increment; //time resolution

        int scan_size; //Number of scans
        std::vector<int>* ranges; //ranges
        std::vector<double>* intensities; //intensities
    };

    class Log{

    public:
        Log(std::string odom_filename, std::string laser_filename);
        lidar* getLidar(double time);
        odom* getOdom(double time);
        int laserCount() {return laserCount_;}
        int odomCount() {return odomCount_;}
        std::vector<unsigned long long int> getTimeStamps() {return time_stamps_;}
        bool isLidar(double time) {return (lidarScans_.count(time) > 0);}
        bool isOdom(double time) {return (odomVals_.count(time) > 0);}
        double getMinAngle() {return angle_min_;}
        double getMaxAngle() {return angle_max_;}
        double getAngleIncrement() {return angle_increment_;}
        double getMinRange() {return range_min_;}
        int getMaxRange() {return range_max_;}
        int getNumScans() {return scan_size_;}
    private:
        std::map<double, lidar*> lidarScans_;
        std::map<double, odom*> odomVals_;
        std::vector<unsigned long long int> time_stamps_;
    	int laserCount_;
        int odomCount_;
        int max_range_;
        int scan_size_;

        double angle_min_;
        double angle_max_;
        double angle_increment_;
        
        double range_min_;
        double range_max_;

        double time_increment_;

    };
}
#endif