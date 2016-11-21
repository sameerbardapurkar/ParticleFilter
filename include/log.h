#ifndef LOG_H_
#define LOG_H_
#include <standard_includes.h>
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
        int getMaxRange() {return max_range_;}
        std::vector<unsigned long long int> getTimeStamps() {return time_stamps_;}
        bool isLidar(double time) {return (lidarScans_.count(time) > 0);}
        bool isOdom(double time) {return (odomVals_.count(time) > 0);}
    private:
        std::map<double, lidar*> lidarScans_;
        std::map<double, odom*> odomVals_;
        std::vector<unsigned long long int> time_stamps_;
    	int laserCount_;
        int odomCount_;
        int max_range_;
    };
}
#endif