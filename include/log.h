#ifndef LOG_H_
#define LOG_H_
#include <standard_includes.h>
namespace data {
    
    struct odom {
        double x;
        double y;
        double theta;
        double t; //time
    };

    struct lidar {
        double t; //time
        std::vector<int>* ranges; //ranges
    };

    class Log{

    public:
        Log(std::string filename);
        lidar* getLidar(double time);
        odom* getOdom(double time);
        int laserCount() {return laserCount_;}
        int odomCount() {return odomCount_;}
        int getMaxRange() {return max_range_;}
        std::vector<double> getTimeStamps() {return time_stamps_;}
        bool isLidar(double time) {return (lidarScans_.count(time) > 0);}
        bool isOdom(double time) {return (odomVals_.count(time) > 0);}
    private:
        std::map<double, lidar*> lidarScans_;
        std::map<double, odom*> odomVals_;
        std::vector<double> time_stamps_;
    	int laserCount_;
        int odomCount_;
        int max_range_;
    };
}
#endif