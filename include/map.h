#pragma once
#include <standard_includes.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <log.h>

#include <particle_state.h>

using namespace cv;
using namespace std;
using namespace ps;

class Map{

  public:

    Map(std::string filename, double max_range);
    Map(Map* map);
    void readMap(std::string file);
    void printMap();
    void displayMap();
    Map *getMap(){
      return this;
    }
    Mat getGrid(){
      return grid;
    }
    std::vector<std::pair<double, double>> getFreeSpace(){
      return free_space_;
    }

    void visualizeParticles(vector<ParticleState>* particle_list, int color);
    void visualizeRobot(vector<ParticleState>* particle_list, int color, double time, int iter);
    void visualizePoints(vector<pair<int,int>>* points_list);
    void visualizeRayTrace(Mat& grid_rgb, ParticleState *particle, vector<pair<int,int>>* points_list,
        int r, int g, int b);

    vector<pair<int,int>> interpolate(int x1, int y1, int x2, int y2);
    void visualizeIdealLidar(ParticleState p);
    void getIdealLidar(ParticleState* p);
    void getIdealLidarVis(ParticleState* p, data::lidar* lidar);
    void interpolate1(Point p1, Point p2, ParticleState* p, int index);

  private:

    int size_x;
    int size_y;
    int res;
    int grid_size;
    Mat grid;
    Mat grid_disp_;
    int rangemax;
    float lidar_xoffset;
    std::vector<std::pair<double, double>> free_space_;
};
