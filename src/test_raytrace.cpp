#include <standard_includes.h>
#include <map.h>
#include <log.h>
#include <robot_state.h>
#include <particle_state.h>
#include <sampler.h>
#include <motion_model.h>

int main(int argc , char *argv[]){

/*  Map *map = new Map("../data/map/wean.dat", 1700);	
  ps::ParticleState new_particle(3825.0, 3000.0, 0, 5.0);
  //map->visualizeIdealLidar(new_particle);
  //vector <int> lidar_reading;
  map->getIdealLidar(new_particle);

  /*int counter = 0;
  for(std::vector<int>::iterator it = lidar_reading.begin(); it != lidar_reading.end(); ++it) {
    printf("lidar[%d] = %d\n", counter, *it);
    counter++;
  }
  map->visualizeIdealLidar(new_particle);
*/
  return 0;
}
