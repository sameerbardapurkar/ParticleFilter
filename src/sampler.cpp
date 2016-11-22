#include <sampler.h>

namespace sp{

Sampler::Sampler(Map *map, int num_particles)
:
map_(map),
num_particles_(num_particles),
ang_res_(1)
{
	free_space_ = map_->getFreeSpace();
	constructFullFreeSpace();
}

void Sampler::constructFullFreeSpace(){

	for(int i = 0; i < free_space_.size(); i++){
    // for(int j = -180 ; j < 180; j+=ang_res_){
		for(int j = 90){
			double ang = j*M_PI/180;
			full_free_space_.push_back(std::make_tuple(free_space_[i].first, free_space_[i].second, ang));
		}
	}
}

void Sampler::sampleUniform(std::vector<ps::ParticleState>& ps, int max_range, double angle_min, 
            double angle_max, double angle_increment, int num_scans) {

	std::random_device rd;
  std::mt19937 gen(rd());

	std::uniform_int_distribution<int> dist(0, full_free_space_.size());

	for (int i = 0; i < num_particles_; i++){	
  	int num = dist(gen);
  	ParticleState p_state;
  	p_state.x(std::get<0>(full_free_space_[num]));
  	p_state.y(std::get<1>(full_free_space_[num]));
  	p_state.theta(std::get<2>(full_free_space_[num]));
    p_state.setRanges();
    p_state.setRayTips(max_range, angle_min, angle_max,
                       angle_increment, num_scans);
    // std::cout << "Sampled point " << p_state.x() << " " << p_state.y() << " " << p_state.theta() << std::endl; 

  	p_state.weight(1.0);

  	ps.push_back(p_state);
	}
}

void Sampler::importanceResample(std::vector<ps::ParticleState> &ps, double resampling_randomization, int iter)
{

    std::vector <double> input_weights;

    for(std::vector<ps::ParticleState>::iterator it = ps.begin(); it != ps.end(); ++it) {
      input_weights.push_back(it->weight());
    }
    /*double min_weight = *std::min_element(input_weights.begin(), input_weights.end());
    for(int i = 0; i <input_weights.size(); i++) {
      input_weights[i] += abs(min_weight);
      //cout<<"Modded weight: "<<input_weights[i]<<endl;
    }*/
    std::random_device rd;
    std::mt19937 gen(rd());
    std::vector<int> particle_count(ps.size(), 0);
    std::discrete_distribution<> d(input_weights.begin(), input_weights.end());
    int max_limit = ps.size();//2 + ps.size()*exp(-0.03*iter);
    cout<<max_limit<<endl;
    std::vector<ps::ParticleState> resampled_particles;

    for(int n=0; n < ps.size(); ++n) {
      int chosen_particle;
      while(1) {
        chosen_particle = d(gen);
        particle_count[chosen_particle] +=1;
        if(particle_count[chosen_particle] <= max_limit) {
          break;
        }
      }
      ps::ParticleState particle = ps[chosen_particle];
      double random = (double)rand() / (double)RAND_MAX;
      particle.x((1 + resampling_randomization*(random-0.5))*particle.x());
      random = (double)rand() / (double)RAND_MAX;
      particle.y((1 + resampling_randomization*(random-0.5))*particle.y());
      random = (double)rand() / (double)RAND_MAX;
      particle.theta((1 + resampling_randomization*(random-0.5))*particle.theta());
      particle.weight(1.0); 
      resampled_particles.push_back(particle);   
    }
    ps = resampled_particles;
}


void Sampler::importanceCombResample(std::vector<ps::ParticleState> &ps, int comb_dist) 
{
    std::vector <double> input_weights;
    std::vector <double> comb_weights;

    for(std::vector<ps::ParticleState>::iterator it = ps.begin(); it != ps.end(); ++it) {
      input_weights.push_back(it->weight());
    }

    double min_weight = *std::min_element(input_weights.begin(), input_weights.end());
    for(int i = 0; i <input_weights.size(); i++) {
      input_weights[i] += abs(min_weight);
      //cout<<"Modded weight: "<<input_weights[i]<<endl;
    }

    std::stable_sort(input_weights.begin(), input_weights.end());
    std::reverse(input_weights.begin(), input_weights.end());
    
    for(int i = 0; i < input_weights.size(); i++){
      if(i % comb_dist == 0)
        comb_weights.push_back(input_weights[i]);
    }

    std::random_device rd;
    std::mt19937 gen(rd());

    std::discrete_distribution<> d(comb_weights.begin(), comb_weights.end());

    std::vector<ps::ParticleState> resampled_particles;

    for(int n=0; n < ps.size(); ++n) {
      resampled_particles.push_back(ps[d(gen)]);
    }
    ps = resampled_particles;
}

void Sampler::lowVarianceResample(std::vector<ps::ParticleState> &ps, int comb_dist, double resampling_threshold,
                                  int max_range, double angle_min, double angle_max, double angle_increment,
                                  int num_scans)
{

  std::vector <double> input_weights;
  double wt = 0;
  double max_weight = SMALL_VALUE;
  for(std::vector<ps::ParticleState>::iterator it = ps.begin(); it != ps.end(); ++it) {
    input_weights.push_back(it->weight());
    wt += it->weight();
    if(wt > max_weight) {
      max_weight = wt;
    }
  }
  cout<<"Max weight is"<<max_weight<<endl;
  if(max_weight < resampling_threshold) {
    sampleUniform(ps, max_range, angle_min, angle_max,
                    angle_increment, num_scans);
    cout<<"lost localization, resampling "<<max_weight<< endl;
    return;
  }
  std::transform (input_weights.begin (), input_weights.end (), input_weights.begin (),
                 std::bind1st (std::multiplies <double> () , 1/wt)) ;

  std::vector<ps::ParticleState> resampled_particles;

  int s = ps.size();
  double max_r = 1/s;

  double r = static_cast <double> (rand()) / (static_cast <double> (RAND_MAX));

  // r = r/s;

  int i = 0;

  double w = input_weights[i];

  for(int m = 0; ;m++){

      double u = r + m;
      while(u > s*w){
        i++;

        if(i > input_weights.size())
          i = 1;

        w += input_weights[i];
      }
      // std::cout << i << std::endl;
 
      if(ps[i].weight() < 10 && m < 10*ps.size())
        continue;

      resampled_particles.push_back(ps[i]);

      if(resampled_particles.size() == s)
        break;
  }

  ps = resampled_particles;

}

}
