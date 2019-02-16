

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;
using std::normal_distribution;
using std::uniform_int_distribution;
using std::uniform_real_distribution;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   *  Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   *  Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  num_particles = 100;  //  Set the number of particles
  std::default_random_engine gen;
  
  //create normal distribution for x,y and theta
  normal_distribution<double> dist_x(x,std[0]);
  normal_distribution<double> dist_y(y,std[1]);
  normal_distribution<double> dist_theta(theta,std[2]);
  
  //generate particles
  for(int i = 0; i<num_particles; ++i) {
    Particle p;
    p.id=i;
    p.x=dist_x(gen);
    p.y=dist_y(gen);
    p.theta=dist_theta(gen);
    p.weight=1.0;
    
    particles.push_back(p);
   
  }
  is_initialized=true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   *  Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */
  //define normal distribution for sensor noise
  std::default_random_engine gen;
  normal_distribution<double> N_x(0,std_pos[0]);
  normal_distribution<double> N_y(0,std_pos[1]);
  normal_distribution<double> N_theta(0,std_pos[2]);
   
  //predict new state
  for (auto &p:particles) {
    if (fabs(yaw_rate)<0.00001) {
      p.x+=velocity*delta_t*cos(p.theta);
      p.y+=velocity*delta_t*sin(p.theta);
    }
    else {
      p.x+=velocity/yaw_rate*(sin(p.theta+yaw_rate*delta_t)-sin(p.theta));
      p.y+=velocity/yaw_rate*(cos(p.theta)-cos(p.theta+yaw_rate*delta_t));
      p.theta+=yaw_rate*delta_t;
    }
    //add sensor noise
    p.x+=N_x(gen);
    p.y+=N_y(gen);
    p.theta+=N_theta(gen);
  }

}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   *  Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */
  
  for (auto &obs:observations) {
    //init minimum distance to maxmum possible
    double min_dist=std::numeric_limits<double>::max();
    
    //current prediction
    for (auto &pred:predicted) {
      //get distance between current/predicted landmarks
      double distance = dist(obs.x, obs.y, pred.x, pred.y);
      //find the predicted landmark nearest the current observed landmark
      if (distance < min_dist) {
        min_dist=distance;
        obs.id=pred.id;
      }
    }
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   *  Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */
  for(auto& p: particles){
    p.weight = 1.0;

    //collect valid landmarks
    vector<LandmarkObs> predictions;
    for(const auto& lm: map_landmarks.landmark_list){
      double distance = dist(p.x, p.y, lm.x_f, lm.y_f);
      if( distance < sensor_range){ // if the landmark is within the sensor range, save it to predictions
        predictions.push_back(LandmarkObs{lm.id_i, lm.x_f, lm.y_f});
      }
    }

    //convert observations coordinates from vehicle to map
    vector<LandmarkObs> observations_map;
    
    for(const auto& obs: observations){
      LandmarkObs map;
      map.x = obs.x * cos(p.theta) - obs.y * sin(p.theta) + p.x;
      map.y = obs.x * sin(p.theta) + obs.y * cos(p.theta) + p.y;
      observations_map.push_back(map);
    }

    // find landmark index for each observation
    dataAssociation(predictions, observations_map);

    // calculaye the particles' weight:
    for(const auto& obs_m: observations_map){

      Map::single_landmark_s landmark = map_landmarks.landmark_list.at(obs_m.id-1);
      double gauss_norm=1/(2 * M_PI * std_landmark[0] * std_landmark[1]);
      double exponent = pow(obs_m.x - landmark.x_f, 2)/(2 * pow(std_landmark[0], 2))+pow(obs_m.y - landmark.y_f, 2) / (2 * pow(std_landmark[1], 2));
      double weight=gauss_norm*exp(-exponent);
      p.weight *= weight;
    }

    weights.push_back(p.weight);

  }


}

void ParticleFilter::resample() {
  /**
   * Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
  vector <Particle> new_particles;
  //new_particles.resize(num_particles);
  // get all current weights
  vector <double> weights;
  for (auto &p:particles) {
    weights.push_back(p.weight);
  }
  
  // generata random starting random index for resampling wheel
  std::default_random_engine gen;
  uniform_int_distribution <int> uniintdist(0,num_particles-1);
  auto index = uniintdist(gen);
  
  // max weight
  double mw = *max_element(weights.begin(),weights.end());
  
  // uniform random distribution(0,mw)
  uniform_real_distribution <double> unirealdist(0.0,mw);
  // spin the resample wheel
  double beta=0.0;
  for (int i = 0; i < num_particles; ++i) {
    beta+=unirealdist(gen)*2.0;
    while (beta > weights[index]){
      beta -= weights[index];
      index = (index+1)%num_particles;
    }
    
    new_particles.push_back(particles[index]);
    
  }
  
  particles=new_particles;
  
 
}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}