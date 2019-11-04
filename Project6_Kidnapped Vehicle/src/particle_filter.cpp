/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

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
using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  
  
  if (is_initialized) {
    return;
  }
  
  // Set the number of particles
  num_particles = 100; 
  
  // Initialize particle positions based on vehicle measurements
  // define the Gaussian distributions
  double std_x = std[0];
  double std_y = std[1];
  double std_theta = std[2];
  
  normal_distribution<double> dist_x(x, std_x);
  normal_distribution<double> dist_y(y, std_y);
  normal_distribution<double> dist_theta(theta, std_theta);
  
  default_random_engine gen;

  particles.resize(num_particles);
  for ( int i=0; i< num_particles; i++){
    particles[i].id = i ;
    particles[i].x = dist_x(gen);
    particles[i].y = dist_y(gen);
    particles[i].theta = dist_theta(gen);
    particles[i].weight = 1.0;
  }
  
  // initialization finish
  is_initialized = true;
  
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */
   
  default_random_engine gen;
  
  // generate random Gaussian noise
  std::normal_distribution<double> std_x(0, std_pos[0]);
  std::normal_distribution<double> std_y(0, std_pos[1]);
  std::normal_distribution<double> std_theta(0, std_pos[2]);
  
  // relocate the particle positions based on vehicle measurement
  for (int i = 0; i < num_particles; i++) {
    if ( fabs(yaw_rate) < 0.0001 ) {
      particles[i].x += velocity * delta_t * cos(particles[i].theta);
      particles[i].y += velocity * delta_t * sin(particles[i].theta);
    }
    else {
      particles[i].x += velocity / yaw_rate * (sin(particles[i].theta + yaw_rate*delta_t) - sin(particles[i].theta));
      particles[i].y += velocity / yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate*delta_t));
      particles[i].theta += yaw_rate * delta_t;
    }
    
    // add noise to predicted particle locations
    particles[i].x += std_x(gen);
    particles[i].y += std_y(gen);
    particles[i].theta += std_theta(gen);
  }
  
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */
	
  	// predicted is the vector for all the landmarks, since anyone of them can be the predicted landmark for a particle
  	// observations is the measurement vector from the vehicle
  	for (unsigned int i = 0; i < observations.size(); i++) {
      // get the current observation
      LandmarkObs obs = observations[i];
      // initialize the minimum distance and landmark id
      double min_dist = numeric_limits<double>::max();
      int map_id = -1;
      // compare obs to each landmark, find the landmark that is closest to obs
      for (unsigned int j = 0; j< predicted.size(); j++) {
        //get the current landmark
        LandmarkObs ldm = predicted[j];
        // get distance between obs and ldm
        double xDist = obs.x - ldm.x;
        double yDist = obs.y - ldm.y;
        double Dist = xDist * xDist + yDist * yDist ;
        if (Dist < min_dist) {
          min_dist = Dist;
          map_id = ldm.id;
        }
      }
      // after looping all the landmarks, assign one landmark to observation[i]
      observations[i].id = map_id;
    }
  
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
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

  // update the weight for each particle
  for ( int i = 0; i < num_particles; i++ ) {
    //-- step 1: get one particle
    double par_x = particles[i].x;
    double par_y = particles[i].y;
    double par_theta = particles[i].theta;
    
    //-- step 2: find the landmarks to be considered
    vector<LandmarkObs> landmark_set;
    // only consider the square area of sensor_range around the particle 
    for ( unsigned int j=0; j < map_landmarks.landmark_list.size(); j++ ) {
      float lm_x = map_landmarks.landmark_list[j].x_f;
      float lm_y = map_landmarks.landmark_list[j].y_f;
      int lm_id = map_landmarks.landmark_list[j].id_i;
      
      if ( fabs(lm_x-par_x) <= sensor_range && fabs(lm_y-par_y)<= sensor_range ) {
        landmark_set.push_back(LandmarkObs{ lm_id, lm_x, lm_y });
      }
    }
    
    //-- step 3: observations, transform vehicle observations to map coordinate
    vector<LandmarkObs> transformed_obs;
    for ( unsigned int j=0; j < observations.size(); j++ ) {
      double Tobs_x = cos(par_theta)*observations[j].x - sin(par_theta)*observations[j].y + par_x;
      double Tobs_y = sin(par_theta)*observations[j].x + cos(par_theta)*observations[j].y + par_y;
      transformed_obs.push_back(LandmarkObs{ observations[j].id, Tobs_x, Tobs_y }); 
    }
    
    //-- step 4: associate, find which landmark each TOBS associates
    dataAssociation(landmark_set, transformed_obs);
    
    //-- step 5: update weight, evaluate how close each TOBS is to the landmark
    particles[i].weight = 1.0;
    double TOBS_x, TOBS_y;
    float LM_x, LM_y;
    // for each TOBS, find the associated landmark
    for (unsigned int j=0; j < transformed_obs.size(); j++ ) {
      TOBS_x = transformed_obs[j].x;
      TOBS_y = transformed_obs[j].y;
      int tar_id = transformed_obs[j].id;
      
      // search for the landmark with tar_id
      for ( unsigned int k=0; k < landmark_set.size(); k++ ) {
        if ( landmark_set[k].id == tar_id ) {
          LM_x = landmark_set[k].x;
          LM_y = landmark_set[k].y;
        }
      }
      
      // calculate the multivariate Gaussian
      double std_x = std_landmark[0];
      double std_y = std_landmark[1];
      double obs_w = ( 1/(2*M_PI*std_x*std_y)) * exp( -( pow(LM_x-TOBS_x,2)/(2*pow(std_x, 2)) + (pow(LM_y-TOBS_y,2)/(2*pow(std_y, 2))) ) );
      
      // calculate the total probability for all the observations
      particles[i].weight *= obs_w;
      
    }
    
  }
  
}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
  default_random_engine gen;
  vector<Particle> Par_resamp;

  // get all of the particle weights
  vector<double> weights;
  for (int i = 0; i < num_particles; i++) {
    weights.push_back(particles[i].weight);
  }
  // get max weight
  double max_weight = *max_element(weights.begin(), weights.end());
  // uniform random distribution 
  uniform_real_distribution<double> unirealdist(0.0, max_weight);

  // generate random starting index 
  uniform_int_distribution<int> uniintdist(0, num_particles);
  auto index = uniintdist(gen);

  double beta = 0.0;
  // resampling process
  for (int i = 0; i < num_particles; i++) {
     beta += unirealdist(gen) * 2.0;
    while (beta > weights[index]) {
      beta -= weights[index];
      index = (index + 1) % num_particles;
    }
    Par_resamp.push_back(particles[index]);
  }

  particles = Par_resamp;
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