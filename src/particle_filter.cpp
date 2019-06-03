/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Original Author: Tiffany Huang
 * Modified By: Danny Bynum, May 31, 2019
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


void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */

  num_particles = 100;

vector<Particle> initial_particles;

//Particle MyParticle;  //debug only

//dwb using random engine to get random value within distribution -this was used in lesson materials
std::default_random_engine dwb_genit1;

// Creating normal distributions for given GPS (x,y,heading) and given std_dev
std::normal_distribution<double> dist_x(x, std[0]);
std::normal_distribution<double> dist_y(y, std[1]);
std::normal_distribution<double> dist_theta(theta, std[2]);

//Loop for creating a particle - randomly select from each normal distribution 
for (int i = 0; i < num_particles; ++i) {

	initial_particles.push_back(Particle()); // dwb construct vector of particles as you go

	//MyParticle.id = i;                             // debug only
	//MyParticle.x = dist_x(dwb_genit);              // debug only
	//MyParticle.y = dist_y(dwb_genit);              // debug only
	//MyParticle.theta = dist_theta(dwb_genit);      // debug only


	initial_particles[i].id = i;
	initial_particles[i].x = dist_x(dwb_genit1);
	initial_particles[i].y = dist_y(dwb_genit1);
	initial_particles[i].theta = dist_theta(dwb_genit1);
	initial_particles[i].weight = 1;


	//dwb debug - print out to terminal to make sure setting it up properly
	if (i < 2) {
		std::cout << "initial_particles " << initial_particles[i].id << " " << initial_particles[i].x << " " << initial_particles[i].y << " "
			<< initial_particles[i].theta << std::endl;
	}

}

particles = initial_particles; //set particles equal to my initial particles - this only runs once at initialization
is_initialized = true;         // initialization complete so set initialized to true
//std::cout << "initialized status: " << is_initialized;
}


//pf.prediction(delta_t, sigma_pos, previous_velocity, previous_yawrate); // call in main.cpp

void ParticleFilter::prediction(double delta_t, double std_pos[],
	double velocity, double yaw_rate) {
	/**
	 * TODO: Add measurements to each particle and add random Gaussian noise.
	 * NOTE: When adding noise you may find std::normal_distribution
	 *   and std::default_random_engine useful.
	 *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	 *  http://www.cplusplus.com/reference/random/default_random_engine/
	 */

	std::default_random_engine dwb_genit2;

	// Creating normal distributions centered about 0 with given standard deviations - to add in noise
	std::normal_distribution<double> pred_noise_x(0, std_pos[0]);
	std::normal_distribution<double> pred_noise_y(0, std_pos[1]);
	std::normal_distribution<double> pred_noise_theta(0, std_pos[2]);


	for (int i = 0; i < num_particles; ++i) {

		//Each particle updated according to physics (time*velocity) separted into x and y components and also
		//changing heading based on heading rate and time
		//also add in normally distributed random noise
		//FIXME - see 21. Explanation of Code walk through - she describes noise differently.
		particles[i].x += (velocity / yaw_rate)*((sin(particles[i].theta*yaw_rate*delta_t)) - sin(particles[i].theta)) + pred_noise_x(dwb_genit2);
		particles[i].y += (velocity / yaw_rate)*(cos(particles[i].theta) - (cos(particles[i].theta*yaw_rate*delta_t))) + pred_noise_y(dwb_genit2);
		particles[i].theta += (yaw_rate * delta_t) + pred_noise_theta(dwb_genit2);

		//dwb debug - print out to terminal to make sure setting it up properly
		if (i < 2) {
			std::cout << "particles updated with prediction and noise " << particles[i].id << " " << particles[i].x << " " << particles[i].y << " "
				<< particles[i].theta << std::endl;
		}
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
	
	//outer for loop is looping through each particle
	for (int i = 0; i < num_particles; ++i) {
		//inner for loop is looping through all the observations made for each time step, which then gets applied to each particle

		particles[i].sense_x.clear(); //using push_back to construct so have to clear it out each time - running this each time step in main
		particles[i].sense_y.clear();
		LandmarkObs temp_measure_associateme;

		for (int j = 0; j < observations.size(); ++j) {
			
			//computing the x,y observation back to map coordinates as applied/viewed from each particle.
			//now each particle has a vector of x,y positions that correspond to map coordinates of where it would place landmarks according to the given observation (which was actually from the car)
			//this will serve as a comparison to how good a particular particle represents the car - given the map and the observations -- since the actual observations are from the ACTUAL car - with some noise/uncertainty
			particles[i].sense_x.push_back(particles[i].x + (cos(particles[i].theta)*observations[j].x) - (sin(particles[i].theta)*observations[j].x));
			particles[i].sense_y.push_back(particles[i].y + (sin(particles[i].theta)*observations[j].x) + (cos(particles[i].theta)*observations[j].y));

			temp_measure_associateme.x = particles[i].sense_x[j];
			temp_measure_associateme.y = particles[i].sense_y[j];

			double temp_dist = 0;
			double prev_temp_dist = 0;

			//FIXME - used 100 here, but this should be size of map - to increment through the full map to get the right association
			for (int k = 0; k < 100; ++k) {

				// DWB - NOTE if using dataAssociation member-Function then - "Predicted" corresponds with the map
				//                                                            "Observations" corresponds to particle
				//
				// FIXME - probably not how you access the map positions but 
				//temp_dist = dist(temp_measure_associateme.x, temp_measure_associateme.y, map_landmarks[j].x, map_landmarks[j].y);

				if (temp_dist < prev_temp_dist) {

					particles[i].associations[j] = map_landmarks[k].id;

				}

				prev_temp_dist = temp_dist;

			}


			//std::cout << particles[i].sense_x[1];

			//dwb debug - print out to terminal to make sure setting it up properly
			if (i < 2) {
				std::cout << "particle" << particles[i].id << " " << "size of new x vector" << particles[i].sense_x.size() << std::endl;
			}
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