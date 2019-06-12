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

  num_particles = 5000;

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
	if (i < 5) {
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
	double yaw_rate_check = double(1E-3);

	//std::cout << "Yaw Rate < " << yaw_rate <<" fabs(yaw_rate): "<< fabs(yaw_rate) << std::endl;


	if (fabs(yaw_rate) < yaw_rate_check) {
		std::cout << "Yaw Rate < "<< yaw_rate_check<<" , Will Use Linear Model" << std::endl;
	}

	// Creating normal distributions centered about 0 with given standard deviations - to add in noise
	//std::normal_distribution<double> pred_noise_x(0, std_pos[0]);
	//std::normal_distribution<double> pred_noise_y(0, std_pos[1]);
	//std::normal_distribution<double> pred_noise_theta(0, std_pos[2]);


	std::default_random_engine dwb_genit2;

	for (int i = 0; i < num_particles; ++i) {

		//Each particle updated according to physics (time*velocity) separted into x and y components and also
		//changing heading based on heading rate and time
		//also add in normally distributed random noise
		//FIXME - see 21. Explanation of Code walk through - she describes noise differently.


		///////////////////////////////////////////////////////////////////////////////////
		///////////////////////////////////////////////////////////////////////////////////
		///////////////////////////////////////////////////////////////////////////////////
		
		//June-11 could do some t-shoot here to see how these distributions are getting generated - specificly related to updating/changing movement_x
		
		double movement_x = 0;
		double movement_y = 0;
		double movement_theta = 0;
		

		if (fabs(yaw_rate) > yaw_rate_check) {

			movement_x = (velocity / yaw_rate)*
				(sin(particles[i].theta + (yaw_rate*delta_t)) - sin(particles[i].theta));

			movement_y = (velocity / yaw_rate)*
				(cos(particles[i].theta) - cos(particles[i].theta + (yaw_rate*delta_t)));

			movement_theta = (yaw_rate * delta_t);

		}
		else {
			movement_x = velocity*delta_t*cos(particles[i].theta);

			movement_y = velocity*delta_t*sin(particles[i].theta);

			movement_theta = yaw_rate_check;

		}


		//June-11, why does it seem to not do well with matching to sensor measurements which are ABOVE the car??  something about how doing the transformation??
		std::normal_distribution<double> meas_with_noise_x(movement_x, std_pos[0]*1);    //June-11 was using *6, *6, *1 here
		std::normal_distribution<double> meas_with_noise_y(movement_y, std_pos[1]*1);
		std::normal_distribution<double> meas_with_noise_theta(movement_theta, std_pos[2]*1);


		particles[i].x += meas_with_noise_x(dwb_genit2);

		particles[i].y += meas_with_noise_y(dwb_genit2);

		particles[i].theta += meas_with_noise_theta(dwb_genit2);

		///////////////////////////////////////////////////////////////////////////////////
		///////////////////////////////////////////////////////////////////////////////////
		///////////////////////////////////////////////////////////////////////////////////

				
		//particles[i].x += (velocity / yaw_rate)*
		//				  (sin(particles[i].theta + (yaw_rate*delta_t)) - sin(particles[i].theta)) + 
		//				  pred_noise_x(dwb_genit2);
		//
		//particles[i].y += (velocity / yaw_rate)*
		//				  (cos(particles[i].theta) - cos(particles[i].theta + (yaw_rate*delta_t))) +
		//				  pred_noise_y(dwb_genit2);
		//
		////leaving incorrect formula in place just for comparison purposes, but commenting out
		////particles[i].y += (velocity / yaw_rate)*(cos(particles[i].theta) - (cos(particles[i].theta*yaw_rate*delta_t))) + pred_noise_y(dwb_genit2);
		//
		//particles[i].theta += (yaw_rate * delta_t) + pred_noise_theta(dwb_genit2);

		
		
		
		//debug to check how much pred_noise is getting added...may need to think about this some more
		//particles[i].x += velocity*delta_t*cos(particles[i].theta) + pred_noise_x(dwb_genit2);
		//particles[i].y += velocity * delta_t*sin(particles[i].theta) + pred_noise_y(dwb_genit2);
		//particles[i].theta += velocity * delta_t + pred_noise_theta(dwb_genit2);

		//dwb debug - print out to terminal to make sure setting it up properly
		if (i < 0) {
			std::cout << "Prediction Step/Function: " << particles[i].id << " " << particles[i].x << " " << particles[i].y << " "
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

	//LandmarkObs temp_measure_associateme;

	//for (int k = 0; k < map.size(); ++k) {

	//	// DWB - NOTE if using dataAssociation member-Function then - "Predicted" corresponds with the map
	//	//                                                            "Observations" corresponds to particle
	//	//
	//	// FIXME - probably not how you access the map positions but 
	//	//temp_dist = dist(temp_measure_associateme.x, temp_measure_associateme.y, map_landmarks[j].x, map_landmarks[j].y);

	//	temp_measure_associateme.x = particles[i].sense_x[j];
	//	temp_measure_associateme.y = particles[i].sense_y[j];

	//	double temp_dist = 0;
	//	double prev_temp_dist = 0;

	//	if (temp_dist < prev_temp_dist) {

	//		particles[i].associations[j] = map_landmarks[k].id;

	//	}
	//}
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
	

	double accumulate_part_weights = 0;

	//outer for loop is looping through each particle
	for (int i = 0; i < num_particles; ++i) {
		

		particles[i].sense_x.clear(); //using push_back to construct so have to clear it out each time - running this each time step in main
		particles[i].sense_y.clear();
		particles[i].associations.clear();
		
		LandmarkObs temp_measure_associateme; // make a temp struct to store transformed x,y particle point of view observations
		
		//inner for loop is looping through all the observations made for each time step, which then gets applied to each particle
		for (int j = 0; j < observations.size(); ++j) {

			//computing the x,y observation back to map coordinates as applied/viewed from each particle.
			//now each particle has a vector of x,y positions that correspond to map coordinates of where it would place landmarks according to the given observation (which was actually from the car)
			//this will serve as a comparison to how good a particular particle represents the car - given the map and the observations -- since the actual observations are from the ACTUAL car - with some noise/uncertainty
			particles[i].sense_x.push_back(particles[i].x + (cos(particles[i].theta)*observations[j].x) - (sin(particles[i].theta)*observations[j].x));
			particles[i].sense_y.push_back(particles[i].y + (sin(particles[i].theta)*observations[j].y) + (cos(particles[i].theta)*observations[j].y));  //June-12 found error here

			temp_measure_associateme.x = particles[i].sense_x[j];
			temp_measure_associateme.y = particles[i].sense_y[j];

			double temp_dist = 0;
			double min_dist = 0;
			//double prev_temp_dist = 0;

			int best_id_index = 0; //the k that I want to bring outside the map landmark for loop
			int best_id = 0; //the k that I want to bring outside the map landmark for loop

			// DWB FIXME - cycling through each map position for each measurement will probably be slow
			// will likely have to implement a check upstream of this to determine map landmarks in sensor range of each particle
			int debug_counter = 0;

			for (int k = 0; k < map_landmarks.landmark_list.size(); ++k) {

				// DWB - clearly don't really need the temp_measure_associateme, but maybe if passing to associate function
				// DWB - this is using the helper function dist to pick the best landmark measurement
				temp_dist = dist(temp_measure_associateme.x, temp_measure_associateme.y, map_landmarks.landmark_list[k].x_f, map_landmarks.landmark_list[k].y_f);
				//std::cout << "temp_dist " << temp_dist << "prev_temp_dist "<<prev_temp_dist<<std::endl;
			
				if (k == 0) {
					min_dist = temp_dist; //initialize min_dist to the first value;
					//best_id_index = k;
				}
				
				if (temp_dist < min_dist) {
					min_dist = temp_dist;   //set a new min_dist
					best_id_index = k;
					
					//debug_counter += 1;
					//std::cout << "debug_counter " << debug_counter << std::endl;
					//update the map association for this particular measurement as distance/fit to matp gets better 
					//in the end only selecting one value for each 'j' here
					

				}

				//setup previous distance for comparison purposes.
				//prev_temp_dist = temp_dist;
				

			}

			//build up the associations vector now that we've found best map landmark (id) fit
			
			best_id = map_landmarks.landmark_list[best_id_index].id_i;
			
			//std::cout << "best id " << best_id << std::endl;
			particles[i].associations.push_back(best_id); 

		}
		
	
		//now actually update the weights for each particle - FINALLY
		//in order to update the weight I need to check "goodness" of each sensor measurement
		//and then combine all of those goodness checks into the weight of the particle
		//vector<double> particle_weight_elements;
		//particle_weight_elements.clear();

		//double measure_goodness_sum = 0;
		//double measure_goodness_product= 1;
		

		for (int ja = 0; ja<particles[i].associations.size(); ++ja){
			//using code from the lesson - Lesson6.Module20: Particle Weights Solution
			// calculate normalization term
		
			//using same names to input into the equation as was used in the lecture
			//otherwise could have just used the values directly below in the calculations
			double sig_x = std_landmark[0];  //brought into this function - given
			double sig_y = std_landmark[1];  //brought into this function - given
			double x_obs = particles[i].sense_x[ja];
			double y_obs = particles[i].sense_y[ja];
			//This one is a little deeper to follow 
			//I want the "closest/associated" ACTUAL map x and y which I get by getting the ".x_f" from the landmark_list AT
			//the id that is most closely associated with the particle measurement -- this is kept in particles.associations list
			
			//using the "id" stored in the associations vector AS an index by subtracting 1
			//this only works because the landmark_list is sequentially ordered as it is read in
			//FIXME a more general approach would be better here - look up the data associated with each ID
			//could do this with a for loop that loops through all of the landmarks and stops when id matches then pick
			//that corresponding x_f and y_f
			double mu_x = map_landmarks.landmark_list[particles[i].associations[ja]-1].x_f;
			
			//double mu_x = map_landmarks.landmark_list[41].x_f;
			double mu_y = map_landmarks.landmark_list[particles[i].associations[ja]-1].y_f;

			//std::cout << "ja "<<ja<< "associations " << particles[i].associations[ja]<<std::endl;
		
			double term1_gauss_norm;
			term1_gauss_norm = 1 / (2 * M_PI * sig_x * sig_y);

			// calculate exponent
			double term2_exponent;
			term2_exponent = (pow(x_obs - mu_x, 2) / (2 * pow(sig_x, 2)))
				+ (pow(y_obs - mu_y, 2) / (2 * pow(sig_y, 2)));

			double sense_goodness = 0;

			sense_goodness = (term1_gauss_norm * exp(-term2_exponent));
			
			///////////////////////////////////////////////////////////////////////////
			///////////////////////////////////////////////////////////////////////////
			///////////////////////////////////////////////////////////////////////////
			///////////////////////////////////////////////////////////////////////////
			//June-11 experimenting with setting a cap on sense_goodness, FIXME probably not what we want to do
			//if (sense_goodness < 0.0000000000000000000001) {
			//	sense_goodness = 0.0000000000000000000001;
			//	if (i < 2) { //print out only for first couple of particles
			//		std::cout << "Weight Too Low, Insert Cap On Sensor Goodness Value" << std::endl;
			//	}
			//}
			///////////////////////////////////////////////////////////////////////////
			///////////////////////////////////////////////////////////////////////////
			///////////////////////////////////////////////////////////////////////////
			///////////////////////////////////////////////////////////////////////////
			///////////////////////////////////////////////////////////////////////////


			// calculate weight using normalization terms and exponent
			//vector<double> particle_weight_elements
			
			
			//measure_goodness_sum += (term1_gauss_norm * exp(-term2_exponent));
			//measure_goodness_product *= (term1_gauss_norm * exp(-term2_exponent));
			//particle_weight_elements.push_back(term1_gauss_norm * exp(-term2_exponent));

			//LEFT OFF HERE on JUNE-4 - PICK UP HERE - not sure why sometimes more than 3 print out in here....hmmmmmm
			//also is the exp of a large negative number close to 0 - is that why this is zero - if so maybe ok? or look at that term2_exponent some more
			//if (ja < 1) {
				//std::cout << "term1 " << term1_gauss_norm <<"term2  " << term2_exponent << std::endl;
				//std::cout << "mu_x " << mu_x << " x_obs " << x_obs << std::endl;
			//}

			//FIXME - this might work but do I really want to use the old particle weight each delta_time step?
			//maybe but not sure this is how it worked in the lesson - maybe just wipe out the old weight and the
			//new weight becomes the multipled together multi-var-gaussian
			if (ja == 0) {
				particles[i].weight = sense_goodness;  //set to current sensor-weight the first time through
			}
			else
			{
				particles[i].weight *= sense_goodness;
			} 

			//particles[i].weight = (term1_gauss_norm * exp(-term2_exponent)); //normalize so that result is between 0 and 1

		}

		//particles[i].weight = measure_goodness_product / measure_goodness_sum; //normalize so that result is between 0 and 1

		

		accumulate_part_weights += particles[i].weight;

		//dwb debug - print out to terminal to make sure setting it up properly
		//this is printing for each particle so it is in the outer for loop here
		if (i < 0) {
		std::cout << "Particle Association Step: "<<"particle " << particles[i].id << " " << "size_x_sense " << particles[i].sense_x.size()<< 
			" size_of_assoc " << particles[i].associations.size()<< std::endl;
		}

		//if (i < 1) {
		//	std::cout << "particle " << particles[i].id << " " << " weight " << particles[i].weight << std::endl;
		//}


	
	}

	//python code for reference
	//def multivar_gauss(x, y, mu_x, mu_y, sdev_x, sdev_y) :
	//	term1 = 1 / (2 * math.pi*sdev_x*sdev_y)
	//	term2a = ((x - mu_x)**2) / (2 * (sdev_x**2))
	//	term2b = ((y - mu_y)**2) / (2 * (sdev_y**2))
	//	term2 = -1 * (term2a + term2b)

	//	return term1 * math.exp(term2)

	//loop through particles one more time to do normalization
	for (int m = 0; m < num_particles; ++m) {
		if (accumulate_part_weights > 0) {
			particles[m].weight = particles[m].weight / accumulate_part_weights; //normalize so that between 0 and 1
		}
		else {
			particles[m].weight = .001;
			if (m < 0) {
				std::cout << "Update Weights Function:  Saved A Divide By Zero - Need to T-Shoot" << std::endl;
			}
		}

		if (m < 0) {
			std::cout << "Update Weights Function: " << "particle " << particles[m].id << " " << "normd weight" << particles[m].weight << std::endl;
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
	std::vector<Particle> resampled_particles;
	
	double temp_find_max_weight = 0;
	//double accumulate_part_weights = 0;

	//loop through particles to find max weight
	for (int i = 0; i < num_particles; ++i) {
		if (particles[i].weight > temp_find_max_weight){
			temp_find_max_weight = particles[i].weight;
		}

	}

	//std::cout << "temp_find_max_weight " << temp_find_max_weight << std::endl;

	//clear out the resampled_particles vector between runs
	resampled_particles.clear();

	std::default_random_engine dwb_genit3(particles[1].sense_x[1]);
	//dwb_genit3.seed(particles[i].sense_x[i]);               //FIXME not sure this is a great "random" seed but might work ok

	std::default_random_engine dwb_genit4(particles[1].sense_x[1]); //June-12 trying to seed dwb_genit4
	//dwb_genit4.seed(particles[i].sense_y[i]);               //FIXME - same note as above


	double curr_sample_wheel_beta = 0;
	int sample_index = 0;

	//using uniform int distribution to chose a random index
	std::uniform_int_distribution<int> random_part_index_dist(0,(num_particles - 1));

	// using uniform distribution - following python implmentation from lesson5
	std::uniform_real_distribution<double> random_samp_whl_beta(0, (2 * temp_find_max_weight));

	sample_index = random_part_index_dist(dwb_genit3);                ///FIXME need to think about whether I want this in or out.
	

	
	//loop through particles to do resampling
	for (int i = 0; i < num_particles; ++i) {

		curr_sample_wheel_beta += random_samp_whl_beta(dwb_genit4);

		//dwb using random engine to get random value within distribution -this was used in lesson materials
		//std::default_random_engine dwb_genit3(particles[i].sense_x); //using the sense values as a seed
		//std::default_random_engine dwb_genit4(particles[i].sense_y); //using the sense values as a seed

	
		if (i < 1) {
			std::cout << "Resampling Function: " << "sample_index " << sample_index << " " << "beta " << curr_sample_wheel_beta << std::endl;
		}

		// this while loop will pass through to using the current sample index to select the particle if beta is smaller than the current weight
		// this implementing the sampling wheel idea where the arc-length of each element in the wheel is equal to the particle weight
		
		while (curr_sample_wheel_beta > particles[sample_index].weight) {
			curr_sample_wheel_beta -= particles[sample_index].weight; // remove the amount of beta that has been used up moving to the next index spot

			sample_index = (sample_index + 1) % (num_particles); //same trick as in Python - using modulo so that index wraps around

			//std::cout << "sample wheel index " << sample_index;
		}

		//sample_index = 0;

		resampled_particles.push_back(particles[sample_index]); //build up the resampled_particles vector

/*     Python implementation of resampling wheel from course (lesson 5)
		p3 = []
			index = int(random.random()*N)   #random 0 - 1 times 1000 gives random index
			print("index ", index)
			beta = 0.0
			mw = max(w)

			for i in range(N) :
				beta += random.random()*2.0*mw
				#print("beta_outer ", beta, "/n")
				#if beta is less than weight of index then pick that index/particle again
				while beta > w[index]:
					#print("beta_inner ", beta, "/n")
					#keep subtracting the current weight from beta  and incrementing the index - interesting ?
					beta -= w[index]
					index = (index + 1) % N  #interesting so there is a random starting place but the modulo helps it "wrap around"
					# to cover all of the values.
				p3.append(p[index])
			
			p = p3
			print p #please leave this print statement here for grading!*/

	
	}
	
	particles = resampled_particles; //set the main particle set to the newly resampled particle set

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