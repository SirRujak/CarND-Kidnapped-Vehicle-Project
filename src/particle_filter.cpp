/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h>
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of
	//   x, y, theta and their uncertainties from GPS) and all weights to 1.
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	num_particles = 10;
	// std[0] == x uncertainty
	// std[1] == y uncertainty
	// std[2] == theta uncertainty
	std::default_random_engine generator;
	// distribution(mean, deviation)
  std::normal_distribution<double> distribution_x(x,std[0]);
	std::normal_distribution<double> distribution_y(y,std[1]);
  std::normal_distribution<double> distribution_theta(theta,std[2]);

	//weights(num_particles, 1.0/num_particles); TODO: Fix this.
	weights;
	particles;
	for (int i = 0; i < num_particles; i++) {
		particles.push_back(Particle());
		particles[i].id = i;
		particles[i].x = distribution_x(generator);
		particles[i].y = distribution_y(generator);
		particles[i].theta = distribution_theta(generator);
		particles[i].weight = 1.0/num_particles;
	}

	is_initialized = true;

	std::cout << particles.size() << endl;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	// x uncertainty == std_pos[0]
	// y uncertainty == std_pos[1]


	std::default_random_engine generator;
	// distribution(mean, deviation)
  std::normal_distribution<double> distribution_x(0.0,std_pos[0]);
  std::normal_distribution<double> distribution_y(0.0,std_pos[1]);
  std::normal_distribution<double> distribution_theta(0.0,std_pos[2]);
	// for yaw_rate = 0:
	// x += velocity * delta_t * cos(theta)
	// y += velocity * delta_t * sin(theta)
	// theta = theta

	// else:
	// x += (velocity/yaw_rate) * (sin(theta + yaw_rate * delta_t) - sin(theta))
	// y += (velocity/yaw_rate) * (cos(theta) - cos(theta + yaw_rate * delta_t))
	// theta += yaw_rate * delta_t
	if (yaw_rate < 0.000001 && yaw_rate > -0.0000001){
		// yaw_rate == 0
		for (int i = 0; i < num_particles; i++) {
			particles[i].x += velocity * delta_t * cos(particles[i].theta)
												+ distribution_x(generator);
			particles[i].y += velocity * delta_t * sin(particles[i].theta)
												+ distribution_y(generator);
			particles[i].theta += distribution_theta(generator);
		}
	} else {
		// yaw_rate != 0
		for (int i = 0; i < num_particles; i++) {
			particles[i].x += distribution_x(generator) + (velocity/yaw_rate) * (
				sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta)
			);
			particles[i].y += distribution_y(generator) + (velocity/yaw_rate) * (
				cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t)
			);
			particles[i].theta += yaw_rate * delta_t + distribution_theta(generator);
		}
	}



}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
	//   implement this method and use it as a helper during the updateWeights phase.


	// predicted is the landmarks on the map, observations are the landmarks we see.
	//	for every particle:
	//		for every observation:
	//			closest = 0
	//			for every prediction:
	//				if prediction is closer to landmarks, closest is i

	// for every LandmarkObs in observations:
	double distance, temp_distance = 300.0;
	int current_match = 0;
	for (int i = 0; i < observations.size(); i++) {
		for (int j = 0; j < predicted.size(); j++) {
			temp_distance = sqrt(pow(observations[i].x - predicted[j].x, 2.0)
										+ pow(observations[i].y - predicted[j].y, 2.0));
			if (temp_distance < distance) {
				current_match = j;
			}
		}
		observations[i].id = current_match;
		predicted.erase(predicted.begin() + current_match);
	}

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

	// Transform observations from vehicle to map coordinates
	// Assume that the observations are coming from each particle
	// x_map = x_particle + (np.cos(theta) * x_obs) - (np.sin(theta) * y_obs)
	// y_map = y_part + np.sin(theta) * x_obs) + (np.cos(theta) * y_obs)

	// Variables for calculating probability.
	double distance, t_val, base, expon;
	weights.clear();
	std::vector<LandmarkObs> car_observations;// Make a list of landmarks from the map to pass to dataAssociation.
	std::vector<LandmarkObs> map_observations;
  for (int i = 0; i < num_particles; i++) {
		// For every particle, convert the observed landmarks to map space.
		car_observations.clear();
		for (int j = 0; j < observations.size(); j++) {
			car_observations.push_back(LandmarkObs());

		  std::cout << car_observations.size() << endl;
		  std::cout << j << endl;
		  std::cout << particles.size() << endl;
			std::cout << particles[i].x << endl;
			car_observations[j].x = particles[i].x;
															+ (cos(particles[i].theta) * observations[j].x)
															- (sin(particles[i].theta) * observations[j].y);

			car_observations[j].y = particles[i].y
															+ (sin(particles[i].theta) * observations[j].x)
															+ (cos(particles[i].theta) * observations[j].y);
		}

		map_observations.clear();
		for (int j = 0; j < map_landmarks.landmark_list.size(); j++) {
			distance = sqrt(pow(map_landmarks.landmark_list[j].x_f
													- particles[i].x, 2.0)
										+ pow(map_landmarks.landmark_list[j].y_f
													- particles[i].y, 2.0));
			if (distance < sensor_range) {
				map_observations.push_back(LandmarkObs());
				map_observations.back().id = map_landmarks.landmark_list[j].id_i;
				map_observations.back().x = map_landmarks.landmark_list[j].x_f;
				map_observations.back().y = map_landmarks.landmark_list[j].y_f;
			}
		}
		// Associate landmarks with observations.
		  std::cout << car_observations.size() << endl;
		if (map_observations.size() <= car_observations.size()) {
			weights.push_back(0.0);
		} else {
			dataAssociation(map_observations, car_observations);

			// Calculate the particle's final weight.
			double s = 1.0;
			for (int j = 0; j < car_observations.size(); j++) {

				  std::cout << car_observations[j].id << endl;
				base = 1.0/(2.0 * M_PI * std_landmark[0] * std_landmark[1]);
				expon = - (pow(car_observations[j].x - map_landmarks
																								.landmark_list[
																									car_observations[j].id]
																								.x_f, 2.0)
								  / (2.0 * pow(std_landmark[0], 2.0)))
								+ (pow(car_observations[j].y - map_landmarks
																								.landmark_list[
																									car_observations[j].id]
																								.y_f, 2.0)
									/ (2.0 * pow(std_landmark[1], 2.0)));
				t_val = base * exp(expon);
				s *= t_val;

			}
			particles[i].weight = s;
			weights.push_back(s);
		}
	}


	//

}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	std::default_random_engine generator;
	std::discrete_distribution<> distribution(weights.begin(), weights.end());
	std::vector<Particle> temp_vec;
	int sample;
	for (int i = 0; i < num_particles; i++) {
		temp_vec.push_back(Particle());
		sample = distribution(generator);
		temp_vec[i].x = particles[sample].x;
		temp_vec[i].y = particles[sample].y;
		temp_vec[i].theta = particles[sample].theta;
	}
	particles = temp_vec;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations,
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
