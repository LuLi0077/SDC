#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <cmath> 

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {

	// Number of particles to draw
	num_particles = 8; 

	// Create normal distributions for x, y and theta
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);

	default_random_engine gen;

	for (int i = 0; i < num_particles; ++i) {
		Particle particle;
    	particle.id = i;
    	particle.x = dist_x(gen);
        particle.y = dist_y(gen);
        particle.theta = dist_theta(gen);
        particle.weight = 1.0;
        particles.push_back(particle);
        weights.push_back(1.0);
    }

    is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {

	normal_distribution<double> dist_x(0, std_pos[0]);
	normal_distribution<double> dist_y(0, std_pos[1]);
	normal_distribution<double> dist_theta(0, std_pos[2]);

	default_random_engine gen;

	for (Particle &particle : particles) {
    
    	if (yaw_rate < 1e-6) {
      		particle.x += velocity*cos(particle.theta)*delta_t + dist_x(gen);
      		particle.y += velocity*sin(particle.theta)*delta_t + dist_y(gen);
      		particle.theta += yaw_rate*delta_t + dist_theta(gen);
    	} else {
      		particle.x += velocity / yaw_rate*(sin(particle.theta + yaw_rate*delta_t) - sin(particle.theta)) + dist_x(gen);
      		particle.y += velocity / yaw_rate*(-cos(particle.theta + yaw_rate*delta_t) + cos(particle.theta)) + dist_y(gen);
      		particle.theta += yaw_rate*delta_t + dist_theta(gen);
    	}
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	/* Find the predicted measurement that is closest to each observed measurement  
	 *  and assign the observed measurement to this particular landmark.
	 */
	for (LandmarkObs &observation : observations) {

    	double closest_dist = INFINITY;
    	int closest_pred = -1;

    	for (int j = 0; j < predicted.size(); ++j) {

    		double distance = dist(observation.x, observation.y, predicted[j].x, predicted[j].y);

    		if (distance < closest_dist) {
      			closest_dist = distance;
        		closest_pred = j;
    		}
    	}

    observation.id = closest_pred;
    
    }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
	// Update the weights of each particle using a mult-variate Gaussian distribution. 

    for (int i = 0; i < num_particles; ++i) {

    	vector<LandmarkObs> trans_obs;
    	for (LandmarkObs &observation : observations) {
        	if (dist(observation.x, observation.y, 0, 0) <= sensor_range) {
        		LandmarkObs obs;
        		obs.x = observation.x*cos(particles[i].theta) - observation.y*sin(particles[i].theta) + particles[i].x;
        		obs.y = observation.x*sin(particles[i].theta) + observation.y*cos(particles[i].theta) + particles[i].y;
        		obs.id = -1;
        		trans_obs.push_back(obs);
      		}
    	}
    
    	vector<LandmarkObs> landmarks;
    	for (int j = 0; j < map_landmarks.landmark_list.size(); ++j) {
        	if (dist(particles[i].x, particles[i].y, map_landmarks.landmark_list[j].x_f, map_landmarks.landmark_list[j].y_f) <= sensor_range) {
        		LandmarkObs landmark;
        		landmark.x = map_landmarks.landmark_list[j].x_f;
        		landmark.y = map_landmarks.landmark_list[j].y_f;
        		landmark.id = map_landmarks.landmark_list[j].id_i;
        		landmarks.push_back(landmark);
        	}
    	}
    
    	dataAssociation(landmarks, trans_obs);

    	double p = 1;

    	for (int j=0; j < trans_obs.size(); ++j) {
    		int k = trans_obs[j].id;
    		p *= exp(-(pow(trans_obs[j].x - landmarks[k].x, 2)/ (2 * pow(std_landmark[0], 2)) + pow(trans_obs[j].y - landmarks[k].y, 2) / (2 * pow(std_landmark[1], 2)))) / (2 * M_PI*std_landmark[0]*std_landmark[1]);      		
    	}

    	weights[i] = p;
    	particles[i].weight = p;

  	} 
}

void ParticleFilter::resample() {
	// Resample particles with replacement with probability proportional to their weight. 

	default_random_engine gen;

    double max_weight = *max_element(begin(weights), end(weights));
    uniform_real_distribution<> dist(0, 2*max_weight);

    vector<Particle> particles_resample;
    int index = rand()%num_particles;
    double beta = 0.0;

    for (int i = 0; i < num_particles; ++i) {
        beta += dist(gen);
        while (beta > weights[index]) {
            beta -= weights[index];
            index = (index + 1) % num_particles;
        }
        particles_resample.push_back(particles[index]);
    } 

    particles = particles_resample;  
}

void ParticleFilter::write(std::string filename) {
	std::ofstream dataFile;
	dataFile.open(filename, std::ios::app);
	for (int i = 0; i < num_particles; ++i) {
		dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
	}
	dataFile.close();
}
