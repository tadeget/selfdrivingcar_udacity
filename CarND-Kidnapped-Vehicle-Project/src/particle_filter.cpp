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
using std::normal_distribution;
//#static std::default_random_engine gen;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  num_particles = 10;  // TODO: Set the number of particles
  std::default_random_engine gen;
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);
  Particle temp;
  for(int i=0; i<num_particles; i++)
  {
    temp.id = i;
    temp.x = dist_x(gen);
    temp.y = dist_y(gen);
    temp.theta = dist_theta(gen);
    temp.weight = 1.0;
    particles.push_back(temp);
    weights.push_back(temp.weight);
  }
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
  std::default_random_engine gen;
           // Creating normal distributions
   normal_distribution<double> dist_x(0, std_pos[0]);
   normal_distribution<double> dist_y(0, std_pos[1]);
   normal_distribution<double> dist_theta(0, std_pos[2]);
  

   for(int i=0; i < particles.size(); i++)
   {
     double theta = particles[i].theta;
     //std::cout<<"predication_before "<<" x "<<particles[i].x <<" y "<<particles[i].y<<" yaw "<<yaw_rate<<" delta_t "<<delta_t<<" theta "<<theta<<std::endl;

     if (yaw_rate!=0 && fabs(velocity/yaw_rate) > 0.0001)
     {
     	particles[i].x += (velocity/yaw_rate)*(sin(theta + yaw_rate*delta_t) -  sin(theta));
     	particles[i].y += (velocity/yaw_rate)*(cos(theta)-cos(theta + yaw_rate*delta_t));
     	particles[i].theta +=yaw_rate*delta_t;
     }
     else
     {
        particles[i].x += velocity * cos(theta) * delta_t;
	    particles[i].y += velocity * sin(theta) * delta_t;

     }
      // Adding noise.
    particles[i].x += dist_x(gen);
    particles[i].y += dist_y(gen);
    particles[i].theta += dist_theta(gen);
   // std::cout<<"predication "<<" x "<<particles[i].x <<" y "<<particles[i].y<<std::endl;

   }
 
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * c: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */
   if(predicted.size() == 0)
     return;

   for(int i=0; i<observations.size(); i ++)
   {
     double obs_x = observations[i].x;
     double obs_y = observations[i].y;
     int index_id=-1;
     double min_dist = std::numeric_limits<double>::max();
     for(int j=0;j<predicted.size();j++)
     {
        double val = dist(obs_x,obs_y,predicted[j].x,predicted[j].y);
        if (min_dist > val)
        {
          min_dist = val;
          index_id = j;
        }
     } 
     observations[i].id = predicted[index_id].id;
     //std::cout<<"data assosiation **********"<< observations[i].id <<"                    " << predicted[index_id].id<<std::endl;
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
   // use for normization of all the weights
    double weight_sum = 0.0;
 	for(int i=0; i<particles.size();i++)
 	{ 
      // transform the observed measurments
      vector<LandmarkObs> transformed_obs;
      double p_theta = particles[i].theta;
      double p_x = particles[i].x;
      double p_y = particles[i].y;
      int id;
      double x,y;
      for(int j=0; j<observations.size(); j++)
      {
        id = observations[j].id;
        x = p_x + (cos(p_theta) * observations[j].x) - (sin(p_theta) * observations[j].y);
        y = p_y + (sin(p_theta) * observations[j].x) + (cos(p_theta) * observations[j].y); 
        transformed_obs.push_back(LandmarkObs{id,x,y});
        //std::cout<<" " <<id<<" ";
      }
     // std::cout<<"Map id starting **********"<<std::endl;
      // filter the observation based on the location of the sensors
      vector<LandmarkObs> predicted_landmarks;
      for(int j=0; j<map_landmarks.landmark_list.size(); j++)
      {
        Map::single_landmark_s map_landmark = map_landmarks.landmark_list[j];
        
        //if ((fabs((p_x - map_landmark.x_f)) <= sensor_range) && (fabs((p_y - map_landmark.y_f)) <= sensor_range)) {
        if (dist(p_x, p_y, map_landmark.x_f, map_landmark.y_f) <= sensor_range){
            predicted_landmarks.push_back(LandmarkObs {map_landmark.id_i, map_landmark.x_f, map_landmark.y_f});
           // std::cout<<" " <<map_landmark.id_i<<" ";
         }
      }
      //std::cout<<std::endl;
    
      // data assosiation
      dataAssociation(predicted_landmarks, transformed_obs);
      
      
      // calculate the weight
      double x_sigma = std_landmark[0];
      double y_sigma = std_landmark[1];
      weight_sum  =0;
      ///std::cout<<"++++++++++++++++++++++++ i= "<<i<<" obs_size= "<<transformed_obs.size() <<" pred_size= "<< predicted_landmarks.size() <<" ++++++++++++++++++++++" <<std::endl;
      particles[i].weight = 1.0;  // reset the weight of the particle
      for(int j=0; j < transformed_obs.size();j++)
      {
        double x_tran = transformed_obs[j].x;
        double y_tran = transformed_obs[j].y;
        
        for(int k=0; k < predicted_landmarks.size(); k++)
        {
          double x_pred= predicted_landmarks[k].x;          
          double y_pred = predicted_landmarks[k].y;
          
          if(predicted_landmarks[k].id == transformed_obs[j].id)
          {
            double val =  Gaussian(x_tran, x_pred, y_tran, y_pred,x_sigma,y_sigma);
            if(val == 0.0)
            {
                  particles[i].weight*= 0.00001;
            }
            else
            {
           		 particles[i].weight*= val;
            }
            
             ///std::cout<< "computing gaussins  "<<predicted_landmarks[k].id <<"  "<<transformed_obs[j].id<<" " << val<<std::endl;
            // std::cout<< "x_tran  "<<x_tran <<"  "<<x_pred<<" " << y_tran <<" "<<y_pred<<std::endl;
             //std::cout<< "postion of particle  "<<p_x <<"  "<<p_y<<" " << p_theta <<std::endl;
            // std::cout<< "orginal observation  "<<observations[j].x <<"  "<<observations[j].y <<std::endl;
           // std::cout<<"++++++++++++++++++++++++" << "computing gaussins  "<<predicted_landmarks[k].id <<"  "<<transformed_obs[j].id<<" " << val<<std::endl;
            //std::cout<<"++++++++++++++++++++++++" << "x_tran  "<<x_tran <<"  "<<x_pred<<" " << y_tran <<" "<<y_pred<<std::endl;
            break;
          }
        }
        weight_sum+=  particles[i].weight;
        
      }
       
  	}
  // normalalize the weights
}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
  for (int i = 0; i < particles.size(); i++) {
    //particles[i].weight /= weight_sum;
    weights[i] = particles[i].weight;
  }
vector<Particle> resampled_particles;
double max_w = *max_element(weights.begin(), weights.end());
 
 //Generate random particle index
std::default_random_engine gen;
std::uniform_int_distribution<int> random_index(0, num_particles - 1);
std::uniform_real_distribution<double> random_weight(0.0, 2*max_w);
  
int index = random_index(gen);
double beta = 0;
 
for(int i =0; i<particles.size(); i++)
{
   beta +=  random_weight(gen);
   while (weights[index] < beta)
     {
        beta = beta - particles[index].weight; 
        index = (index +1)% particles.size();
     }
    resampled_particles.push_back(particles[index]);
    //std::cout<<"resample "<<" x "<<particles[index].x <<" y "<<particles[index].y<<std::endl;
 } 
  
  particles = resampled_particles;
         
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
  
  //Clear the previous associations
   particle.associations.clear(); 
   particle.sense_x.clear();
   particle.sense_y.clear();
  
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
