#include "pros/rtos.hpp"
#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <vector>
#include <cmath>
#include <random>
using namespace std;
// Define field dimensions
const double FIELD_WIDTH = 144.0;   // Inches
const double FIELD_HEIGHT = 144.0;  // Inches

// Number of particles
const int NUM_PARTICLES = 2000;

// Robot motion noise
const double ODOMETRY_NOISE = 0.5;  // Inches
const double ROTATION_NOISE = 0.02; // Radians

// Structure to represent a particle
struct Particle {
    double x, y, theta; // Position and orientation
    double weight;      // Probability weight
};

// Random number generator
random_device rd;
mt19937 gen(rd());
uniform_real_distribution<double> rand_pos(-72, 72);
uniform_real_distribution<double> rand_angle(0, 360);
normal_distribution<double> rand_noise(0, ODOMETRY_NOISE);
normal_distribution<double> rand_rot_noise(0, ROTATION_NOISE);

double sd(double distance){
    double variance = std::max(distance * 0.05, 0.590551);
    return sqrt(variance);
}
// Initialize particles randomly on the field
vector<Particle> initializeParticles(int num_particles) {
    vector<Particle> particles;
    for (int i = 0; i < num_particles; i++) {
        particles.push_back({rand_pos(gen), rand_pos(gen), rand_angle(gen), 1.0 / num_particles});
    }
    return particles;
}

// Apply odometry update to particles
void motionUpdate(vector<Particle>& particles, double delta_x, double delta_y, double delta_theta) {
    for (auto& p : particles) {
        p.x += delta_x + rand_noise(gen);
        p.y += delta_y + rand_noise(gen);
        p.theta += delta_theta + rand_rot_noise(gen);
    }
}

double distance_to_wall(double x, double y, double theta){
    int quadrant = theta/90 + 1;
    double related_acute = theta - (quadrant-1)*90;
    if(quadrant == 1){
        double x_dist = abs(72-x);
        double y_dist = abs(72-y);
        double max_dist = sqrt(x_dist*x_dist + y_dist*y_dist);
        double possibility1 = x_dist/abs(cos(90-related_acute));
        double possibility2 = y_dist/(abs(cos(related_acute)));
        if(possibility1 < max_dist){
            return possibility1;
        }
        else if(possibility2 < max_dist){
            return possibility2;
        }
        else return 72*sqrt(2);
    }
    else if(quadrant == 2){
        double x_dist = abs(72-x);
        double y_dist = abs(-72-y);
        double max_dist = sqrt(x_dist*x_dist + y_dist*y_dist);
        double possibility1 = x_dist/abs(cos(related_acute));
        double possibility2 = y_dist/(abs(cos(90-related_acute)));
        if(possibility1 < max_dist){
            return possibility1;
        }
        else if(possibility2 < max_dist){
            return possibility2;
        }
        else return 72*sqrt(2);
    }
    else if(quadrant == 3){
        double x_dist = abs(-72-x);
        double y_dist = abs(-72-y);
        double max_dist = sqrt(x_dist*x_dist + y_dist*y_dist);
        double possibility1 = x_dist/abs(cos(90-related_acute));
        double possibility2 = y_dist/(abs(cos(related_acute)));
        if(possibility1 < max_dist){
            return possibility1;
        }
        else if(possibility2 < max_dist){
            return possibility2;
        }
        else return 72*sqrt(2);
    }
    else {
        double x_dist = abs(-72-x);
        double y_dist = abs(72-y);
        double max_dist = sqrt(x_dist*x_dist + y_dist*y_dist);
        double possibility1 = x_dist/abs(cos(related_acute));
        double possibility2 = y_dist/(abs(cos(90-related_acute)));
        if(possibility1 < max_dist){
            return possibility1;
        }
        else if(possibility2 < max_dist){
            return possibility2;
        }
        else return 72*sqrt(2);
    }
}



// Calculate particle weights based on sensor measurement
void measurementUpdate(vector<Particle>& particles, double sensor_measurement_front, double sensor_measurement_left, double sensor_measurement_right) {
    double total_weight = 0;
    
    for (auto& p : particles) {
        double expected_distance_front = distance_to_wall(p.x, p.y, p.theta);
        double expected_distance_left = distance_to_wall(p.x, p.y, (p.theta+270)-((p.theta+270)/360)*360);
        double expected_distance_right = distance_to_wall(p.x, p.y, (p.theta+90)-((p.theta+90)/360)*360);
        double standard_dev = sd(expected_distance_front);
        double gaussianF = (exp(-0.5*pow((sensor_measurement_front-expected_distance_front)/standard_dev, 2)))/(standard_dev*sqrt(2*M_PI));
        double gaussianL = (exp(-0.5*pow((sensor_measurement_left-expected_distance_left)/standard_dev, 2)))/(standard_dev*sqrt(2*M_PI));
        double gaussianR = (exp(-0.5*pow((sensor_measurement_right-expected_distance_right)/standard_dev, 2)))/(standard_dev*sqrt(2*M_PI));
        p.weight = max((gaussianF + gaussianL + gaussianR)/3, 1e-19);
        total_weight += p.weight;
    }       

    // Normalize weights
    for (auto& p : particles) {
        p.weight /= total_weight;
    }
}

// Resample particles based on weight
vector<Particle> resampleParticles(vector<Particle>& particles) {
    vector<Particle> new_particles;
    
    return new_particles;
}

// Compute the estimated robot position as a weighted average
Particle estimatePosition(const vector<Particle>& particles) {
    double x_sum = 0, y_sum = 0, theta_sum = 0, weight_sum = 0;

    for (const auto& p : particles) {
        x_sum += p.x * p.weight;
        y_sum += p.y * p.weight;
        theta_sum += p.theta * p.weight;
        weight_sum += p.weight;
    }

    return {x_sum / weight_sum, y_sum / weight_sum, theta_sum / weight_sum, 1.0};
}

// Main function
int main() {
    vector<Particle> particles = initializeParticles(NUM_PARTICLES);

    while (true) {
        // Simulate motion (from odometry)
        double delta_x = 2.0;    // Example movement in inches
        double delta_y = 1.5;    
        double delta_theta = 0.1; // Rotation in radians

        motionUpdate(particles, delta_x, delta_y, delta_theta);

        // Simulate sensor measurement (from distance sensor)
        double sensor_measurement_front = 0;
        double sensor_measurement_left = 0;
        double sensor_measurement_right = 0;

        measurementUpdate(particles, sensor_measurement_front, sensor_measurement_left, sensor_measurement_right);
        particles = resampleParticles(particles);

        // Estimate position
        Particle estimated = estimatePosition(particles);

        pros::delay(100);
    }

    return 0;
}