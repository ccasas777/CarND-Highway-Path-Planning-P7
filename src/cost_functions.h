#ifndef COST_FUNCTIONS
#define COST_FUNCTIONS

#include <iostream>
#include <cassert>
#include <vector>
#include <algorithm>
#include <cmath>
#include "constants.h"
#include "PTG.h"

using namespace std;

// COST FUNCTIONS

double logistic(double x) {
    // A function that returns a value between 0 and 1 for x in the range[0, infinity] and - 1 to 1 for x in 
    // the range[-infinity, infinity]. Useful for cost functions.
    return 2.0 / (1 + exp(-x)) - 1.0;
}

double time_diff_cost(double target_time, double actual_time) {
    
    // Penalizes trajectories that span a duration which is longer or shorter than the duration requested.
    return logistic(fabs(actual_time - target_time) / target_time);
}

double traj_diff_cost(vector<double> traj, vector<double> target,  double T) {

    // Penalizes trajectories whose s coordinate (and derivatives) differ from the goal. Target is s, s_dot, and s_ddot.
    // can be used for d trajectories as well (or any other 1-d trajectory)
    vector<double> actual_path(target.size());
    actual_path[0] = generate_poly(traj, T);
    actual_path[1] = generate_poly(do_differentiate(traj), T);
    actual_path[2] = generate_poly(do_double_differentiate(traj), T);
    
    double cost = 0;
    for (int i = 0; i < actual_path.size(); i++) {
        cost += abs(actual_path[i] - target[i]) ;
    }
    return logistic(cost);
}

double exceeds_speed_limit_cost(vector<double> s_traj, vector<double> d_traj, vector<double> target, double T) {
    
    // Penalty if ds/dt for any two points in trajectory is greater than SPEED_LIMIT
    vector<double> s_vel_coef = do_differentiate(s_traj);
    vector<double> d_vel_coef = do_differentiate(d_traj);
    double N_sample = 20;
    double dt = T / N_sample;
    double t = 0;
    for (int i = 0; i < N_sample; i++) {
        
        double s_vel = generate_poly(s_vel_coef, t);
        double d_vel = generate_poly(d_vel_coef, t);
        double total_vel = sqrt(s_vel * s_vel + d_vel + d_vel);
        if (total_vel > SPEED_LIMIT) {
            return 1;
        }
        t += dt;
    }
    return 0;
}
double max_accel_cost(vector<double> s_traj, vector<double> d_traj,vector<double> target, double T) {
    // Penalty if ds/dt for any two points in trajectory is greater than SPEED_LIMIT
    vector<double> s_acc_coef = do_double_differentiate(s_traj);
    vector<double> d_acc_coef = do_double_differentiate(s_traj);
    double N_sample = 10;
    double dt = T / N_sample;
    double t = 0;
    for (int i = 0; i < N_sample; i++) {

        double s_acc = fabs(generate_poly(s_acc_coef, t));
        double d_acc = fabs(generate_poly(d_acc_coef, t));
        double total_acc = sqrt(s_acc * s_acc + d_acc * d_acc);
        if (total_acc > MAX_INSTANTANEOUS_ACCEL) {
            return 1;
        }
        t += dt;
    }
    return 0;
}

double max_jerk_cost(vector<double> s_traj, vector<double> d_traj, vector<double> target, double T) {
    // Penalty if ds/dt for any two points in trajectory is greater than SPEED_LIMIT
    vector<double> s_jerk_coef = do_tripple_differentiate(s_traj);
    vector<double> d_jerk_coef = do_tripple_differentiate(s_traj);
    double N_sample = 10;
    double dt = T / N_sample;
    double t = 0;
    for (int i = 0; i < N_sample; i++) {

        double s_jerk = fabs(generate_poly(s_jerk_coef, t));
        double d_jerk = fabs(generate_poly(d_jerk_coef, t));
        double total_jerk = sqrt(s_jerk * s_jerk + d_jerk * d_jerk);
        if (total_jerk > MAX_INSTANTANEOUS_JERK) {
            return 1;
        }
        t += dt;
    }
    return 0;
}

double efficiency_cost(vector<double> s_traj, vector<double> target, double T) {
    // Rewards high average speeds.
    double actual_end_path = generate_poly(s_traj, T);
    double actual_ave_v = actual_end_path / T;
    double pred_ave_v = target[0] / T;   
    double cost = 2 * ((pred_ave_v - actual_ave_v) / actual_ave_v);
    
    return logistic(cost);
}

double average_acc_cost(vector<double> s_traj, vector<double> target, double T) {
    // Rewards high average speeds.
    vector<double> coef_acc = do_double_differentiate(s_traj);
    double total_acc = 0;
    double N_sample = 10;
    double dt = T / N_sample;
    double t = 0;
    for (int i = 0; i < N_sample; i++) {
        total_acc += abs(generate_poly(coef_acc, t) * dt);
        t += dt;
    }     
    double cost = total_acc / T / EXPECTED_ACC_IN_ONE_SEC;
    return logistic(cost);
}

double average_jerk_cost(vector<double> s_traj, vector<double> target, double T) {
    // Rewards high average speeds.
    vector<double> coef_jerk = do_tripple_differentiate(s_traj);
    double total_jerk = 0;
    double N_sample = 10;
    double dt = T / N_sample;
    double t = 0;
    for (int i = 0; i < N_sample; i++) {
        total_jerk += abs(generate_poly(coef_jerk, t) * dt);
        t += dt;
    }

    double cost = total_jerk / T / EXPECTED_JERK_IN_ONE_SEC;
    return logistic(cost);
}
double not_middle_lane_cost(vector<double> d_traj, vector<double> d_target, double T) {
    // penalize not shooting for middle lane (d = 6)
    double end_d = generate_poly(d_traj, T);
    
    return logistic(pow(end_d - d_target[0], 2));
}

#endif