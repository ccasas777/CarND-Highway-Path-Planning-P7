#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <random>
#include <tuple>
#include <math.h>


#include "constants.h"
#include "Eigen-3.3/Eigen/Dense"
#include "PTG.h"
#include "cost_functions.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

using namespace std;

vector<double> get_traj_coeffs(vector<double> start, vector<double> end, double T)
{
    MatrixXd a(3, 3);
    double T2 = T * T,
        T3 = T2 * T,
        T4 = T3 * T,
        T5 = T4 * T;
    a << T3, T4, T5,
        3 * T2, 4 * T3, 5 * T4,
        6 * T, 12 * T2, 20 * T3;
    MatrixXd aInv = a.inverse();

    VectorXd b(3);
    b << end[0] - (start[0] + start[1] * T + 0.5 * start[2] * T2),
        end[1] - (start[1] + start[2] * T),
        end[2] - (start[2]);
    VectorXd alpha = aInv * b;

    vector<double> output = { start[0], start[1], 0.5 * start[2], alpha[0], alpha[1], alpha[2] };
    return output;
}


double generate_poly(vector<double> coefficients, double T) {
    double x = 0.0f;
    for (unsigned i = 0; i < coefficients.size(); i++) {
        x += coefficients[i] * pow(T, i);
    }
    return x;
}

vector<double> do_differentiate(vector<double> coefficients) {
    vector<double> new_coefficients;

    for (int i = 1; i < coefficients.size(); i++) {
        new_coefficients.push_back(coefficients[i] * (double)i);
    }
    return new_coefficients;
}

vector<double> do_double_differentiate(vector<double> coefficients) {
    vector<double> new_coefficients;

    for (int i = 2; i < coefficients.size(); i++) {
        new_coefficients.push_back(coefficients[i] * (double)i);
    }
    return new_coefficients;
}

vector<double> do_tripple_differentiate(vector<double> coefficients) {
    vector<double> new_coefficients;

    for (int i = 3; i < coefficients.size(); i++) {
        new_coefficients.push_back(coefficients[i] * (double)i);
    }
    return new_coefficients;
}


vector<double> StateFromCoefficients(vector<double> coefficients, double T) {
    vector<double> state;

    double s = generate_poly(coefficients, T);
    state.push_back(s);

    // dot
    vector<double> dot_coefficients = do_differentiate(coefficients);
    double s_dot = generate_poly(dot_coefficients, T);
    state.push_back(s_dot);

    // ddot
    vector<double> ddot_coefficients = do_differentiate(dot_coefficients);
    double s_ddot = generate_poly(ddot_coefficients, T);
    state.push_back(s_ddot);

    return state;
}


vector<vector<double>> PertubedGoal(vector<double> s_target, vector<double> s_sigma_dists, vector<double> d_target, vector<double> d_sigma_dists){

    random_device rd1, rd2;
    mt19937 e1(rd1()),e2(rd2());

    vector<double> s_new_goal, d_new_goal;
    for (int i = 0; i < s_target.size(); i++) {

        normal_distribution<double> s_perturbed_target(s_target[i], s_sigma_dists[i]);
        normal_distribution<double> d_perturbed_target(d_target[i], d_sigma_dists[i]);                
        
        s_new_goal.push_back(s_perturbed_target(e1));
        d_new_goal.push_back(d_perturbed_target(e2));


    }
    //std::cout << "-ing PertubedGoal" << std::endl;
    return { s_new_goal ,d_new_goal };
}

double calculate_cost(vector<double> s_traj, vector<double> d_traj, vector<double> s_target, vector<double> d_target, double T) {

    double total_cost = 0;
    
    vector<vector<double>> costs = {
        {efficiency_cost(s_traj, s_target, T), 1},
        {not_middle_lane_cost(d_traj,d_target,T), 10},
        {time_diff_cost(2,T),1},
        {exceeds_speed_limit_cost(s_traj, d_traj, s_target, T), 999},
        {traj_diff_cost(s_traj,s_target,T), 2 },
        {traj_diff_cost(d_traj,d_target,T), 2 },
        {max_accel_cost(s_traj,d_traj, s_target,T), 999},
        {max_jerk_cost(s_traj,d_traj, s_target,T), 999 }, 
        {average_acc_cost(s_traj,s_target,T),10},
        {average_jerk_cost(s_traj,s_target,T),10}
    };

    
    for (int i = 0; i < costs.size(); i++) {
        total_cost += costs[i][0] * costs[i][1];    
        
    }

    return total_cost;
}

tuple<vector<double>, vector<double>, double> get_best_trajectory(vector<double> current_s, vector<double> target_s, vector<double> current_d, vector<double> target_d, double T) {

    double timestep = 0.4f;
    int N = 1;
    int perturb_samples = 5;
    
    
    vector<double> s_sigma = { 3.0, 0.1, 1.0 };
    vector<double> d_sigma = { 0.3,0.1,0.1 };

    vector<double> new_s_traj = get_traj_coeffs(current_s, target_s, T);
    vector<double> new_d_traj = get_traj_coeffs(current_d, target_d, T);

    double min_cost = calculate_cost(new_s_traj, new_d_traj, target_s, target_d, T);
    vector<double> best_s_goal = target_s;
    vector<double> best_d_goal = target_d;
    vector<double> best_s_traj = new_s_traj;
    vector<double> best_d_traj = new_d_traj;
    double best_T = T;

    double t = T - N * timestep;
    while (t <= T + N * timestep) {
        
        // generate samples

        for (int i = 0; i < perturb_samples; i++) {
            
             vector<vector<double>> new_goal = PertubedGoal(target_s, s_sigma, target_d, d_sigma);   
             vector<double> new_s_goal = new_goal[0];
             vector<double> new_d_goal = new_goal[1];

             new_s_traj = get_traj_coeffs(current_s, new_s_goal, t);
             new_d_traj = get_traj_coeffs(current_d, new_d_goal, t);

            double current_cost = calculate_cost(new_s_traj, new_d_traj, new_s_goal, new_d_goal, t);
            //double current_cost = 0;

            if (current_cost < min_cost) {
                min_cost = current_cost;
                best_s_goal = new_s_goal;
                best_d_goal = new_d_goal;
                best_T =t;
                std::cout << "find new goal" << std::endl;
            }
        }
        t += timestep;
    }

    return make_tuple(best_s_goal, best_d_goal, best_T);
   
}