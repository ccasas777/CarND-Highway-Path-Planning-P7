#ifndef PTG
#define PTG

#include <vector>

using namespace std;

vector<double> get_traj_coeffs(vector<double> start, vector<double> end, double T);
//vector<vector<double>> generate_trajectory(vector<double> current_s, vector<double> current_d, vector<vector<double>> target, double duration);
vector<double> do_differentiate(vector<double> coefficients);
vector<double> do_double_differentiate(vector<double> coefficients);
vector<double> do_tripple_differentiate(vector<double> coefficients);
double generate_poly(vector<double> coefficients, double T);
double calculate_cost(vector<double> s_traj, vector<double> d_traj, vector<double> s_target, vector<double> d_target, double T);
vector<double> PertubedGoal(vector<double> target, vector<double> sigma_dists);
tuple<vector<double>, vector<double>, double> get_best_trajectory(vector<double> current_s, vector<double> target_s, vector<double> current_d, vector<double> target_d, double T);
#endif