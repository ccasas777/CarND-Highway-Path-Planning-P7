#ifndef INTERPOLATION_MAP_H
#define INTERPOLATION_MAP_H


#include <math.h>
#include <string>
#include <vector>
#include "spline.h"


// for convenience
using std::string;
using std::vector;

vector<double> generate_high_precision_map(vector<double> map_waypoints_s, vector<double> map_waypoints) {
	vector<double> new_map_waypoints;
	int map_size = map_waypoints_s.size();
	double s_start = map_waypoints_s[0];	
	double s_end = map_waypoints_s[map_size - 1];
	tk::spline sp;	
	sp.set_points(map_waypoints_s, map_waypoints);

	for (double s = s_start; s <= s_end; s++) {
		double new_map_waypoint = sp(s);
		new_map_waypoints.push_back(new_map_waypoint);
	}

	return new_map_waypoints;

}

#endif