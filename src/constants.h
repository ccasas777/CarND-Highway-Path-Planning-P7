#ifndef CONSTANTS
#define CONSTANTS

#define N_SAMPLES 5
#define NUM_PATH_POINTS 50
#define DELTA_T 0.02
#define DT 1
#define SPEED_LIMIT 48.0
#define MAX_INSTANTANEOUS_ACCEL 10
#define MAX_INSTANTANEOUS_JERK 2
#define EXPECTED_ACC_IN_ONE_SEC 1
#define EXPECTED_JERK_IN_ONE_SEC 2
//#define max_s 5128.307
// cost function weights
#define COLLISION_COST_WEIGHT 1
#define BUFFER_COST_WEIGHT 1
#define IN_LANE_BUFFER_COST_WEIGHT 1
#define EFFICIENCY_COST_WEIGHT 1
#define SPEED_LIMIT_COST_WEIGHT 1
#define NOT_MIDDLE_LANE_COST_WEIGHT 1
#define MAX_ACCEL_COST_WEIGHT 1
#define MAX_JERK_COST_WEIGHT 1
#define EXPECTED_ACCEL_COST_WEIGHT 1
#define EXPECTED_JERK_COST_WEIGHT 1


#endif