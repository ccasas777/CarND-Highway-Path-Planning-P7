[//]: # (Image References)
[image1]: ./img/img1.jpg

# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

## Overview  
Here is the video that the program performance demo: https://youtu.be/3GwYtBBtu_s

1. main.cpp : receiving all the sensor data and the main path generation to the simulator ["next_x_vals"], ["next_y_vals"]

2. PTG.cpp : there are the JMT transform and perturbation method to pick up the best trajectory sending to main.cpp 

3. cost_functions.h : there are the cost functions that might be used to the cost calculation at PTG.cpp

4. constant.h : restore the constants like the speed limit or acceleration limit...

5. helper.h : some tools to translate the s-d coordinate and x-y coordinate

6. spline.h : the spline library

## Descriptions

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points I have used so I can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed.

3. Initially, I fast completed the coding example as the teaching video by using a spline method to produce a smooth path and following lane as below codes. 

```cpp
  int prev_size = previous_path_x.size();

  vector<double> ptsx;
  vector<double> ptsy;
  
  double ref_x = car.x;
  double ref_y = car.y;
  double ref_yaw = deg2rad(car.yaw);

  if (prev_size < 2) {
    double prev_car_x = car.x - cos(car.yaw);
    double prev_car_y = car.y - sin(car.yaw);
  
    ptsx.push_back(prev_car_x);
    ptsx.push_back(car.x);
  
    ptsy.push_back(prev_car_y);
    ptsy.push_back(car.y);
  } else {
    ref_x = previous_path_x[prev_size-1];
    ref_y = previous_path_y[prev_size-1];
  
    double ref_x_prev = previous_path_x[prev_size-2];
    double ref_y_prev = previous_path_y[prev_size-2];
    ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
  
    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);
  
    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }
  
  vector<double> next_wp0 = map.getXYspline(car.s+30, get_dcenter(target.lane));
  vector<double> next_wp1 = map.getXYspline(car.s+60, get_dcenter(target.lane));
  vector<double> next_wp2 = map.getXYspline(car.s+90, get_dcenter(target.lane));
  
  
  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);
  
  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);
  
  
  for (int i = 0; i < ptsx.size(); i++) {
    // shift car reference angle to 0 degrees
    // transformation to local car's coordinates (cf MPC)
    // last point of previous path at origin and its angle at zero degree
  
    // shift and rotation
    double shift_x = ptsx[i]-ref_x;
    double shift_y = ptsy[i]-ref_y;
  
    ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0 - ref_yaw));
    ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0 - ref_yaw));
  }
  
  
  tk::spline spl;
  spl.set_points(ptsx, ptsy);
  
  vector<double> next_x_vals;
  vector<double> next_y_vals;
  
  for (int i = 0; i < prev_size; i++) {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }
  
  // Calculate how to break up spline points so that we travel at our desired reference velocity
  double target_x = 30.0;
  double target_y = spl(target_x);
  double target_dist = sqrt(target_x*target_x + target_y*target_y);
  
  double x_add_on = 0;
  
  // fill up the rest of our path planner after filing it with previous points
  // here we will always output 50 points
  for (int i = 1; i <= PARAM_NB_POINTS - prev_size; i++) {
    double N = (target_dist / (PARAM_DT * mph_to_ms(target.velocity))); // divide by 2.24: mph -> m/s
    double x_point = x_add_on + target_x/N;
    double y_point = spl(x_point);
  
    x_add_on = x_point;
  
    double x_ref = x_point; // x_ref IS NOT ref_x !!!
    double y_ref = y_point;
  
    // rotate back to normal after rotating it earlier
    x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
    y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));
  
    x_point += ref_x;
    y_point += ref_y;
  
    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }


```

Then I started to produce the path by the JMT method that showed in the classes. Basically, I write the most tools (including cost functions) by the reference of python codes that offered by the class. For example, the  JMT transform:

```cpp
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

```
## Path smooth & Coordinate transform:


```cpp
tk::spline spline_x;
tk::spline spline_y;
tk::spline spline_dx;
tk::spline spline_dy;

spline_x.set_points(map_waypoints_s, map_waypoints_x);
spline_y.set_points(map_waypoints_s, map_waypoints_y);
spline_dx.set_points(map_waypoints_s, map_waypoints_dx);
spline_dy.set_points(map_waypoints_s, map_waypoints_dy);
                    .
                    .
                    .
                    .

double x_point = spline_x(s_val) + d_val * spline_dx(s_val);
double y_point = spline_y(s_val) + d_val * spline_dy(s_val);

 ```
## Path Planning:　
![image1]

I simply separate the possible driving conditions into three parts in the highway driving project:

1. Regular driving: no other car so that our car just need to follow the lanes and don't break the rules of speed limit, acceleration limit and so on.

2. Following the car: can't change the lane so that I need to follow the car and wait the better time to cut the car off

3. Change the lane: choose a better path to pass the car in front of us


In regular driving, I setup a road curve detection to prevent over the speed limit:

```cpp
 //check the road curve
  double slope1 = spline_dy(car_s + 5) / spline_dx(car_s + 5 );
  double slope2 = spline_dy(car_s + 6) / spline_dx(car_s + 6 );
  double A = pow(sqrt((1 + slope1 * slope2)), 3);
  double B = fabs(slope2 - slope1);
  double curve = A / B;
  
  if (curve < 6000 && curve > 1000) {
      target_s_dot = (SPEED_LIMIT - 0.7) * 0.44704;
      std::cout << "slightly slow down" << std::endl;
  }
  else if (curve < 1000) {
      target_s_dot = (SPEED_LIMIT - 2) * 0.44704;
      std::cout << "slow down" << std::endl;
  }
  else {
      target_s_dot = SPEED_LIMIT * 0.44704;
  
  }
                  
 ```

Only in change-lanes coding, I add the perturbation method to pick up the best trajectory. In regular driving and following driving, I simply use the default target without perturbation to reduce the computing effort  

```cpp
  //set current state and target
  start_s = { ref_s, s_dot,s_ddot };
  start_d = { ref_d, d_dot,d_ddot };

  end_s = { target_s, target_s_dot, target_s_ddot };
  end_d = { target_d, target_d_dot, target_d_ddot };

  //get coeffs of path with perturbation algorithm
  best_trajectory = get_best_trajectory(start_s, end_s, start_d, end_d, prediction_T);
  double planning_T = get<2>(best_trajectory);
  end_s = get<0>(best_trajectory);
  end_d = get<1>(best_trajectory); 

  s_traj_coeffs = get_traj_coeffs(start_s, end_s, planning_T);
  d_traj_coeffs = get_traj_coeffs(start_d, end_d, planning_T);

                  
 ```

## Discussion:　

1. I have failed to write an advanced part that I want to complete is "preparing for lane-change". So, the car can't predict the cars around my car and prepare to cut into the lanes depending on the cars velocity, position and acceleration in the wanted lane.

2. I didn't write smartly so that limit my perturbation samples to calculate. I found if the consuming time(0.02 s) of coding is too long to send to the simulator, the simulator would display a red warning for acceleration. Maybe I should write some "class" to restore the ego.car information to reduce the repeated calculation such as my car position and velocity.