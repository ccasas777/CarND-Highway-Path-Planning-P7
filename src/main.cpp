#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>


#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "PTG.h"
#include "constants.h"
#include "interpolation_map.h"
// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  // notice: Current operating path is under ../project4/build_vs/..
  string map_file_ = "../data/highway_map.csv";

  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;
  
  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;    
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }
  /*
  vector<double> new_map_waypoints_x = generate_high_precision_map(map_waypoints_s, map_waypoints_x);
  vector<double> new_map_waypoints_y = generate_high_precision_map(map_waypoints_s, map_waypoints_y);
  vector<double> new_map_waypoints_dx = generate_high_precision_map(map_waypoints_s, map_waypoints_dx);
  vector<double> new_map_waypoints_dy = generate_high_precision_map(map_waypoints_s, map_waypoints_dy);
  vector<double> new_map_waypoints_s = generate_high_precision_map(map_waypoints_s, map_waypoints_s);
  */



  //std::cout << "Successfully generate new map by spline interpolation!" << std::endl;

  //std::cout << map_waypoints_s.size() << std::endl;
  double current_acc = 5;
  int lane = 1;
  int brake = 0;  
  int frame = 0;
  double t = 0;
  int count = 0;
  vector<double> s_traj_coeffs = { 0,0,0,0,0,0 };
  vector<double> d_traj_coeffs = { 0,0,0,0,0,0 };
  double start_time = 0;

  tk::spline spline_x;
  tk::spline spline_y;
  tk::spline spline_dx;
  tk::spline spline_dy;

  spline_x.set_points(map_waypoints_s, map_waypoints_x);
  spline_y.set_points(map_waypoints_s, map_waypoints_y);
  spline_dx.set_points(map_waypoints_s, map_waypoints_dx);
  spline_dy.set_points(map_waypoints_s, map_waypoints_dy);
  double prediction_T = 2.0;
  int N_sample =int(prediction_T/DELTA_T);

  h.onMessage([&prediction_T ,&N_sample ,&start_time, &count, &t, &s_traj_coeffs ,&d_traj_coeffs, &frame, &brake, &lane,&current_acc,&spline_x,&spline_y,&spline_dx,
               &spline_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event

    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
          //sensor_fusion[i][3] =>vx
          //sensor_fusion[i][4] =>vy
          //sensor_fusion[i][5] =>car_s
          //sensor_fusion[i][6] =>car_d
          frame += 1;

          /**
           * TODO: define a path  made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
               build a trajectroy in to next_x/y_vals by using the cost function, smooth function(jerk...), ...and so on.
           */

          // spline adding 
          
          //std::cout << "#################FRAME: " << frame <<"#############################" <<std::endl;
          int prev_size = previous_path_x.size();            
          double next_d = 2 + 4 * lane;
          
          
          if (prev_size > 2) {
              car_s = end_path_s;
          }

          bool tooclose = false;          
          bool car_in_left = false;
          bool car_in_right = false;          
          double keep_speed = SPEED_LIMIT * 0.44704;
          

          for (int i = 0; i < sensor_fusion.size(); ++i) {

              //detect the car in the lane that we are in 4 < d < 8
              float d = sensor_fusion[i][6];
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx * vx + vy * vy);
              double check_car_s = sensor_fusion[i][5];
                              
              check_car_s += ((double)prev_size * DELTA_T * check_speed);

              //check the car in the left around our car.
              if (d < (2 + 4 * (lane - 1) + 2) && d > (2 + 4 * (lane - 1) - 2) && lane > 0) {

                  if (check_car_s > car_s && abs(check_car_s - car_s) < 45) {
                      //forward car
                      car_in_left = true;

                  }
                  else if(check_car_s < car_s && abs(check_car_s - car_s) < 8){
                      //rear car
                      car_in_left = true;
                  }

              }

              //check the car in the right around our car.
              if (d < (2 + 4 * (lane + 1) + 2) && d > (2 + 4 * (lane + 1) - 2) && lane < 2) {
                  if (check_car_s > car_s && abs(check_car_s - car_s) < 45) {
                      //forward car
                      car_in_right = true;
                  }
                  else if (check_car_s < car_s && abs(check_car_s - car_s) < 8) {
                      //rear car
                      car_in_right = true;
                  }
              }
          }

          bool change_lane = false;
          double keep_s = car_s + 10;
          for (int i = 0; i < sensor_fusion.size(); ++i) {

              //detect the car in the lane that we are.
              float d = sensor_fusion[i][6];
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx * vx + vy * vy);
              double check_car_s = sensor_fusion[i][5];
              check_car_s += ((double)prev_size * DELTA_T * check_speed);

              if (d < (2 + 4 * lane + 2) && d >(2 + 4 * lane - 2)) {

                  //check the s values greater than mine and s gap
                  if (check_car_s > car_s && abs(check_car_s - car_s) < 30) {

                      tooclose = true;
                      keep_speed = check_speed;
                      keep_s = sensor_fusion[i][5];
                      
                      if (!car_in_left && lane > 0) {
                          lane -= 1;
                          std::cout << "turn the left, now in the lane:" << lane << std::endl;
                          change_lane = true;
                      }
                      else if (!car_in_right && lane < 2) {
                          lane += 1;
                          std::cout << "turn the right, now in the lane:" << lane << std::endl;
                          change_lane = true;
                      }
                      
                  }
              }
          }
          
        
         
          // ************************ PATH GENERATION *******************************
                    // Vehicle class requires s,s_d,s_dd,d,d_d,d_dd - in that order
          double ref_s, s_dot, s_ddot;
          double ref_d, d_dot, d_ddot;
          double target_s, target_s_dot, target_s_ddot;
          double target_d, target_d_dot, target_d_ddot;
          /*
          std::cout << "-----------current car's state-------" << std::endl;
          std::cout << "car_s: " << car_s << std::endl;
          std::cout << "car_d: " << car_d << std::endl;
          std::cout << "car_speed: " << car_speed << std::endl;
          std::cout << "-------------------------------------" << std::endl;
          */
          //creat the list of widely spaced (x, y) waypoints, evenly spaced at 30 m
          //Later we will interoplate these points with the spline method.
          
          vector<double> start_s;
          vector<double> start_d;
          vector<double> end_s;
          vector<double> end_d;
          tuple<vector<double>, vector<double>, double> best_trajectory;

          // use default values if not enough previous path points
          // reference x, y, yaw for the starting point as where the car is or at the previous paths end point.
          if (prev_size < 4) {
              prediction_T = 3.0;
             
              // initialize
              ref_s = car_s;
              s_dot = 0;
              d_dot = 0;

              ref_d = car_d;
              s_ddot = 0;
              d_ddot = 0;

              target_s_dot = (SPEED_LIMIT - 10) * 0.44704;
              target_s_ddot = target_s_dot / prediction_T;
              target_s = ref_s + 0.5 * target_s_dot * prediction_T;

              target_d = next_d;
              target_d_dot = 0;
              target_d_ddot = 0;

              start_s = { ref_s, s_dot,s_ddot };
              start_d = { ref_d, d_dot,d_ddot };

              end_s = { target_s, target_s_dot, target_s_ddot };
              end_d = { target_d, target_d_dot, target_d_ddot };

              count = 0;              
              double planning_T = prediction_T;
         
              s_traj_coeffs = get_traj_coeffs(start_s, end_s, planning_T);
              d_traj_coeffs = get_traj_coeffs(start_d, end_d, planning_T);

              N_sample = int(planning_T / DELTA_T);

              //std::cout << "#################FRAME: " << frame <<"#############################" <<std::endl;
              std::cout << "initialized coeffs !!" << std::endl;
              t = 0; // start for the accumulation time of the generated path
              //-----------------------Debug -------------------------------
              for (int i = 0; i < start_s.size(); i++) {
                  std::cout << " start_s " << i << ": " << start_s[i] << ", end_s " << i << ": " << end_s[i] << std::endl;
              }

          }


          //########################  Set new path plan ###############################

          vector<double> s_dot_traj_coeffs;
          vector<double> s_ddot_traj_coeffs;
          vector<double> d_dot_traj_coeffs;
          vector<double> d_ddot_traj_coeffs;
         
          // when the path's points are running out


          if (tooclose) {
              // car_state at current time
              s_dot_traj_coeffs = do_differentiate(s_traj_coeffs);
              s_ddot_traj_coeffs = do_differentiate(s_dot_traj_coeffs);
              d_dot_traj_coeffs = do_differentiate(d_traj_coeffs);
              d_ddot_traj_coeffs = do_differentiate(d_dot_traj_coeffs);

              ref_s = generate_poly(s_traj_coeffs, t);
              s_dot = generate_poly(s_dot_traj_coeffs, t);
              s_ddot = generate_poly(s_ddot_traj_coeffs, t);

              ref_d = generate_poly(d_traj_coeffs, t);
              d_dot = generate_poly(d_dot_traj_coeffs, t);
              d_ddot = generate_poly(d_ddot_traj_coeffs, t);

              if (change_lane) {
                  prediction_T = 2.2;

                  target_s_dot = (SPEED_LIMIT - 1.5) * 0.44704;
                  target_s_ddot = 0;
                  target_s = ref_s + 0.5 * (s_dot + target_s_dot) * prediction_T;

                  target_d = next_d;
                  target_d_dot = 0;
                  target_d_ddot = 0;

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
                  //double planning_T = prediction_T;

                  s_traj_coeffs = get_traj_coeffs(start_s, end_s, planning_T);
                  d_traj_coeffs = get_traj_coeffs(start_d, end_d, planning_T);

                  N_sample = int(planning_T / DELTA_T);

              }
              else {

                  //waiting to change lane. current state is KL( keep in lane)
                  prediction_T = (NUM_PATH_POINTS - prev_size) * DELTA_T;
                  double diff_s = keep_s - ref_s; 
                  double diff_dot = s_dot - keep_speed; //error sig

                  
                  target_s_dot = s_dot - 0.02 * diff_dot; // approaching the follwoing car speed.

                  if (diff_s < 10) {
                      //the distance for lane change;
                      target_s_dot -= 0.05;
                      std::cout << "tooclose" << std::endl;
                  }

                  target_s = ref_s + 0.5 * (s_dot + target_s_dot) * prediction_T;
                 
                  target_s_ddot = 0; //supposing the process is very slow as no acceleration.
                  
                  double diff_d = ref_d - next_d;
                  if (fabs(diff_d) > 0.2) {
                      target_d = ref_d - 0.1 * diff_d;
                      target_d_dot = 0.1 ;
                      target_d_ddot = 0;
                  }
                  else {
                      target_d = ref_d;
                      target_d_dot = 0;
                      target_d_ddot = 0;
                  }
                  

                  //set current state and target
                  start_s = { ref_s, s_dot,s_ddot };
                  start_d = { ref_d, d_dot,d_ddot };

                  end_s = { target_s, target_s_dot, target_s_ddot };
                  end_d = { target_d, target_d_dot, target_d_ddot };

                  //get coeffs of path             
                  double planning_T = prediction_T;

                  s_traj_coeffs = get_traj_coeffs(start_s, end_s, planning_T);
                  d_traj_coeffs = get_traj_coeffs(start_d, end_d, planning_T);

                  N_sample = int(planning_T / DELTA_T);                  

              }
              t = 0;
              count = 0;             
              
          }
          else if (count >= N_sample) {
              if (!car_in_left && !car_in_right) {
                  lane = 1;
             }
              // no events happened. current state is KL (keep in lane)
              prediction_T = 3.0;

              // car_state at current time
              s_dot_traj_coeffs = do_differentiate(s_traj_coeffs);
              s_ddot_traj_coeffs = do_differentiate(s_dot_traj_coeffs);
              d_dot_traj_coeffs = do_differentiate(d_traj_coeffs);
              d_ddot_traj_coeffs = do_differentiate(d_dot_traj_coeffs);

              ref_s = generate_poly(s_traj_coeffs, t);
              s_dot = generate_poly(s_dot_traj_coeffs, t);
              s_ddot = generate_poly(s_ddot_traj_coeffs, t);


              ref_d = generate_poly(d_traj_coeffs, t);
              d_dot = generate_poly(d_dot_traj_coeffs, t);
              d_ddot = generate_poly(d_ddot_traj_coeffs, t); 

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
                  
              
                            
              target_s_ddot = 0;
              target_s = ref_s + 0.5 * (s_dot + target_s_dot) * prediction_T;

              target_d = next_d;
              target_d_dot = 0;
              target_d_ddot = 0;

              start_s = { ref_s, s_dot,s_ddot };
              start_d = { ref_d, d_dot,d_ddot };

              end_s = { target_s, target_s_dot, target_s_ddot };
              end_d = { target_d, target_d_dot, target_d_ddot };

              //set current state and target
              start_s = { ref_s, s_dot,s_ddot };
              start_d = { ref_d, d_dot,d_ddot };

              end_s = { target_s, target_s_dot, target_s_ddot };
              end_d = { target_d, target_d_dot, target_d_ddot };

           
              double planning_T = prediction_T;

              s_traj_coeffs = get_traj_coeffs(start_s, end_s, planning_T);
              d_traj_coeffs = get_traj_coeffs(start_d, end_d, planning_T);

              N_sample = int(planning_T / DELTA_T);
              t = 0;
              count = 0;

          }

              
              //std::cout << "---------------path lists start--------------------" << std::endl;
              /*
              double test_t = 0;
              for (int i = 0; i < N_sample; ++i) {

                  //the time of the next point

                  double s_val = generate_poly(s_traj_coeffs, test_t);
                  double d_val = generate_poly(d_traj_coeffs, test_t);

                  std::cout << " s_val[" << i << "]: " << s_val << ", end_s[" << i << "]: " << d_val << std::endl;
                  test_t += DELTA_T;
              }
              std::cout << "---------------path lists end--------------------" << std::endl;
              
              //---------------------------- -------------------------------
          }*/
         
         //##################### Generate path ###################################
         //Define the actuall points that we will use for planner.

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // Recall the previous path 
          for (int i = 1; i < previous_path_x.size(); ++i) {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
          }


          
            //fill up with the the next_vals to size of 50.            
            for (int i = 0; i < (NUM_PATH_POINTS - prev_size); ++i) {
                if (count < N_sample) {
                    //from zero to N_sample
                    count += 1;
                    
                    //the time of the next point
                    t += DELTA_T;
                    double s_val = generate_poly(s_traj_coeffs, t);
                    double d_val = generate_poly(d_traj_coeffs, t);
                    
                    double x_point = spline_x(s_val) + d_val * spline_dx(s_val);
                    double y_point = spline_y(s_val) + d_val * spline_dy(s_val);

                    
                    next_x_vals.push_back(x_point);
                    next_y_vals.push_back(y_point);
                }
                           
            }
            //std::cout << "count: " << count << std::endl;
               
          //END
            


          json msgJson;

         //Send to sumulator the path that we plan
          msgJson["next_x"] = next_x_vals; //[0], [1]...//50  --> 47 -->previous_path
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}