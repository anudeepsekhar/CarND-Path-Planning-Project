#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

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
  double ref_vel = 0.0;
  int lane = 1;

  // Waypoint map to read from
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
  

  h.onMessage([&ref_vel,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&lane]
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

          json msgJson;

          int path_size = previous_path_x.size();
          
          

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          /*###################################################        Sensor Fusion         ###############################################################*/

          if (path_size > 0){
            car_s = end_path_s;
          }

          //set flag for vehicle detection
          bool car_ahead = false;
          bool car_left = false;
          bool car_left_ahead = false;
          bool car_right = false;
          bool car_right_ahead = false;
          bool lane_free = false;
          double lane_change_left = 0;
          double lane_change_right = 0;
          double stay_in_lane = 10;
          bool slow_down = false;
          double car_dist = 1;
          double cara_speed = ref_vel; 

          //check each car in sensor fusion
          int car_lane = 1;
          for (int i = 0; i<sensor_fusion.size(); i++){
            //check if car is in your lane
            double d = sensor_fusion[i][6];
            if(d > 0 && d < 4){
              car_lane = 0;
            } else if (d > 4 && d < 8){
              car_lane = 1;
            } else if(d > 8 && d < 12){
              car_lane = 2;
            }
            //get car values
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_car_v = sqrt(pow(vx,2)+pow(vy,2));
            double check_car_s = sensor_fusion[i][5];
            double max_dist = 30;
            double car_width = 10;
            if (car_lane == lane){
              //check car s in future
              check_car_s += ((double)path_size*0.02*check_car_v);
              
              //check if we are too close to the car
              if ((check_car_s > car_s) && ((check_car_s - car_s) < max_dist)){
                car_ahead = true;
                car_dist = check_car_s - car_s; // Calculate the distance from detected vehicle
                cara_speed = check_car_v;
              }
              }
              else if((car_lane - lane == -1)){// Check the lane to the left
                if (((check_car_s < car_s) && ((car_s - check_car_s-15) < car_width))||((check_car_s > car_s) && ((check_car_s - car_s) < car_width))){
                  car_left = true;
                }
                if (((check_car_s > car_s) && ((check_car_s - car_s) < car_width+3))){ // Check for car a little ahead in the lane to the left
                  car_left_ahead = true;
                }
              }
              else if((car_lane - lane == 1)){// Check the lane to the right
                if (((check_car_s < car_s) && ((car_s - check_car_s-15) < car_width))||((check_car_s > car_s) && ((check_car_s - car_s) < car_width))){
                  car_right = true;
                }
                if (((check_car_s > car_s) && ((check_car_s - car_s) < car_width + 3))){// Check for car a little ahead in the lane to the right
                  car_right_ahead = true;
                }
              }else if(lane == 0){
                car_left = true;
              }else if (lane == 2){
                car_right = true;
              }
          }
          

          /*###################################################      Behavior Planning       ###############################################################*/
          
          // //DEBUG
          // std::cout << "car ahead: "<< car_ahead <<std::endl;
          // std::cout << "car left: "<< car_left <<std::endl;
          // std::cout << "car right: "<< car_right <<std::endl;
          // std::cout << "car dist: "<< car_dist <<std::endl;
          
          double throttle_p = 0.5;
          double brake = 0;

          brake = (ref_vel - cara_speed)/car_dist;
          
          // Determine what behavior to execute...

          if (car_ahead){
            slow_down = true;
            stay_in_lane -= 3; // penalize staying in the lane

            if (!car_left){ // check if lane change left possible
              slow_down = false;
              lane_change_left += 5; // Favor changing lane to the left
              stay_in_lane -= 5; // Further penalize staying in current lane because lane change is possible
              
            }else if(!car_right){ // check if lane change right possible
              slow_down = false;
              lane_change_right += 5; // Favor changing lane to the right
              stay_in_lane -= 5; // Further penalize staying in current lane because lane change is possible
              
            }else{
              stay_in_lane = 10; // Favour staying in current lane
              slow_down = true;
            }
            if (lane_change_left == lane_change_right){ 
            if (car_left_ahead && (lane_change_left>0)){ // check if car is detected ahead in the left lane 
              lane_change_left -= 3; // penalize changing lane to the left
              stay_in_lane +=3; // favour staying in the current
            }else if (car_right_ahead && (lane_change_right>0)){ // check if car is detected ahead in the right lane 
              lane_change_right -=3; // penalize changing lane to the right
              stay_in_lane +=3;  // favour staying in the current
            }
          }           
          }
          else {
            slow_down = false;
          }
 
          // Executing determined behavior...

          if (slow_down){
            std::cout<<"Slowing down..."<<std::endl;
            brake = (ref_vel - cara_speed)/car_dist; // Calculate the braking proportional to difference between ego vehicle speed and detected vehicle and also the distance between them.
            ref_vel = ref_vel - brake;
            // std::cout<<"ref_vel v "<<std::endl;
          }else{
            if (ref_vel < 49.5){
            double accel = (49.5 - ref_vel)/40; 
            ref_vel = ref_vel + accel;
            std::cout<<"Speeding up"<<std::endl; 
          }
          }

         // Behavior execution for changing lanes.

          if (lane_change_left > lane_change_right){
            lane = lane - 1;
            std::cout<<"Changing lane to the LEFT"<<std::endl;
              if(lane < 0){
                lane = 0;
              }
          }else if (lane_change_right > lane_change_left){
            lane = lane + 1;
            std::cout<<"Changing lane to the right"<<std::endl;
              if (lane > 2){
                lane = 2;
              }
          }
          // //DEBUG
          // std::cout << "Change Lane Left: "<< lane_change_left <<std::endl;
          // std::cout << "Change Lane Right: "<< lane_change_right <<std::endl;
          // std::cout << "Stay In Lane: "<< stay_in_lane <<std::endl;
          

          /*###################################################    Trajectory Generation     ###############################################################*/

          // Reference states for x,y and yaw
          // we can use these reference points to reference the starting point of our path. it can be either the car current posintion or the previous paths endpoint
          vector<double> ptsx;//X points array to creat the spline 
          vector<double> ptsy;//Y points array to creat the spline 
          double ref_car_x = car_x;
          double ref_car_y = car_y;
          double ref_car_yaw = deg2rad(car_yaw);
          
          //this is to start generating the spline,
          //if the previous path is almost empty, we can use the car's position as the starting reference.
          if (path_size < 2) {

            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(ref_car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(ref_car_y);


          } else { //take the previous path's endpoint as reference.
            ref_car_x = previous_path_x[path_size-1];
            ref_car_y = previous_path_y[path_size-1];

            double ref_prev_x = previous_path_x[path_size-2];
            double ref_prev_y = previous_path_y[path_size-2];
            ref_car_yaw = atan2(ref_car_y-ref_prev_y,ref_car_x-ref_prev_x);

            ptsx.push_back(ref_prev_x);
            ptsx.push_back(ref_car_x);
            
            ptsy.push_back(ref_prev_y);
            ptsy.push_back(ref_car_y);
          }

          // std::cout << "ptsx: ";
          // printvec(ptsx);
          // std::cout<< std::endl;

          // std::cout<<"ref_x: "<<ref_car_x<<std::endl;
          // std::cout<<"ref_y: "<<ref_car_y<<std::endl;

          //generate frenet's coordinates which are spaced 30m apart.
          //get there XY coordinates.
          vector<double> pt0 = getXY(car_s+30, 2+4*lane, map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> pt1 = getXY(car_s+60, 2+4*lane, map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> pt2 = getXY(car_s+90, 2+4*lane, map_waypoints_s,map_waypoints_x,map_waypoints_y);

          ptsx.push_back(pt0[0]);
          ptsy.push_back(pt0[1]);

          ptsx.push_back(pt1[0]);
          ptsy.push_back(pt1[1]);

          ptsx.push_back(pt2[0]);
          ptsy.push_back(pt2[1]);

          // std::cout << "ptsx: " << ptsx.size() << std::endl;
          // std::cout << "ptsx: ";
          // printvec(ptsx);
          // std::cout<< std::endl;

          //transform the points to the cars reference frame for easier calculations.

          for (int i = 0; i < ptsx.size(); i++){
            double shift_x = ptsx[i]-ref_car_x;
            double shift_y = ptsy[i]-ref_car_y;

            ptsx[i] = shift_x*cos(0-ref_car_yaw)-shift_y*sin(0-ref_car_yaw);
            ptsy[i] = shift_x*sin(0-ref_car_yaw)+shift_y*cos(0-ref_car_yaw);
          }

          // std::cout << "ptsx: ";
          // printvec(ptsx);
          // std::cout<< std::endl;

          //Create the spline.
          tk::spline s;

          s.set_points(ptsx,ptsy);


          //Start building the trajectory.

          //Add all the previous path points to the trajectory
          for (int i = 0; i < previous_path_x.size(); i++){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          // std::cout<<"prev size: "<< previous_path_x.size() << std::endl;
          // std::cout << "next_xvals: ";
          // printvec(next_x_vals);
          // std::cout<< std::endl;

          //Calculate how to space the points to control the speed.
          double horizon_x = 30;
          double horizon_y = s(horizon_x);

          double target_dist = sqrt(pow(horizon_y,2)+pow(horizon_x,2));
          // std::cout<<target_dist<<std::endl;
          double sim_timestep = 0.02;
          double x_addon = 0;

          //fill up the rest of the points of the trajectory.

          for (int i = 1; i < 50 - previous_path_x.size(); i++){

            //divide the path into N points such that ref velocity is maintained.
            double N = target_dist/(ref_vel*sim_timestep/2.24);
            double x_point = x_addon + target_dist/N;
            double y_point = s(x_point);

            x_addon = x_point;

            //transform back into global reference frame
            
            double ref_x = x_point;
            double ref_y = y_point;

            x_point = ref_x*cos(ref_car_yaw) - ref_y*sin(ref_car_yaw);
            y_point = ref_x*sin(ref_car_yaw) + ref_y*cos(ref_car_yaw);

            x_point = x_point+ref_car_x;
            y_point = y_point+ref_car_y;

            //add these points to trajectory of the car to follow.
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);

          }
          // std::cout<<"prev size: "<< previous_path_x.size() << std::endl;
          // std::cout << "next_xvals: ";
          // printvec(next_x_vals);
          // std::cout<< std::endl;


          
          msgJson["next_x"] = next_x_vals;
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