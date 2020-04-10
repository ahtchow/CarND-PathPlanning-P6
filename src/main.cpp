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

const double SAFETY_DIST(20.0);
const double MAX_VEL(49.5);
const double MAX_ACC(0.15);

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

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
    //start in lane 1
  int lane = 1;
  //reference velocity
  double ref_vel = 0.0; // mph

  h.onMessage([&ref_vel, &lane, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
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

          int prev_size = previous_path_x.size();
          if(prev_size > 0){car_s = end_path_s;}

          //PREDICTION: Predict the future position of cars given sensor fusion
          std::cout << "CURRENT LANE: " << lane << std::endl;
          //Flags
          bool car_in_front = false;
          bool car_left = false;
          bool car_right = false;
          double speed_increment_rate = 0;

          //Update the surrounding status using sensor fusion
          updateSurroundingStatus(car_in_front, car_left, car_right, lane, car_s, ref_vel, sensor_fusion, prev_size, SAFETY_DIST);
          // Perform behaviour planning based on surrounding status
          executeDecision(lane, car_in_front, car_left, car_right, speed_increment_rate, MAX_ACC, MAX_VEL);
          

          // Create  a list of widely spaced (x,y) waypoints, evenly spaced at 30 m
          // Later we will interprpolate these waypoints with a spline and fit it in with more points
          vector<double> pts_x;
          vector<double> pts_y;

          //reference x,y yaw status
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw); 

         // Check previous path size
          if(prev_size < 2)
          {
              // If less than two points just use previous car state and current state to define the tangent path
              double prev_car_x = car_x - cos(car_yaw);
              double prev_car_y = car_y - sin(car_yaw);
              pts_x.push_back(prev_car_x);
              pts_x.push_back(car_x);
              pts_y.push_back(prev_car_y);
              pts_y.push_back(car_y);
          }
          else{
              //Use previous points to define a previous path 
              ref_x = previous_path_x[prev_size -1];
              ref_y = previous_path_y[prev_size -1];
              double ref_x_prev = previous_path_x[prev_size - 2];
              double ref_y_prev = previous_path_y[prev_size -2];
              ref_yaw = atan2(ref_y- ref_y_prev,ref_x-ref_x_prev);

              //Use the two points that make the path tangent to the path's end point
              pts_x.push_back(ref_x_prev);
              pts_x.push_back(ref_x);
              pts_y.push_back(ref_y_prev);
              pts_y.push_back(ref_y);
          
          }

          //In Frenet add evenly 30m spaced points ahead of the starting reference (3 points ahead)
          vector<double> next_wp0 = getXY(car_s +30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s +60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s +90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
          // Add to list (x,y)
          pts_x.push_back(next_wp0[0]);
          pts_x.push_back(next_wp1[0]);
          pts_x.push_back(next_wp2[0]);

          pts_y.push_back(next_wp0[1]);
          pts_y.push_back(next_wp1[1]);
          pts_y.push_back(next_wp2[1]);

          //Transformation to local car coordinates
          for(int i = 0; i < pts_x.size(); ++i){

              // Shift car reference to 0 degrees
              double shift_x = pts_x[i]- ref_x;
              double shift_y = pts_y[i] - ref_y;
              pts_x[i] = (shift_x * cos(0-ref_yaw) - shift_y*sin(0 - ref_yaw));
              pts_y[i] = (shift_x * sin(0-ref_yaw) + shift_y*cos(0 - ref_yaw));

          }

          // Create spline and set points to the spline
          tk::spline spline_tool;
          spline_tool.set_points(pts_x,pts_y); //Anchor Points

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //Add previous path to the list for the next path
          //Clarify: Previous path is the points that did not get processed in given time
          //PURPOSE: Assist with transition between path states
          for(int i = 0; i < prev_size; ++i){
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
          }

          //Calculate how to break up spline points so that we travel at our desired reference velocity
          // PURPOSE: We neeed to space the points evenly to determine how fast we are travelling in given time
          double target_x =  SAFETY_DIST;
          double target_y = spline_tool(target_x); // What is y given x
          double target_distance = sqrt((target_x*target_x)+(target_y*target_y));
          double x_add_on = 0;

          // Fill up the rest of our path planner after filling it with previous points, always end up with 50 points
          for(int i = 1; i <= 50 - prev_size; ++i){

              // Increment Speed according to Planning
              ref_vel += speed_increment_rate;
              if(ref_vel > MAX_VEL) ref_vel = MAX_VEL;
              if(ref_vel < MAX_ACC) ref_vel = MAX_ACC;

              double N = target_distance/(0.02*ref_vel/2.24); // distance / velocity (mps)= time
              double x_point = x_add_on + (target_x)/N;
              double y_point = spline_tool(x_point);
              x_add_on = x_point;

              //Car reference
              double x_ref = x_point;
              double y_ref = y_point;

              //rotate back to normal coordinates after rotating it earlier
              x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
              y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

              //Update reference
              x_point += ref_x;
              y_point += ref_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);

          }

          json msgJson;
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