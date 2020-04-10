#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>
#include "json.hpp"
#include <iostream>

// for convenience
using std::string;
using std::vector;
using nlohmann::json;
using namespace std;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, 
                    const vector<double> &maps_y) {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, 
                 const vector<double> &maps_y) {
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = std::min(2*pi() - angle, angle);

  if (angle > pi()/2) {
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, 
                         const vector<double> &maps_x, 
                         const vector<double> &maps_y) {
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if (next_wp == 0) {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, 
                     const vector<double> &maps_x, 
                     const vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1))) {
    ++prev_wp;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),
                         (maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}

void updateSurroundingStatus(
      bool & car_in_front, bool & car_left, bool & car_right,
      const int lane, const double car_s, const double ref_vel,
      const nlohmann::json & sensor_fusion, 
      const int prev_size, const double SAFETY_DIST){
    
    const double SAFE_PASS_DIST = 7.5;
    double average_delta_s = 0;
    size_t valid_entries = 0;

  for(int i=0; i < sensor_fusion.size(); ++i){
    float d = sensor_fusion[i][6];
    int car_lane = -1;
    //Identify the lane of the vehicle
    if ( d > 0 && d < 4 ) 
      car_lane = 0;
    else if ( d > 4 && d < 8 ) 
      car_lane = 1;
    else if ( d > 8 && d < 12 ) 
      car_lane = 2;
    // No cars found
    if(car_lane < 0) continue;

    // Determine car speed 
    double v_x = sensor_fusion[i][3]; // x velocity
    double v_y = sensor_fusion[i][4]; // y velocity
    double speed_mag = sqrt(v_x*v_x + v_y*v_y);
    double sensor_car_s = sensor_fusion[i][5]; // s value 

    // Predict future position under linear velocity assumption
    sensor_car_s +=  (double)prev_size*0.02*speed_mag;

    //Update the flags
    if(car_lane == lane)
      // Is car ahead, and within SAFETY_DIST meters
      car_in_front |= (sensor_car_s > car_s ) && ((sensor_car_s - car_s) < SAFETY_DIST) ;
    
    else if(car_lane == (lane-1) ){
      // Is car to the left, and within SAFETY_DIST meters vertically
      car_left |= car_s - sensor_car_s < SAFETY_DIST  && car_s - sensor_car_s > -SAFETY_DIST;
      // SAFE_PASS
      if(sensor_car_s < car_s  && car_s - sensor_car_s < SAFE_PASS_DIST)
        car_left |= 0;
    }
      
    else if(car_lane == (lane+1) ){
      // Is car to the right, and within SAFETY_DIST meters vertically
      car_right |= car_s - sensor_car_s < SAFETY_DIST && car_s - sensor_car_s > -SAFETY_DIST;
      // SAFE_PASS
      if(sensor_car_s < car_s  && car_s - sensor_car_s < SAFE_PASS_DIST)
        car_right |= 0;
    }
  }

  cout <<  "STATUS LEFT: " << car_left << " "
          << "STATUS FRONT: " << car_in_front << " " 
          << "STATUS RIGHT: " << car_right << endl;
}

int findBestLaneChange(
      const int lane, 
      const bool car_in_front, 
      const bool car_left, 
      const bool car_right){

  //Check if car is located straight ahead
  if(car_in_front){
    //Try to make a left lane change
    if(!car_left && lane > 0)
      return (lane -1);

    //If left lanechange is not availible, try right lane change
    else if(!car_right && lane  != 2)
      return (lane +1);
    
    //Stay in lane if lane change is not safe
    else 
      return lane;
  }

  return lane;
}

void executeDecision(
    int & lane, 
    bool & car_in_front, 
    bool & car_left, 
    bool & car_right,
    double & speed_increment_rate,
    const double MAX_ACC,
    const double MAX_VEL){

const double SAFETY_WORTH = 30.0;
double target_lane = findBestLaneChange(lane, car_in_front, car_left, car_right);
//If the target lane is the current and there is a car ahead -> SLOW DOWN
if(lane == target_lane && car_in_front  == 1)
  speed_increment_rate = -MAX_ACC;

//If the target lane is the current and there is no car ahead -> MAINTAIN SPEED 
else if(lane == target_lane && car_in_front == 0)
  speed_increment_rate = MAX_ACC;

lane = target_lane;

}

#endif  // HELPERS_H