#include <iostream>
#include <vector>
#include <cmath>

using std::cout;
using std::endl;

double inefficiency_cost(int target_speed, int intended_lane, int final_lane, 
                         const std::vector<int> &lane_speeds) {
  // Cost becomes higher for trajectories with intended lane and final lane 
  //   that have traffic slower than target_speed.

  int speed_intended_lane = lane_speeds[intended_lane];
  int speed_final_lane = lane_speeds[final_lane];
  double cost = cost = 1 - exp(-(std::abs(2.0*target_speed-speed_intended_lane-speed_final_lane)/target_speed));
    
  return cost;
}

int main() {
  // Target speed of our vehicle
  int target_speed = 10;

  // Lane speeds for each lane
  std::vector<int> lane_speeds = {6, 7, 8, 9};
    
  // Test cases used for grading - do not change.
  double cost;
  cout << "Costs for (intended_lane, final_lane):" << endl;
  cout << "---------------------------------------------------------" << endl;
  cost = inefficiency_cost(target_speed, 3, 3, lane_speeds);
  cout << "The cost is " << cost << " for " << "(3, 3)" << endl;
  cost = inefficiency_cost(target_speed, 2, 3, lane_speeds);
  cout << "The cost is " << cost << " for " << "(2, 3)" << endl;
  cost = inefficiency_cost(target_speed, 2, 2, lane_speeds);
  cout << "The cost is " << cost << " for " << "(2, 2)" << endl;
  cost = inefficiency_cost(target_speed, 1, 2, lane_speeds);
  cout << "The cost is " << cost << " for " << "(1, 2)" << endl;
  cost = inefficiency_cost(target_speed, 1, 1, lane_speeds);
  cout << "The cost is " << cost << " for " << "(1, 1)" << endl;
  cost = inefficiency_cost(target_speed, 0, 1, lane_speeds);
  cout << "The cost is " << cost << " for " << "(0, 1)" << endl;
  cost = inefficiency_cost(target_speed, 0, 0, lane_speeds);
  cout << "The cost is " << cost << " for " << "(0, 0)" << endl;
    
  return 0;
}

/**
 * @brief Cost Function Design
 * 
 *  Difficulties:
 *      - Solving new problems without unsolving old ones   
 *              - Regression Testing!
 *      - Balancing Cost of drastically different magnitudes
 *              - feasibility, safety, legality, comfort, efficiency
 *              - weights must change on different situations
 *      - Reasoning about individual CFs
 *              - specificity of cost function responsibility
 *              - Binary vs discrete vs continious cost functions
 *              - ALL CFs output between -1 and 1
 *              - Parametrization (when possible)
 *              - Thinking in terms of vehicle state
 *  
 *              
 */