/**
 * @file notes_continued.cpp
 * @brief 
 * @version 0.1
 * @date 2020-04-04
 * 
 * @copyright Copyright (c) 2020
 * 
 */

/**
 * @brief  Environment Classification
 *  
 *      Unstructured  (E.g Parking Lot)
 *              - Less specific rules and lower speeds
 *              - No obvious reference path or trajectory
 *      
 *      Structured
 *           - Predefined rules regarding how to move on the road
 *                      - Direction of traffic, lane boundaries, speed limits
 *          - Road structure can used as a reference
 */

/**
 * @brief We must consider time as well for planning
 *      
 *  i.e Making a lane change on a highways (while stopped)
 *          - we can plan the change in [s,d] dimension
 *          - but we can not simply execute the following tasks whenever (i.e oncoming traffic)
 *          - so we add a third dimension time to plan [s,d,t]
 * 
 *          Plan  on s-t  graph and d-t graph
 * 
 */

/**
 * @brief Trajectories with Boundary Locations
 * 
 *      Continuity and Smoothness
 *          - Speed, acceleration should be continious
 *           -Need position continuity
 * 
 *          Humans feel disconfort when JERK is high
 *              - Design for jerk minimization 
 *              - Jerk = d'''(position) 
 * 
 *           Jerk Minimizing Trajectories:
 *                  Total Jerk =  Integral from 0 to Tf  for d'''(position) ^2
 *            
 *              Minimum 1D Jerk Trajectories:
 *                     s(t) = a + b*t + c* t^2 +d* t^3 + e* t^4 + f * t^5
 *                      (6 coefficients to define the motion of jerk)
 * 
 *                  So for direction s and d
 *                  [S_i, S_i*, S_i** , S_f, S_f*, S_f**]
 *                  [D_i, D_i*, D_i** , D_f, D_f*, D_f**]
 *                  
 *                  How do we find the coefficients that match points we want:
 *                  Solve: Ignored derivation
 *                  
 *                      s(t) = (t^3)  *  d + (t^4) * e  + (t^5) * f +C1
 *                      s(t)/dt = (3t^2)  *  d + (4t^3) * e  + (5t^4) * f +C2
 *                      s(t)/dt^2 = (6t)  *  d + (12t^2) * e  + (20t^3) * f +C3
 *          
 *                  We can use matrix inversing to solve this system of equations
 *                 
 */

/**
 * @brief   Polynomial Solver for Jerk Minimizing Trajectory
 * 
 *          Input: Initial [Position, Velocity, Accerlation], Final [Position, Velocity, Accerlation]
 *          Output : Coefficients that describe the trajectory
 * 
 *          |   T^3        T^4          T^5    |        |  d  |       |  s_f - (s_i + s_i_dot * T + 1/2(s_i_dot_dot* T^2)  |
 *          |   3T^2       4T^3       5T^4  |   x   |  e  |  =   |            s_f_dot - (s_i_dot + s_i_dot_dot * T)         |
 *          |     6T        12T^2     20T^3 |        |  f  |        |                     s_f_dot_dot    - s_i_dot_dot              |
 * 
 */

#include <cmath>
#include  <iostream>
#include  <vector>
#include "Eigen/Dense"

using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * @brief  Calculate the Jerk Minimizing Trajectory that connects the initial state
 * @param start -> [s, s_dot, s_double_dot]
 * @param end -> [s, s_dot, s_double_dot]
 * @param T is the time  in seconds where this maneuver occurs
 * @return coefficients  in form [a, b, c, d, e, f]
 * 
 */
vector<double> JerkMinimizingTrajectory(vector<double> & start, vector<double>  & end, double T){

    MatrixXd A = MatrixXd(3,3);
    A << T*T*T, T*T*T*T, T*T*T*T*T,
            3*T*T, 4*T*T*T,5*T*T*T*T,
            6*T, 12*T*T, 20*T*T*T;

    double s_i  = start[0];
    double s_i_dot = start[1];
    double s_i_dot_dot = start[2];
    double s_f  = end[0];
    double s_f_dot = end[1];
    double s_f_dot_dot = end[2];

    MatrixXd B = MatrixXd(3,1);
    B <<   s_f - (s_i + s_i_dot * T + (0.5)*(s_i_dot_dot*T*T) ),
               s_f_dot - (s_i_dot + s_i_dot_dot * T) ,
               s_f_dot_dot - s_i_dot_dot;
    
    MatrixXd A_inverse = A.inverse();
    MatrixXd C = A_inverse * B;
    vector <double> coefficients;
    coefficients.push_back(s_i);
    coefficients.push_back( s_i_dot);
    coefficients.push_back(s_i_dot_dot * 0.5);

    for(int i; i < C.size(); ++i){
        coefficients.push_back(C.data()[i]);
    }

    return coefficients;
}

/**
 * @brief Feasibility
 *      
 *      Consider: Max Velocity, Min Velocity, Max Accerleration, Min Acceleration, Steering Angle.
 *              -> can not exceed max speed of vechile 
 *              -> minimunum velocity should not be negative, no going backwards
 *              -> lateral acceleration needs to be checked to avoid rollover
 *              -> longitutinal acceleration needs to be checked with powertrain capabilities
 *              -> corresponds to max braking force
 * 
 *          Constraints:
 *              -> For s_dot_dot (Longitudinal Acceleration) :    max breaking <  s_i_dot_dot < max acceleration
 *              -> Fot d_dot_dot (Lateral Acceleration):  | d_dot_dot | < a_y
 *              -> For s_dot (Longitudinal Speed): velocity min < s_dot < velocity speed limit
 */

/**
 * @brief Putting it all together
 *  -> We want to define a end goal
 *      
 *          1. Sample a variety of jerk minimizing trajectories in the define zone
 *          2. Discard all non-drivable trajectories
 *          3. Find best trajectory using cost functions
 *                      -> lateral jerk,  distance to obstacle, distance to center lane, arrival time.
 *          
 * 
 * 
 */


int main(){
    return 0;
}