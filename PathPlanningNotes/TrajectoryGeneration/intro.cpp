/**
 * @author Adrian Chow (you@domain.com)
 * @brief  Trajectory Generation Intro
 * @version 0.1
 * @date 2020-04-02
 * 
 * @copyright Copyright (c) 2020
 * 
 */

/**
 * @brief The Motion Planning Problem
 * 
 *  Configuration Space:
 *  - Defines all possible configurations in 2-d world
 *  - 2D can be [x,y] or 3-d [x,y,theta]
 *  
 *  Given:
 *      - start configuration Qstart (from localization/sensors)
 *      -a goal configuration Qgoal (from behaviour planning)
 *      -constraints
 * 
 *  Problem:
 *      - move object from start to goal w/o hitting any obstacles
 * 
 * 
 */

/**
 * @brief Motion Planning ALgorithim Properties
 *      
 *   Completeness:
 *      - If solution exists, find the solution
 *      - If no solution exists, terminate and report
 *   Optimality:
 *       - given cost function , always return sequence of action with least cost
 * 
 */

/**
 * @brief Motion Planning Algorithm
 * 
 *      1. Combinatorial Methods
 *          - Using graphs (nodes,edges )
 * 
 *      2. Potential Field Methods
 *          - Anti gravity field to guide and plan trajectories
 *          - Shy away from padestrians
 *       
 *      3. Optimal Control
 *          -  Solving path planning approach using one algorithm
 *          - dynamic model that output steering, throttle
 * 
 *      4. Sampling Based Methods
 *          - Collision detection by probing free space
 *          - Utlizes graph search like Dijkstra or A star
 *          - Discrete Methods: Discretization of the configuration and input space
 *              - A* , D* , Dijkstras , etc
 *          - Probabilistic methods: Random exploration of the configuration and input space
 *              - RRT, RRT*, PRM, etc ..
 */