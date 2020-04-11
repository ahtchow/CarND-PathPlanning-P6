/**
 * @file A_star.cpp
 * @author Adrian Chow 
 * @brief A*
 * @version 0.1
 * @date 2020-04-02
 * 
 * @copyright Copyright (c) 2020
 * 
 */

/**
 * @brief A* Reminder
 *  
 *      True:
 *          -Uses optimistic heuristic function to guide grid cell expansion
 *          -It always find solution if it exists and is always optimal
 *          - Solution finds are drivable assuming map is accurate, but not realistic
 *         
 *      False:
 *          - The algorithim is continious method
 :
 *          
 * 
 */

/**
 * @brief Hybrid A*
 * 
 *      - Now world is continious: but A* is discrete
 *      - Key to solving is state transition function
 *      
 *      True:
 *          - Is continiour search method
 *          - It always find solution if it exists and is always optimal
 *          - Driving paths are always drivable
 *      False:
 *          - Does always find solution if exists
 *          - Solutions are always optimal
 * 
 * 
 *      Equations:
 *          x(t+Δt) = x(t) + v * Δt * cos(Ѳ)
 *          y(t+Δt) = y(t) + v * Δt * sin(Ѳ)
 *          Ѳ(t+Δt) = Ѳ(t) + w * Δt
 *          w = v/L * tan(δ(t))
 * 
 *      Practical Assumptions:
 *          - maximum steering angle in both directions and zero 
 *              - -35, 0 and 35
 *          - assemble path of 3 components 
 * 
 */