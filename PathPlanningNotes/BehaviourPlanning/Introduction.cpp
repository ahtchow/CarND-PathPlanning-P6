/**
 * @file Behaviour Planning Intro
 * @author Adrian Chow (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2020-03-26
 * 
 * @copyright Copyright (c) 2020
 * 
 */

/**
 * @brief Behaviour PLanning
 *  - Lowest update rate
 *  - Input from: localization,sensor fusion, motion control
 *  - Process: Prediction->Behaviour PLanning -> Trajectory
 * 
 *  Behaviour Planner
 *      Input: Map, Route, Predictions
 *      Output: Suggested Maneuver
 *      
 *      Responsibilities: 
 *          - Filter for feasible , safe, legal, efficient paths
 *      Not responsible for: execution details, collision avoidance.
 * 
 * 
 */

/**
 * @brief Navigator / Finite State Machine
 *     
 *      - Makes decisions based on finite amount of decisions
 *      - Each state can be connected by set of possible transitions
 *      - Self-Transition: 
 *      - Accepting State: A state that has no transitions
 * 
 *      - Takes the input and uses transition function to decide which transition to make
 *      - There are so many states, often one state inherits other ones.
 * 
 *      - We will simplify to: Keep lane, change lane left, change lane right
 *      
 *      LANE KEEP:
 *          - d -> stay near center line for lane
 *          - s -> drive at target speed when feasible
 *          
 *      LANE CHANGE LEFT/RIGHT
 *          - d -> move left or right
 *          - s -> same rules as keep lane (for initial lane)
 * 
 *      Problems: 
 *          Could not slow down to get closer to gap,
 *          lane change is not safe ( need to speed up to lane change),
 *          not clear when to turn on turn signals
 * 
 *      THus, we add: PREPARE FOR LANE CHANGE LEFT/RIGHT STAGE:
 *              d - stay near center line for current lane
 *              s - attempt to match position and speed of "gap" in lane
 *              signal - activate turning signal
 * 
 * 
 */

/**
 * @brief What do we need as input for our transition functions?
 *        
 *      Predictions, Map, Speed Limit, Localization Data, Current State
 *      
 * 
 */

/**
 * @brief COST FUNCTION
 * 
 *  How do we design a cost function.
 *      - Compare Cost vs Velocity 
 *      - Above speed limit is maximum cost
 *      - Not moving is high cost too
 * 
 *      target_speed = speed_limit - BUFFER_V
 *      if v < target_speed:
 *          cost = STOP_COST * ((target_speed - v)) / target_speed)
 *      if v >speed_limit:
 *          cost  = 1.0
 *      if v <speed_limit & v> target_speed:
 *          cost = (v - target_speed) / BUFFER_V
 * 
 * 
 * So we want a cost function that penalizes large |Δd| and we want that 
 *  penalty to be bigger when Δs is small.
 * 
 *  Furthermore, we dont want the cost to ever exceed one or go below zero
 * 
 *      so: cost = 1− exp(-|Δd|/Δs)
 * 
 */