/**
 * @file intro.cpp
 * @author Adrian Chow (you@domain.com)
 * @brief Intro to Prediction
 * @version 0.1
 * @date 2020-03-25
 * 
 * @copyright Copyright (c) 2020
 */

/**
 * @brief Prediction is multi-model. (In terms of probability)
 * - multiple possibility
 * - must constantly update using sensor data
 * 
 *  MAP/SENSOR DATA -> PREDICTION MODULE -> PREDICTION
 */

/**
 * @brief TABLE OF CONTENTS
 * 
 *  1. Introduction
 *         a. High Level 
 *         b Inputs Outputs
 *  2. How is Prediction Done?
 *          a. Model-based -> math based prediction
 *          b. Data-driven -> machine learning prediction
 *              (Tragectory Clustering)
 *  3. Model Based Approach
 *          a. Process Models
 *          b. Multi-modal estimators
 * 
 *  4. Hybrid Approach 
 *          a. Intent Classification -> Trajectory Generation
 *          b. Naive Bayes
 * 
 */


/**
 * @brief Model-Based vs Data-Driven Approaches
 *      
 *   Which is better?
 *      
 *  Model-Based:
 *             
 *          Go Straight Model -> Go Straight Trajectory \    
 *          Go Straight Model -> Go Straight Trajectory-> Multimodel Estimation Algorithim -> Prediction Trajectory
 *                               Observed Behaviour     /
 * 
 * 
 *  Data-Driven:
 *           
 *          Observed Behaviour \
 *                              Machine Learning -> Preedicted Trajectories
 *              Training Data  /
 * 
 * 
 *   Each prediction method: Has own strengths, hybrid model may be best approach
 * 
 */                             