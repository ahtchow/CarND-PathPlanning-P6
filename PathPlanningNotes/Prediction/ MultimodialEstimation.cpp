/**
 * @file  MultimodialEstimation.cpp
 * @author Adrian Chow (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2020-03-25
 * 
 * @copyright Copyright (c) 2020
 * 
 */

/**
 * @brief Autonomour Multiple Model Algorithm - Variables
 *      Variables:
 *          - Consider some set of M process models/Behaviours
 *          - Probabbilities for process models mu1, mu2, mu3... muM
 * 
 *      How is probability calculated:
 *          - A Multimodel Gaussian Distribution is plotted with x-axis at s
 *          - A slice through the Probabbility Density Functions are compared as ratios
 * 
 *      mu(K)  =  ( mu(K-1) * L(K) ) / Summation of (mu(K-1) * L(K))
 *   (probability) = (probability at k-1)("likelyhood ratio at k") / Normalizer
 * 
 */

/**
 * @brief NAIVE BAYES
 *  
 *  Simple Classifier
 *      e.g male or female
 *  
 *  - Compute P(male| h, w) and P(female|h,w)
 *      -  P(male | h,w) = P(h|w,h) * P(w|male) * P(male) / P(h,w)
 *  
 * Called Naive Bayes because it assumes "naively" that all feautures contribute independantly
 *      - No need to know P(h,w) to normalize
 *      all about finding these terms: p(h|male), p(w|male)
 *  
 *  Thus we can use: Gaussian Naive Bayes
 *      
 *      p(h|male) =  ~ N(mu_male_height, var_male_height)
 * 
 *      Data's Responsibility:
 *          1. Select Relevant Features
 *          2. Indentify means/ variances for diff classes
 *                  a. Lots of Data
 *                  b. Identify good features
 *          
 */