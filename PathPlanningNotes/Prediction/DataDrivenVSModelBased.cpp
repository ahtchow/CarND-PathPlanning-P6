/**
 * @file DataDrivenVSModelDriven.cpp
 * @author Adrian Chow (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2020-03-25
 * 
 * @copyright Copyright (c) 2020
 * 
 */

/**
 * @brief Trajectory Clustering
 *  
 *  1. Offline Stage - Training Model   
 *          a. Get lot of tragetories
 *          b. Clean Data
 *          c. Define some measure of similarity (classification->straight/right turn)
 *          d. Unsupervised learning
 *          f. Define best prototype trajectories for each cluster
 *  
 *  2. Online Stage
 *          Every update cycle:
 *          a. Observe  vechile's partial trajectory
 *          b. Compare to prototype trajectories
 *          c. Belief is updated based on similary to prototype
 *          d. Predict a trajectory
 * 
 */

/**
 * @brief Model Based Approaches
 *  
 *  For each dynamic object nearby
 *      1. Identify common driving behaviours(change lane, turn left, cross street)
 *      2. Define process model for each (Mathmatically)
 *      3. Update beliefs by comparing observation with output of process model
 *      4. Trajectory Generation
 *      5. Derive probability of each trajection
 * 
 *  Repeat until the "horizon" is achieved
 * 
 */

/**
 * @brief Frenet Coordinates
 *  
 *   - We must represent the position on road in more intuitive way than (x,y)
 *   - FRENET COORDINATES: 
 *          a. s coordinate - longitudinal distance along the road
 *          b. d coordinate - side-to-side (lateral) positon on the road
 *      
 *     Why? Because Curvy roads. 
 *      -> Essentially this is a orgin that move relative to middle of road
 *      So only why increases if the car stays in the middle of the road
 *      and can be modelled by s(t) = Vo*t  /  d(t) = 0
 * 
 */


/**
 * @brief Process Model
 *  Consider merging onto the highway:
 *    Possible Behaviours ---- Process Modesl -------------
 *      -  Ignore Us -> Follow A with Constant Velocity
 *      -  Speed Up -> Follow A with Positive Acceleration
 *      -  Slow Down -> Follow A with Negative Acceleration
 *      -  Change Lane -> Follow B with Constant Velocity
 * 
 *  Four Models for Lane Following:
 *     
 *      1. Linear point model (Treat as point particle)
 *          -> TOO SIMPLE
 * 
 *      2. Non-linear point model (Constant acceleration w/ curvature)
 *      
 *      3. Kinematic Bicycle Model w. PID controller on distance and angle
 *      
 *      4. Dynamic Bicycle Model w/ PID controller on distance and angle  
 *      
 *      In Practice: Too much uncertainty to predicting other drivers, 
 *              minor accuracy improves, arent worth the overhead of complexity
 *      
 *      Instead: A Multivariate Gaussian w/ Zero Mean is used
 */

/**
 * @brief Implementing Gaussian Naive Bayes
 *   
 *   3 Lane highway, 4m wide/ lane
 *      
 *      1. Lane Change Left (Blue)
 *      2. Keep Lane (Black)
 *      3. Lane Change Right (Red)
 *  
 *      Write a classifier that can predict which of these three maneuvers 
 *      a vehicle is engaged in given a single coordinate
 *      
 *       Coordinate: [s,d,ds/dt,dd/dt]
 *      
 *     Instructions:
 *      1. Implement train(data,labbels) method in the Class GNB
 *          -> Compute and store the mean and standard deviation from the data for each label/feauture.
 *          -> E.g given label "lane change left" and feature ds/dt , add the data to the classification
 *          -> Compute and store the prior probability p(C_k) for each label C_k. This can be done by keeping track of the number of times each label appears in the training data
 *      
 *      2. Implement the predict(observation)
 *          -> Given a new data point, prediction requires two steps:
 *          1. Compute the conditional probabilities for each feature/label combinations
 *           
 *          For a feature x and label C , with mean mu and sigma (s.d) 
 *          
 *              p(x=v | C) = (1/sqrt(2*pi*mu^2)) * exp(-(v-mu)^2/2(sigma)^2)
 * 
 * 
 *          2. Use the conditional probabilities in a Naive Bayes Classifier
 * 
 *                  argmax taken over all possible labels C(k) and the product
 *                  is taken over all features x(i) w/ values v(i)
 * 
 */

#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen/Dense"

using Eigen::ArrayXd;
using std::cout;
using std::endl;
using std::ifstream;
using std::string;
using std::vector;

class GNB{

public:
    GNB(){
        left_means = ArrayXd(4);
        left_means << 0,0,0,0;
        left_sds = ArrayXd(4);
        left_sds << 0,0,0,0;

        keep_means = ArrayXd(4);
        keep_means << 0,0,0,0;
        keep_sds = ArrayXd(4);
        keep_sds << 0,0,0,0;

        left_means = ArrayXd(4);
        left_means << 0,0,0,0;
        left_means = ArrayXd(4);
        left_means << 0,0,0,0;

    }
    virtual ~GNB();

    void train(const vector<vector<double>> &data, const vector<string> &labels){
    
        //Initialize size of lists
        float left_size = 0;
        float keep_size = 0;
        float right_size = 0;

        //For each label, compute the numerators of the means for each class
        // and the total number of data points given with that label.
        for(int i = 0; i < labels.size(); ++i){

            //Label is left
            if(labels[i] == possible_labels[0]){
                left_means += ArrayXd::Map(data[i].data(), data[i].size());
                left_size += 1;
            }

            //Label is middle
            else if(labels[i] == possible_labels[1]){
                keep_means += ArrayXd::Map(data[i].data(), data[i].size());
                keep_size += 1;
            }

            //Label is left
            else if(labels[i] == possible_labels[2]){
                right_means += ArrayXd::Map(data[i].data(), data[i].size());
                right_size += 1;
            }

        }

        left_means /= left_size;
        keep_means /= keep_size;
        right_means /= right_means;

        ArrayXd data_point;

        // Compute numerators of the standard deviations.
        for (int i=0; i<labels.size(); ++i) {
            
            data_point = ArrayXd::Map(data[i].data(), data[i].size());

            if (labels[i] == possible_labels[0])
                left_sds += (data_point - left_means)*(data_point - left_means);
            
            else if (labels[i] == possible_labels[1]) 
                keep_sds += (data_point - keep_means)*(data_point - keep_means);
            
            else if (labels[i] == possible_labels[2])
                right_sds += (data_point - right_means)*(data_point - right_means);
            
        }

        left_sds = (left_sds/left_size).sqrt();
        keep_sds = (keep_sds/keep_size).sqrt();
        right_sds = (right_sds/right_size).sqrt();

        //Compute the probability of each label
        left_prior = left_size/labels.size();
        keep_prior = keep_size/labels.size();
        right_prior = right_size/labels.size();

    }

    string predict(const vector<double> &sample){

        // Calculate product of conditional probabilities for each label.
        double left_p = 1.0;
        double keep_p = 1.0;
        double right_p = 1.0; 

        for(int i=0; i<4; ++i){

            left_p *= (1.0/sqrt(2.0*M_PI*pow(left_sds[i],2)))
                    *exp(-0.5*pow(sample[i]-left_means[i],2) / pow(left_sds[i], 2));

            keep_p *= (1.0/sqrt(2.0*M_PI*pow(keep_means[i],2)))
                    *exp(-0.5*pow(sample[i]-keep_means[i],2) / pow(keep_sds[i], 2));
            
            right_p *= (1.0/sqrt(2.0*M_PI*pow(right_means[i],2)))
                    *exp(-0.5*pow(sample[i]-right_means[i],2) / pow(right_sds[i], 2));

        }

        // Multiply each by the prior
        left_p *= left_prior;
        keep_p *= keep_prior;
        right_p *= right_prior;

        double probs[3] = {left_p,keep_p,right_p};
        double max = left_p;
        double max_index = 0;

        for(int j=0;j<3; ++j){
            if(probs[j]>max){
                max = probs[j];
                max_index = j;
            }
        }

        return this->possible_labels[max_index];




    }

    //Member Variables
    vector<string> possible_labels = {"left","keep","right"};
    ArrayXd left_means;
    ArrayXd left_sds;
    double left_prior;
    
    ArrayXd keep_means;
    ArrayXd keep_sds;
    double keep_prior;
    
    ArrayXd right_means;
    ArrayXd right_sds;
    double right_prior;

};


// Load state from .txt file
vector<vector<double> > Load_State(string file_name) {
  ifstream in_state_(file_name.c_str(), ifstream::in);
  vector< vector<double >> state_out;
  string line;
    
  while (getline(in_state_, line)) {
    std::istringstream iss(line);
    vector<double> x_coord;
      
    string token;
    while (getline(iss,token,',')) {
      x_coord.push_back(stod(token));
    }
    state_out.push_back(x_coord);
  }

  return state_out;
}

// Load labels from .txt file
vector<string> Load_Label(string file_name) {
  ifstream in_label_(file_name.c_str(), ifstream::in);
  vector< string > label_out;
  string line;
  while (getline(in_label_, line)) {
    std::istringstream iss(line);
    string label;
    iss >> label;
    
    label_out.push_back(label);
  }
    
  return label_out; 
}

int main(){
  vector< vector<double> > X_train = Load_State("./train_states.txt");
  vector< vector<double> > X_test  = Load_State("./test_states.txt");
  vector< string > Y_train = Load_Label("./train_labels.txt");
  vector< string > Y_test  = Load_Label("./test_labels.txt");

  cout << "X_train number of elements " << X_train.size() << endl;
  cout << "X_train element size " << X_train[0].size() << endl;
  cout << "Y_train number of elements " << Y_train.size() << endl; 

  GNB gnb = GNB();

  gnb.train(X_train, Y_train);

  cout << "X_test number of elements " << X_test.size() << endl;
  cout << "X_test element size " << X_test[0].size() << endl;
  cout << "Y_test number of elements " << Y_test.size() << endl;

  int score = 0;
  for(int i = 0; i < X_test.size(); ++i){
      vector<double> coords = X_test[i];
      string predicted = gnb.predict(coords);
      if (predicted.compare(Y_test[i]) == 0) {
        score += 1;
      }
  }

  float fraction_correct = float(score) / Y_test.size();
  cout << "You got " << (100*fraction_correct) << " correct" << endl;

}