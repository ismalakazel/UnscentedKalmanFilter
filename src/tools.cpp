#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth) {
    
    // Build root mean square error vector
    VectorXd rmse(4);
    rmse << 0,0,0,0;
    
    // Check validity of inputs
    if(estimations.size() == 0 or estimations.size() != ground_truth.size()){
        cout << "Error in inputs" << endl;
        return rmse;
    }
    
    // Build residuals vector
    VectorXd residuals(4);
    for(int i=0; i < estimations.size(); ++i){
        
        // Update residuals vector
        residuals = estimations[i] - ground_truth[i];
        residuals = residuals.array() * residuals.array();
        
        // Update rmse vector
        rmse = rmse + residuals;
    }
    
    // Calculate the mean
    rmse = rmse / estimations.size();
    rmse = rmse.array().sqrt();
    
    return rmse;
}
