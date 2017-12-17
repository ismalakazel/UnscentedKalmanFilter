#include "measurement_package.h"


/**
 @description Add sensor measurements to the state vector
 @param &x The state vector
 @param radar A boolean indicating if radar measurements should be processed
 @param laser A boolean indicating if laser measurements should be processed
 */
void MeasurementPackage::process(Eigen::VectorXd &x, bool radar, bool laser) {
    if (radar and this->sensor_type_ == MeasurementPackage::RADAR) {
        
        // Process radar measurements
        radarMeasurement(x);
    } else if (laser and this->sensor_type_ == MeasurementPackage::LASER) {
        
        // Process laser measurements
        laserMeasurement(x);
    };
}

/**
 @description Adds radar measurements to the state vector x. Converts coordinates from polar to cartesian
 @param &x The state vector
 */
void MeasurementPackage::radarMeasurement(Eigen::VectorXd &x) {
    
    // Range
    float rho = this->raw_measurements_[0];
    
    // Bearing
    float phi = this->raw_measurements_[1];
    
    // Velocity
    float rho_dot = this->raw_measurements_[2];
    
    // X position
    float px = rho * cos(phi);
    
    // Y position
    float py = rho * sin(phi);
    
    // Velocity on x
    float vx = rho_dot * cos(phi);
    
    // Velocity on y
    float vy = rho_dot * sin(phi);
    
    // Velocity
    float v  = sqrt(vx * vx + vy * vy);
    
    // Update state vector
    x << px, py, v, 0, 0;
};

/**
 @description Adds lidar measurements to the state vector x.
 @param &x The state vector.
 */
void MeasurementPackage::laserMeasurement(Eigen::VectorXd &x) {
    
    // X position
    float px = this->raw_measurements_[0];
    
    // Y position
    float py = this->raw_measurements_[1];
    
    // Update state vector
    x << px, py, 0, 0, 0;
}
