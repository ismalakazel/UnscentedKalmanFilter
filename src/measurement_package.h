#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

#include "Eigen/Dense"

class MeasurementPackage {
public:
    long timestamp_;
    
    enum SensorType{
        LASER,
        RADAR
    } sensor_type_;
    
    Eigen::VectorXd raw_measurements_;
    
    /**
     @description Add sensor measurements to the state vector
     @param &x The state vector
     @param radar A boolean indicating if radar measurements should be processed
     @param laser A boolean indicating if laser measurements should be processed
     */
    void process(Eigen::VectorXd &x, bool radar, bool laser);
private:
    
    /**
     @description Adds radar measurements to the state vector x. Converts coordinates from polar to cartesian
     @param &x The state vector
    */
    void radarMeasurement(Eigen::VectorXd &x);
    
    /**
     @description Adds lidar measurements to the state vector x.
     @param &x The state vector.
     */
    void laserMeasurement(Eigen::VectorXd &x);
};

#endif /* MEASUREMENT_PACKAGE_H_ */

