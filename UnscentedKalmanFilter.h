#ifndef UnscentedKalmanFilter_hpp
#define UnscentedKalmanFilter_hpp

class Filter {
public:
    
    /**
     @description generated sigma points
     @param x The state vector
     @param P The covariance matrix
     @param lambda The spreading factor
     */
    MatrixXd GenerateSigmaPoints(VectorXd *x, MatrixXd P, double lambda);
}

#endif /* UnscentedKalmanFilter_hpp */
