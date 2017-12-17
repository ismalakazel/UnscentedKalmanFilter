#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>


using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;


/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
    // if this is false, laser measurements will be ignored (except during init)
    use_laser_ = true;
    
    // if this is false, radar measurements will be ignored (except during init)
    use_radar_ = true;
    
    // initial state vector
    x_ = VectorXd(5);
    
    // initial covariance matrix
    P_ = MatrixXd(5, 5);
    
    // Process noise standard deviation longitudinal acceleration in m/s^2
    std_a_ = 30;
    
    // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 30;
    
    // Laser measurement noise standard deviation position1 in m
    std_laspx_ = 0.15;
    
    // Laser measurement noise standard deviation position2 in m
    std_laspy_ = 0.15;
    
    // Radar measurement noise standard deviation radius in m
    std_radr_ = 0.3;
    
    // Radar measurement noise standard deviation angle in rad
    std_radphi_ = 0.0175;
    
    // Radar measurement noise standard deviation radius change in m/s
    std_radrd_ = 0.1;
    
    // Application initialization flag
    is_initialized_ = false;
    
    // Time when state is true
    time_us_ = 0.0;
    
    // State dimension
    n_x_ = 5;
    
    // Augmented state dimension
    n_aug_ = n_x_ + 2;
    
    // Sigma point spreading parameter
    lambda_ = 3 - n_aug_;
    
    // Predicted sigma points matrix
    Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
    
    // Weights of sigma points
    weights_ = VectorXd(2 * n_aug_ + 1);
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
    if (!is_initialized_) {
        
        // Mark application as initialized
        is_initialized_ = true;
        
        // Register first measurement timestamp
        time_us_ = meas_package.timestamp_;
        
        // Process initial measurement
        meas_package.process(x_, use_radar_, use_laser_);
        
        return;
    }
    
    // Calculate delta time
    float dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
    
    // Update last measurement timestamp
    time_us_ = meas_package.timestamp_;
    
    // Prediction step
    Prediction(dt);
    
    // Update step
    if (use_radar_ and meas_package.sensor_type_ == MeasurementPackage::RADAR) {
        UpdateRadar(meas_package);
    }
    else if (use_laser_ and meas_package.sensor_type_ == MeasurementPackage::LASER) {
        UpdateLidar(meas_package);
    }
}


MatrixXd UKF::AugmentedSigmaPoints() {
    
    // Create augmented mean vector
    VectorXd x_aug = VectorXd(n_aug_);
    
    // Create augmented mean state
    x_aug.head(5) = this->x_;
    x_aug(5) = 0;
    x_aug(6) = 0;
    
    // Create augmented state covariance
    MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
    
    // Create augmented covariance matrix
    P_aug.fill(0.0);
    P_aug.topLeftCorner(n_x_, n_x_) = P_;
    P_aug(5,5) = std_a_ * std_a_;
    P_aug(6,6) = std_yawdd_ * std_yawdd_;
    
    // Create square root matrix
    MatrixXd L = P_aug.llt().matrixL();
    
    // Create sigma point matrix
    MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
    
    // Create augmented sigma points
    Xsig_aug.col(0) = x_aug;
    for (int i = 0; i < n_aug_; i++) {
        Xsig_aug.col(i + 1) = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
        Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
    }
    
    return Xsig_aug;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
    
    // Create sigma points
    MatrixXd Xsig_aug = AugmentedSigmaPoints();
    
    // Predict sigma points
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        
        // Extract values for better readability
        double p_x = Xsig_aug(0, i);
        double p_y = Xsig_aug(1, i);
        double v = Xsig_aug(2, i);
        double yaw = Xsig_aug(3, i);
        double yawd = Xsig_aug(4, i);
        double nu_a = Xsig_aug(5, i);
        double nu_yawdd = Xsig_aug(6, i);
        
        // Predicted state values
        double px_p, py_p;
        
        // Avoid division by zero
        if (fabs(yawd) > 0.001) {
            px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
            py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
        } else {
            px_p = p_x + v * delta_t * cos(yaw);
            py_p = p_y + v * delta_t * sin(yaw);
        }
        
        double v_p = v;
        double yaw_p = yaw + yawd * delta_t;
        double yawd_p = yawd;
        
        // Add noise
        px_p = px_p + 0.5 * nu_a * delta_t * delta_t * cos(yaw);
        py_p = py_p + 0.5 * nu_a * delta_t * delta_t * sin(yaw);
        v_p = v_p + nu_a * delta_t;
        yaw_p = yaw_p + 0.5 * nu_yawdd*delta_t * delta_t;
        yawd_p = yawd_p + nu_yawdd * delta_t;
        
        // Write predicted sigma point into right column
        Xsig_pred_(0, i) = px_p;
        Xsig_pred_(1, i) = py_p;
        Xsig_pred_(2, i) = v_p;
        Xsig_pred_(3, i) = yaw_p;
        Xsig_pred_(4, i) = yawd_p;
    }
    
    // Set weights
    double weight_0 = lambda_ / (lambda_ + n_aug_);
    weights_(0) = weight_0;
    for (int i = 1; i < 2 * n_aug_ + 1; i++) {  //2n+1 weights
        double weight = 0.5 / (n_aug_ + lambda_);
        weights_(i) = weight;
    }
    
    // Create vector for predicted state
    VectorXd x_pred = VectorXd(n_x_);

    // Predicted state mean
    x_pred.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
        x_pred = x_pred + weights_(i) * Xsig_pred_.col(i);
    }
    
    x_pred = Xsig_pred_ * weights_;
    
    // Create covariance matrix for prediction
    MatrixXd P_pred = MatrixXd(n_x_, n_x_);

    // Predicted state covariance matrix
    P_pred.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
        
        // State difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        
        // Angle normalization
        while (x_diff(3) > M_PI) x_diff(3) -=2. * M_PI;
        while (x_diff(3) < -M_PI) x_diff(3) +=2. * M_PI;
        
        P_pred = P_pred + weights_(i) * x_diff * x_diff.transpose() ;
    }
    
    x_ = x_pred;
    P_ = P_pred;
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {

}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
    
    // Set measurement dimension, radar can measure r, phi, and r_dot
    int n_z = 3;
    
    // Create matrix for sigma points in measurement space
    MatrixXd Zsig = MatrixXd(n_z, n_sig_);
    
    // Transform sigma points into measurement space
    for (int i = 0; i < 2 * n_aug + 1; i++) {
        
        // Extract values for better readibility
        double p_x = Xsig_pred(0, i);
        double p_y = Xsig_pred(1, i);
        double v  = Xsig_pred(2, i);
        double yaw = Xsig_pred(3, i);
        
        double v1 = cos(yaw) * v;
        double v2 = sin(yaw) * v;
        
        // Measurement model
        Zsig(0, i) = sqrt(p_x * p_x + p_y * p_y);
        Zsig(1, i) = atan2(p_y, p_x);
        Zsig(2, i) = (p_x * v1 + p_y * v2 ) / sqrt(p_x * p_x + p_y * p_y);
    }
    
    // Mean predicted measurement
    VectorXd z_pred = VectorXd(n_z);
    z_pred.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        z_pred = z_pred + weights(i) * Zsig.col(i);
    }
    
    // Innovation covariance matrix S
    MatrixXd S = MatrixXd(n_z,n_z);
    S.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        //residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        
        //angle normalization
        while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
        while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
        
        S = S + weights(i) * z_diff * z_diff.transpose();
    }
    
    // Add measurement noise covariance matrix
    MatrixXd R = MatrixXd(n_z,n_z);
    R << std_radr_ * std_radr_, 0, 0,
         0, std_radphi_ * std_radphi_, 0,
         0, 0, std_radrd_ * std_radrd_;
    
    S = S + R;
    
    // Create vector for incoming radar measurement
    VectorXd z = VectorXd(n_z);
}

