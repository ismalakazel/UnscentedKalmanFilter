#include "UnscentedKalmanFilter.h"

MatrixXd Filter::GenerateSigmaPoints(VectorXd *x, MatrixXd *P, double lambda) {
    
    // State dimension
    int xsize = x.size();
    
    // Create sigma point matrix
    MatrixXd xsigma = MatrixXd(xsize, 2 * xsize + 1);
    
    // Calculate square root of P
    MatrixXd A = P.llt().matrixL();
    
    // Set first column of sigma point matrix
    xsigma.col(0)  = x;
    
    // Set remaining sigma points
    for (int i = 0; i < xsize; i++) {
        xsigma.col(i + 1) = x + sqrt(lambda + xsize) * A.col(i);
        xsigma.col(i + 1 + xsize) = x - sqrt(lambda + xsize) * A.col(i);
    };
    
    return xsigma;
};
