#include "kalman_filter.h"
#include <math.h>
#include <iostream>
using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
    VectorXd z_pred = H_ * x_;
    VectorXd y = z - z_pred;
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;
    
    // update x_ and P_
    x_ = x_ + (K * y);
    long x_scale = x_.size();
    MatrixXd I = MatrixXd::Identity(x_scale, x_scale);
    P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
    float rho = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
    float phi = atan2(x_(1), x_(0));
    float rho_dot;
    if (fabs(rho) < 0.0001) {
        rho_dot = 0;
    } else {
        rho_dot = (x_(0)*x_(2) + x_(1)*x_(3))/rho;
    }
    
    // initialize parameters
    VectorXd z_pred(3);
    z_pred << rho, phi, rho_dot;
    VectorXd y = z - z_pred;
    
    // check if Phi of measurement and estimation has different sign. (+PI vs. -PI)
    if (y(1) > 6 || y(1) < -6) {
        cout << "phi value error! difference sign between measurements and estimation: y = " << y(1) << endl;
        y(1) = 0;
    }
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;
    
    // update x_ and P_
    x_ = x_ + (K * y);
    long x_scale = x_.size();
    MatrixXd I = MatrixXd::Identity(x_scale, x_scale);
    P_ = (I - K * H_) * P_;
}
