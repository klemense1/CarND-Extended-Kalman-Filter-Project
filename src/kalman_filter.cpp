#include "kalman_filter.h"
#include <iostream>

using namespace std;

using Eigen::MatrixXd;
using Eigen::VectorXd;

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
  /*
    * predict the state
  */
    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /*
    * Update the state by using Kalman Filter equations
    * For Lidar measurement update
    * Uses the H matrix for calculating y, S, K and P
  */

    VectorXd z_pred = H_ * x_;
    VectorXd y = z - z_pred;
    MatrixXd Ht = H_.transpose();
    MatrixXd PHt = P_ * Ht;
    MatrixXd S = H_ * PHt + R_;
    MatrixXd Si = S.inverse();
    MatrixXd K = PHt * Si;
    
    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
    * Update the state by using Extended Kalman Filter equations
    * Used for Radar measurement update
    * Directly calculates new measurement y in polar coordinates from h function (instead of y = z - Hj * x)
    * Uses Hjacobian Hj to calculate S, K and P.
  */

    VectorXd y = z - hx_; // in polar coordinates!
    if (y[2] > M_PI) {
        y[2] = y[2] - 2*M_PI;

        cout << "Adjusted phi in y" << endl;
    }
    else if (y[2] < -M_PI) {
        y[2] = y[2] + 2*M_PI;

        cout << "Adjusted phi in y" << endl;
    }

    MatrixXd Hj = H_;
    MatrixXd Hjt = Hj.transpose();
    MatrixXd PHt = P_ * Hjt;
    MatrixXd S = Hj * PHt + R_;
    MatrixXd Si = S.inverse();
    MatrixXd K = PHt * Si;
    
    //new estimate
    x_ = x_ + (K * y); // in cartesian coordinates!
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * Hj) * P_;
}
