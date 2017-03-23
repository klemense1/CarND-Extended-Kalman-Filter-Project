#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
    is_initialized_ = false;

    previous_timestamp_ = 0;

    //measurement covariance matrix - laser
    R_laser_ = MatrixXd(2, 2);
    R_laser_ << 0.0225, 0,
                0, 0.0225;

    //measurement covariance matrix - radar
    R_radar_ = MatrixXd(3, 3);
    R_radar_ << 0.09, 0, 0,
                0, 0.0009, 0,
                0, 0, 0.09;

    // Set the process and measurement noises
    H_laser_ = MatrixXd(2, 4);
    H_laser_ << 1, 0, 0, 0,
                0, 1, 0, 0;

    Hj_ = MatrixXd(3, 4);
    Hj_ <<  0, 0, 0, 0,
            0, 0, 0, 0,
            0, 0, 0, 0;

    //set the acceleration noise components
    noise_ax = 9;
    noise_ay = 9;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


    /*****************************************************************************
     *  INITIALIZATION
     ****************************************************************************/
    if (!is_initialized_) {
        /**
          * Initializing the state ekf_.x_ with the first measurement
          * Creating the covariance matrix
          * Converting radar from polar to cartesian coordinates
        */
        // first measurement
        cout << "EKF: " << endl;
        ekf_.x_ = VectorXd(4);
        ekf_.P_ = MatrixXd(4, 4);
        ekf_.F_ = MatrixXd(4, 4);
        ekf_.Q_ = MatrixXd(4, 4);

        //state covariance matrix P_
        ekf_.P_ <<  1, 0, 0, 0,
                    0, 1, 0, 0,
                    0, 0, 1000, 0,
                    0, 0, 0, 1000;

        //the initial transition matrix F_
        ekf_.F_ << 1, 0, 1, 0,
                0, 1, 0, 1,
                0, 0, 1, 0,
                0, 0, 0, 1;

        // initial process covariance matrix Q_
        ekf_.Q_ << 0, 0, 0, 0,
                0, 0, 0, 0,
                0, 0, 0, 0,
                0, 0, 0, 0;

        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
            /**
            Convert radar data from polar to cartesian coordinates and initialize state.
            */

            float rho = measurement_pack.raw_measurements_[0];
            float phi = measurement_pack.raw_measurements_[1];

            float px = rho*cos(phi);
            float py = rho*sin(phi);

            ekf_.x_ << px, py, 0, 0;
            previous_timestamp_ = measurement_pack.timestamp_;

        } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
            /**
            Initialize state for Laser Data
            */

            float px = measurement_pack.raw_measurements_[0];
            float py = measurement_pack.raw_measurements_[1];

            ekf_.x_ << px, py, 0, 0;
            previous_timestamp_ = measurement_pack.timestamp_;

        }

        // done initializing, no need to predict or update
        is_initialized_ = true;
        return;
    }

    /*****************************************************************************
     *  PREDICTION
     ****************************************************************************/

    /**
       * Update the state transition matrix F according to the new elapsed time.
       * Update the process noise covariance matrix Q (use noise_ax = 9 and noise_ay = 9)
     */

    // compute the time elapsed between the current and previous measurements (convert timestamp to seconds)
    float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;    //dt - expressed in seconds
    previous_timestamp_ = measurement_pack.timestamp_;

    float dt_2 = dt * dt;
    float dt_3 = dt_2 * dt;
    float dt_4 = dt_3 * dt;

    // F matrix is modified so that the time is integrated
    ekf_.F_(0, 2) = dt;
    ekf_.F_(1, 3) = dt;

    cout << "F_ = " << ekf_.F_ << endl;

    // update process covariance matrix Q
    ekf_.Q_ = MatrixXd(4, 4);
    ekf_.Q_ << dt_4 / 4 * noise_ax, 0, dt_3 / 2 * noise_ax, 0,
               0, dt_4 / 4 * noise_ay, 0, dt_3 / 2 * noise_ay,
               dt_3 / 2 * noise_ax, 0, dt_2 * noise_ax, 0,
               0, dt_3 / 2 * noise_ay, 0, dt_2 * noise_ay;

    // call Predict function from kalman_filter.cpp
    ekf_.Predict();

    // print the output
    cout << "x_ = " << ekf_.x_ << endl;
    cout << "P_ = " << ekf_.P_ << endl;

    /*****************************************************************************
     *  UPDATE
     ****************************************************************************/

    /**
       * Use the sensor type to perform the update step.
       * Update the state and covariance matrices.
     */

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        ekf_.hx_ = VectorXd(3);

        // Radar updates
        double px = ekf_.x_(0);
        double py = ekf_.x_(1);
        double vx = ekf_.x_(2);
        double vy = ekf_.x_(3);

        double rho = sqrt(px*px + py*py);
        double phi = fabs(px) > 0.0001 ? atan2(py, px) : 0;
        double rho_dot = fabs(rho) > 0.0001 ? (px*vx + py*vy)/rho : 0;

        // predicted state x_ is mapped into measurement space
        ekf_.hx_ << rho, phi, rho_dot;

        // set matrices for Radar update
        ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
        ekf_.R_ = R_radar_;

        ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    } else {
        // set matrices for Laser updates
        ekf_.H_ = H_laser_;
        ekf_.R_ = R_laser_;

        ekf_.Update(measurement_pack.raw_measurements_);
    }

    // print the output
    cout << "x_ = " << ekf_.x_ << endl;
    cout << "P_ = " << ekf_.P_ << endl;
}
