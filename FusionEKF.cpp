#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h" 

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd::Zero(2, 2);
  R_radar_ = MatrixXd::Zero(3, 3);
  H_laser_ = MatrixXd::Zero(2, 4);
  

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;
  //Setting up Measurement Function for Laser
  H_laser_(0,0) = 1;
  H_laser_(1,1) = 1;

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */


}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;
    
    //Create state covariance matrix
    ekf_.P_ = MatrixXd(4,4);
    ekf_.P_ << 1, 0, 0, 0,
               0, 1, 0, 0,
               0, 0, 1000, 0,
               0, 0, 0, 1000;
    //initial transition matrix
    ekf_.F_ = MatrixXd(4,4);
    ekf_.F_ << 1, 0, 1, 0,
               0, 1, 0, 1,
               0, 0, 1, 0,
               0, 0, 0, 1;
    //Initializing Process Covariance Matrix with Zero
    ekf_.Q_ = MatrixXd::Zero(4, 4);

        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
          // TODO: Convert radar from polar to cartesian coordinates 
          //         and initialize state.
          float rho    = measurement_pack.raw_measurements_(0);
          float phi    = measurement_pack.raw_measurements_(1);

          ekf_.x_(0) = rho * cos(phi);
          ekf_.x_(1) = rho * sin(phi);
          ekf_.x_(2) = 0;
          ekf_.x_(3) = 0;
        }
        else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
          // TODO: Initialize state. 
          ekf_.x_(0) = measurement_pack.raw_measurements_(0);
          ekf_.x_(1) = measurement_pack.raw_measurements_(1);
          ekf_.x_(2) = 0;
          ekf_.x_(3) = 0;
        }
    // done initializing, no need to predict or update
    is_initialized_     = true;
    previous_timestamp_ = measurement_pack.timestamp_;
    return;
  }

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  float noise_ax = 9;
  float noise_ay = 9;  
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  
  float dt2 = dt*dt;
  float dt3_by_2 = (dt2 * dt)/2;
  float dt4_by_4 = (dt2 * dt2)/4;
  
  // TODO: YOUR CODE HERE
  // 1. Modify the F matrix so that the time is integrated
  ekf_.F_(0,2) = dt;
  ekf_.F_(1,3) = dt;
  // 2. Set the process covariance matrix Q
  ekf_.Q_ = MatrixXd::Zero(4,4);
  //Row 1
  ekf_.Q_(0,0) = dt4_by_4 * noise_ax;
  ekf_.Q_(0,2) = dt3_by_2 * noise_ax;
  //Row 2
  ekf_.Q_(1,1) = dt4_by_4 * noise_ay;
  ekf_.Q_(1,3) = dt3_by_2 * noise_ay;
  //Row 3
  ekf_.Q_(2,0) = dt3_by_2 * noise_ax;
  ekf_.Q_(2,2) = dt2 * noise_ax;
  //Row 4
  ekf_.Q_(3,1) = dt3_by_2 * noise_ay;
  ekf_.Q_(3,3) = dt2 * noise_ay;
  // 3. Call the Kalman Filter predict() function
  ekf_.Predict();

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates
    Tools tools;
    ekf_.R_ = R_radar_;
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    
  } else {
    // TODO: Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);

  }

  // print the output
  //cout << "x_ = " << ekf_.x_ << endl;
  //cout << "P_ = " << ekf_.P_ << endl;
}
