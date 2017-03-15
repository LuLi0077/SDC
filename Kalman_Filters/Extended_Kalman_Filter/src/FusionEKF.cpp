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

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);
  MatrixXd P_ = MatrixXd(4, 4);
  MatrixXd F_ = MatrixXd(4, 4);
  MatrixXd Q_ = MatrixXd(4, 4);

 /*
  * Set the process and measurement noises
  */
  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  H_laser_ << 1, 0, 0, 0,
        0, 1, 0, 0;

  Hj_ << 0, 0, 0, 0,
        1e+9, 1e+9, 0, 0,
        0, 0, 0, 0;

  P_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1000, 0,
        0, 0, 0, 1000;

  F_ << 1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1;
  
  VectorXd x_ = VectorXd(4);
  ekf_.Init(x_, P_, F_, H_laser_, R_laser_, Q_);

  float noise_ax = 9;
  float noise_ay = 9;
}

/*
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
   /*
    * Initialize the state ekf_.x_ with the first measurement.
    * Create the covariance matrix.
    * Convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;
    // Initialize only if initial state is non-zero
//    if(x_in.norm() > 1e-4)
//    {
//      previous_timestamp_ = measurement_pack.timestamp_;
      // done initializing, no need to predict or update
//      is_initialized_ = true;
//      return;
//    }
    float px = 0;
    float py = 0;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
     /*
      *Convert radar from polar to cartesian coordinates and initialize state.
      */
      float px = measurement_pack.raw_measurements_[0] * cos(measurement_pack.raw_measurements_[1]);
      float py = measurement_pack.raw_measurements_[0] * sin(measurement_pack.raw_measurements_[1]);

      if (px == 0 or px == 0){
        return;
      }

      
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      float px = measurement_pack.raw_measurements_[0];
      float py = measurement_pack.raw_measurements_[1];
    }

    ekf_.x_ << px, py, 0, 0;

    // Create the covariance matrix.
    //state covariance matrix P
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 1, 0, 0, 0,
               0, 1, 0, 0,
               0, 0, 1000, 0,
               0, 0, 0, 1000;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    previous_timestamp_ = measurement_pack.timestamp_;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /*
   * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
   * Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for Q matrix.
   * If two measurements coming in at the same time, use both of them. 
   */

  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0; 
  float noise_ax = 9;
  float noise_ay = 9; 

  if(dt > 0.001){

        ekf_.F_ <<  1, 0, dt, 0,
                0, 1, 0, dt,
                0, 0, 1, 0,
                0, 0, 0, 1;

        ekf_.Q_ <<  pow(dt, 4) / 4 * noise_ax, 0, pow(dt, 3) / 2 * noise_ax, 0,
                0, pow(dt, 4) / 4 * noise_ay, 0, pow(dt, 3) / 2 * noise_ay,
                pow(dt, 3) / 2 * noise_ax, 0, pow(dt, 2) * noise_ax, 0,
                0, pow(dt, 3) / 2 * noise_ay, 0, pow(dt, 2) * noise_ay;

        ekf_.Predict();
  }

  previous_timestamp_ = measurement_pack.timestamp_;

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /*
   * Use the sensor type to perform the update step.
   * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    ekf_.R_ = R_radar_;
    ekf_.H_= tools.CalculateJacobian(ekf_.x_);
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    ekf_.R_ = R_laser_;
    ekf_.H_ = H_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
