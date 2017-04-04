#include <iostream>
#include "ukf.h"

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  is_initialized_ = false;
  previous_timestamp_ = 0.0;

  // if this is false, laser measurements will be ignored (except during init)
  bool use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  bool use_radar_ = true;

  // initial state vector
  VectorXd x_ = VectorXd(5);

  // initial covariance matrix
  MatrixXd P_ = MatrixXd(5, 5);

  // initial predicted sigma points matrix
  MatrixXd Xsig_pred_ = MatrixXd();

  // time when the state is true, in us
  long time_us_;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_ = 3;

  // Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_ = 0.5;

  // Laser measurement noise standard deviation position1 in m
  double std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  double std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  double std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  double std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ = 0.3;

  // Weights of sigma points
  VectorXd weights_;

  // State dimension
  int n_x_ = 5;

  // Augmented state dimension
  int n_aug_;

  // Sigma point spreading parameter
  double lambda_ = 3 - n_x_;

  // the current NIS for radar
  double NIS_radar_;

  // the current NIS for laser
  double NIS_laser_;
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  if (!is_initialized_) {

    double px = 0.0;
    double py = 0.0;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR && use_radar_ == true) {
     /*
      *Convert radar from polar to cartesian coordinates and initialize state.
      */
      double px = measurement_pack.raw_measurements_[0] * cos(measurement_pack.raw_measurements_[1]);
      double py = measurement_pack.raw_measurements_[0] * sin(measurement_pack.raw_measurements_[1]); 
      double vx = measurement_pack.raw_measurements_[2] * cos(measurement_pack.raw_measurements_[1]);
      double vy = measurement_pack.raw_measurements_[2] * sin(measurement_pack.raw_measurements_[1]);
      ekf_.x_ << px, py, vx, vy;    
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER && use_laser_ == true) {
      double px = measurement_pack.raw_measurements_[0];
      double py = measurement_pack.raw_measurements_[1];
      ekf_.x_ << px, py, 0.0, 0.0;
    }

    previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    return;
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
}
