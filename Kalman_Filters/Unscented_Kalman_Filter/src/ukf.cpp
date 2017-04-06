#include "ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

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

  // State dimension
  int n_x_ = 5;

  // Augmented state dimension
  int n_aug_ = 7;

  // Sigma point spreading parameter
  double lambda_ = 3 - n_aug_ ;

  // initial state vector
  VectorXd x_ = VectorXd(n_x_);

  // initial covariance matrix
  MatrixXd P_ = MatrixXd(n_x_, n_x_);
  P_ <<    0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
          -0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
           0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
          -0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
          -0.0020,    0.0060,    0.0008,    0.0100,    0.0123;

  // initial sigma point matrix
  MatrixXd Xsig_aug_ = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  // initial predicted sigma points matrix
  MatrixXd Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  // initial matrix for sigma points in measurement space
  MatrixXd Zsig_ = MatrixXd(3, 2 * n_aug_ + 1);

  // initial mean predicted measurement
  VectorXd z_pred_ = VectorXd(3);

  // initial measurement covariance matrix S
  MatrixXd S_ = MatrixXd(3,3);

  // time when the state is true, in us
  long long time_us_;

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
  VectorXd weights_ = VectorXd(2 * n_aug_ + 1);
  weights_(0) = lambda_ / (lambda_+n_aug_);
  for (int i=1; i<2*n_aug_+1; i++) {  //2n+1 weights_
    weights_(i) = 0.5/(n_aug_+lambda_);
  }

  // Current NIS for radar
  double NIS_radar_ = 0;

  // Current NIS for laser
  double NIS_laser_ = 0;

  // Count NIS > 7.815 for radar
  int c_NIS_radar_ = 0;

  // Count NIS > 5.991 for laser
  int c_NIS_laser_ = 0;

  // Count all sample for radar
  int total_radar_ = 0;

  // Count all sample for laser
  int total_laser_ = 0;
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
    double vx = 0.0;
    double vy = 0.0;
    double v = 0.0;
    double yaw = 0.0;
    double yawd = 0.0;

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_ == true) {
     /*
      *Convert radar from polar to cartesian coordinates and initialize state.
      */
      px = meas_package.raw_measurements_[0] * cos(meas_package.raw_measurements_[1]);
      py = meas_package.raw_measurements_[0] * sin(meas_package.raw_measurements_[1]); 
      v = meas_package.raw_measurements_[2];
      vx = v * cos(meas_package.raw_measurements_[1]);
      vy = v * sin(meas_package.raw_measurements_[1]);
      yaw = atan2(vy, vx);
      yawd = 0.0; 

      if (px != 0.0 && py != 0.0) {
        x_ << px, py, v, yaw, yawd;
      } else {
        x_ << 0.0001, 0.0001, 0.0, 0.0, 0.0;
      } 
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_ == true) {
      px = meas_package.raw_measurements_[0];
      py = meas_package.raw_measurements_[1];
      vx = 0.1;
      vy = 0.1;
      v = sqrt(vx*vx + vy*vy);
      if (px != 0.0 && py != 0.0) {
        x_ << px, py, v, yaw, yawd;
      } else {
        x_ << 0.0001, 0.0001, 0.0, 0.0, 0.0;
      }
    }

    previous_timestamp_ = meas_package.timestamp_;
    is_initialized_ = true;
    return;
  }

  //compute the time elapsed between the current and previous measurements
  double dt = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0; 
  previous_timestamp_ = meas_package.timestamp_;

  // Generate Augmented Sigma Points
  GenerateAugmentedSigmaPoints();

  // Sigma Point Prediction
  SigmaPointPrediction(dt);

  // Predict Mean and Covariance
  PredictMeanAndCovariance(); 

  // Predict Measurement
  Prediction(meas_package, dt);

  // Update state
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    UpdateRadar(meas_package);
  } else if (meas_package.sensor_type_ == MeasurementPackage::LASER)  {
    UpdateLidar(meas_package);
  }
}

void UKF::GenerateAugmentedSigmaPoints() {
  //create augmented mean state
  VectorXd x_aug = VectorXd(n_aug_);
  x_aug.head(n_x_) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  //create augmented covariance matrix
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_,n_x_) = P_;
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;

  //create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  //create augmented sigma points
  Xsig_aug_.col(0)  = x_aug;
  for (int i = 0; i< n_aug_; i++)
  {
    Xsig_aug_.col(i+1)       = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
    Xsig_aug_.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
  }
}

void UKF::SigmaPointPrediction(double delta_t) {
  //predict sigma points
  for (int i = 0; i< 2*n_aug_+1; i++)
  {
    //extract values for better readability
    double p_x = Xsig_aug_(0,i);
    double p_y = Xsig_aug_(1,i);
    double v = Xsig_aug_(2,i);
    double yaw = Xsig_aug_(3,i);
    double yawd = Xsig_aug_(4,i);
    double nu_a = Xsig_aug_(5,i);
    double nu_yawdd = Xsig_aug_(6,i);

    //predicted state values
    double px_p, py_p;

    //avoid division by zero
    if (fabs(yawd) > 0.001) {
        px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    }
    else {
        px_p = p_x + v*delta_t*cos(yaw);
        py_p = p_y + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    //add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    //write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }
}

void UKF::PredictMeanAndCovariance() {
  //create vector for predicted state
  VectorXd x = VectorXd(n_x_);

  //create covariance matrix for prediction
  MatrixXd P = MatrixXd(n_x_, n_x_);

  //predicted state mean
  x.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    x = x+ weights_(i) * Xsig_pred_.col(i);
  }

  //predicted state covariance matrix
  P.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    P = P + weights_(i) * x_diff * x_diff.transpose() ;
  }

  x_ = x;
  P_ = P;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(MeasurementPackage meas_package, double delta_t) {
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    //transform sigma points into measurement space
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
      // extract values for better readibility
      double p_x = Xsig_pred_(0,i);
      double p_y = Xsig_pred_(1,i);
      double v  = Xsig_pred_(2,i);
      double yaw = Xsig_pred_(3,i);

      double v1 = cos(yaw)*v;
      double v2 = sin(yaw)*v;

      // measurement model
      Zsig_(0,i) = sqrt(p_x*p_x + p_y*p_y); //r
      Zsig_(1,i) = atan2(p_y,p_x);          //phi
      if (Zsig_(0,i) > 0) {                               
        Zsig_(2,i) = (p_x*v1 + p_y*v2 ) / Zsig_(0,i);   //r_dot
      } else {
        Zsig_(2,i) = 0;
      } 
    } 
  } else {
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
      Zsig_(0,i) = Xsig_pred_(0,i);
      Zsig_(1,i) = Xsig_pred_(1,i);  
    }
  }

  //mean predicted measurement
  z_pred_.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; i++) {
      z_pred_ = z_pred_ + weights_(i) * Zsig_.col(i);
  }

  //measurement covariance matrix S
  S_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig_.col(i) - z_pred_;

    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S_ = S_ + weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(3,3);
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    R <<    std_laspx_*std_laspx_, 0,
            0,std_laspy_*std_laspy_;
  } else {
    R <<    std_radr_*std_radr_, 0, 0,
            0, std_radphi_*std_radphi_, 0,
            0, 0,std_radrd_*std_radrd_;
  }
  S_ = S_ + R;
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  // Extarct the measurement
  VectorXd z = meas_package.raw_measurements_;

  //calculate cross correlation matrix
  MatrixXd Tc = MatrixXd(n_x_, 2);
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig_.col(i) - z_pred_;
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S_.inverse();

  //residual
  VectorXd z_diff = z - z_pred_;

  //angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S_*K.transpose();

  // Calculate NIS
  total_laser_++;
  NIS_laser_ = z_diff.transpose() * S_.inverse() * z_diff;
  if ( NIS_laser_ > 5.991)
    c_NIS_laser_ ++;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  // Extarct the measurement
  VectorXd z = meas_package.raw_measurements_ ;
 
  //calculate cross correlation matrix
  MatrixXd Tc = MatrixXd(n_x_, 3);
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig_.col(i) - z_pred_;
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S_.inverse();

  //residual
  VectorXd z_diff = z - z_pred_;

  //angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S_*K.transpose();

  // Calculate NIS
  total_radar_ ++;
  NIS_radar_ = z_diff.transpose() * S_.inverse() * z_diff;
  if ( NIS_radar_ > 7.815)
    c_NIS_radar_ ++;
}
