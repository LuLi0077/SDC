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
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // State dimension
  n_x_ = 5;

  // Augmented state dimension
  n_aug_ = 7;

  // Sigma point spreading parameter
  lambda_ = 3 - n_aug_ ;

  // initial state vector
  x_ = VectorXd(n_x_);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);
  P_ <<   1, 0, 0, 0, 0,
          0, 1, 0, 0, 0,
          0, 0, 1, 0, 0,
          0, 0, 0, 1, 0,
          0, 0, 0, 0, 1;

  // initial sigma point matrix
  Xsig_aug_ = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  // initial predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 0.3;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.5;

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  // Weights of sigma points
  weights_ = VectorXd(2 * n_aug_ + 1);
  weights_(0) = lambda_ / (lambda_+n_aug_);
  for (int i=1; i<2*n_aug_+1; i++) {  //2n+1 weights_
    weights_(i) = 0.5/(n_aug_+lambda_);
  }

  // Current NIS for radar
  NIS_radar_ = 0;

  // Current NIS for laser
  NIS_laser_ = 0;
}

UKF::~UKF() {}

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

      px = meas_package.raw_measurements_[0] * cos(meas_package.raw_measurements_[1]);
      py = meas_package.raw_measurements_[0] * sin(meas_package.raw_measurements_[1]);       
      vx = meas_package.raw_measurements_[2] * cos(meas_package.raw_measurements_[1]);
      vy = meas_package.raw_measurements_[2] * sin(meas_package.raw_measurements_[1]);
      v = fabs(meas_package.raw_measurements_[2]);
      yaw = atan(meas_package.raw_measurements_[1]);
      yawd = 0.0; 

      if(vx !=0){
        yaw = vy/vx;
      }
      if (px == 0.0) {
        px = 0.0001;
      } 
      if (py == 0.0) {
        py = 0.0001;
      }       
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_ == true) {
      px = meas_package.raw_measurements_[0];
      py = meas_package.raw_measurements_[1];
      v = 0.0;
      yaw = 0.0;
      yawd = 0.0; 

      if (px == 0.0) {
        px = 0.0001;
      } 
      if (py == 0.0) {
        py = 0.0001;
      }       
    }
    
    x_ << px, py, v, yaw, yawd;
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

  // Predict Measurement & Update state
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_ == true) {
    RADAR(meas_package, dt);
  } else if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_ == true)  {
    LASER(meas_package, dt);
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
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;

  //create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  //create augmented sigma points
  Xsig_aug_.col(0)  = x_aug;
  for (int i = 0; i < n_aug_; i++)
  {
    Xsig_aug_.col(i+1)       = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
    Xsig_aug_.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
  }
}

void UKF::SigmaPointPrediction(double delta_t) {
  //predict sigma points
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
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
        px_p = p_x + v/yawd * (sin (yaw + yawd*delta_t) - sin(yaw));
        py_p = p_y + v/yawd * (cos(yaw) - cos(yaw+yawd*delta_t));
    } else {
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
  //predicted state mean
  x.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    x += weights_(i) * Xsig_pred_.col(i);
  }

  //create covariance matrix for prediction
  MatrixXd P = MatrixXd(n_x_, n_x_);
  //predicted state covariance matrix
  P.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    P += weights_(i) * x_diff * x_diff.transpose() ;
  }

  x_ = x;
  P_ = P;
}

void UKF::RADAR(MeasurementPackage meas_package, double delta_t) {
  // initial matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(3, 2 * n_aug_ + 1);
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
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y); //r
    Zsig(1,i) = atan2(p_y,p_x);          //phi
    if (Zsig(0,i) > 0.001) {                               
      Zsig(2,i) = (p_x*v1 + p_y*v2 ) / Zsig(0,i);   //r_dot
    } else {
      Zsig(2,i) = (p_x*v1 + p_y*v2 ) / 0.001;
    } 
  } 

  // initial mean predicted measurement
  VectorXd z_pred = VectorXd(3);
  //mean predicted measurement
  z_pred.fill(0.0);
  for (int i=0; i < 2 * n_aug_ + 1; i++) {
      z_pred += weights_(i) * Zsig.col(i);
  }

  // initial measurement covariance matrix S
  MatrixXd S = MatrixXd(3, 3);	
  //measurement covariance matrix S
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S += weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(3,3);
  R <<  std_radr_*std_radr_, 0, 0,
        0, std_radphi_*std_radphi_, 0,
        0, 0,std_radrd_*std_radrd_;
  S = S + R;

  // Extarct the measurement
  VectorXd z = meas_package.raw_measurements_ ;
 
  //calculate cross correlation matrix
  MatrixXd Tc = MatrixXd(n_x_, 3);
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
  	//residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc += weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = z - z_pred;

  //angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();

  // Calculate NIS
  NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;
}

void UKF::LASER(MeasurementPackage meas_package, double delta_t) {
  // initial matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(2, 2 * n_aug_ + 1);
  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
      Zsig(0,i) = Xsig_pred_(0,i);
      Zsig(1,i) = Xsig_pred_(1,i);  
  } 

  // initial mean predicted measurement
  VectorXd z_pred = VectorXd(2);
  //mean predicted measurement
  z_pred.fill(0.0);
  for (int i=0; i < 2 * n_aug_ + 1; i++) {
      z_pred += weights_(i) * Zsig.col(i);
  }

  // initial measurement covariance matrix S
  MatrixXd S = MatrixXd(2, 2);	
  //measurement covariance matrix S
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S += weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(2,2);
  R <<  std_laspx_*std_laspx_, 0,
        0,std_laspy_*std_laspy_;
  S = S + R;

  // Extarct the measurement
  VectorXd z = meas_package.raw_measurements_;
 
  //calculate cross correlation matrix
  MatrixXd Tc = MatrixXd(n_x_, 2);
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
  	//residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc += weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = z - z_pred;

  //angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();

  // Calculate NIS
  NIS_laser_ = z_diff.transpose() * S.inverse() * z_diff;
}

