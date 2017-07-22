#include "ukf.h"
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
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
    ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_ = false;

  ///* predicted sigma points matrix
  MatrixXd Xsig_pred_;

  ///* time when the state is true, in us
  long long time_us_ = 0;

  ///* Weights of sigma points
  VectorXd weights_;
  
  double weight_0 = lambda/(lambda+n_aug);
  weights(0) = weight_0;
  for (int i=1; i<2*n_aug+1; i++) {  //2n+1 weights
    double weight = 0.5/(n_aug+lambda);
    weights(i) = weight;
  }

  ///* State dimension
  int n_x_ = 5;

  ///* Augmented state dimension
  int n_aug_ = 7;

  ///* Sigma point spreading parameter
  double lambda_ = 3 - n_aug;
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  if (!is_initialized_) {
    /**
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    float px_init;
    float py_init;
    float vx_init = 0.0;
    float vy_init = 0.0;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      float r_init = measurement_pack.raw_measurements_[0];
      float theta_init = measurement_pack.raw_measurements_[1];
      px_init = r_init * cos(theta_init);
      py_init = r_init * sin(theta_init);
      
      // TODO: Improve how to translate rho and theta initial uncertainty into
      //       px and py initial uncertainty.
      P_init << std_radr_,         0,    0,    0,    0,
                        0, std_radr_,    0,    0,    0,
                        0,         0, 1000,    0,    0,
                        0,         0,    0, 1000,    0,
                        0,         0,    0,    0, 1000;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      px_init = measurement_pack.raw_measurements_[0];
      py_init = measurement_pack.raw_measurements_[1];

      P_init << std_laspx_,          0,    0,    0,    0,
                         0, std_laspy_,    0,    0,    0,
                         0,          0, 1000,    0,    0,
                         0,          0,    0, 1000,    0,
                         0,          0,    0,    0, 1000;
    }

    if (px_init < 0.0001 && py_init < 0.0001) return;

    x_ << px_init, py_init, vx_init, vy_init;
    time_us_ = measurement_pack.timestamp_;
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  double delta_t = measurement_pack.timestamp_ - time_us_;
  time_us_ = measurement_pack.timestamp_;

  if(dt > 0.0001){
      Prediction(delta_t);
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
  Xsig.col(0)  = x;
  
  for (int i = 0; i < n_x; i++){
      Xsig.col(i+1) =     x + (sqrt(lambda + n_x) * A.col(i));
      Xsig.col(i+1+n_x) = x - (sqrt(lambda + n_x) * A.col(i));
  }
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
