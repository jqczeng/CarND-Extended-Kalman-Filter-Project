# Extended Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program

In this project a kalman filter wil be utilized to estimate the state of a moving object of interest with noisy lidar and radar measurements.

## FusionEKF.cpp

The `FusionEKF` class is an extended Kalman filter (EKF) class. The EKF takes in data from radar and laser measurements and fuses the data together to form a state prediction and update the states based on sensor measurement data.

The following steps are carried out to finish implementing `FusionEKF`.

### 1. Initialize Variables and Matrices

First the matrices within `FusionEKF.cpp` are initalized. This can be found within lines 35-52 within the `FusionEKF.cpp` file. The process covariance matrix `efk.P_` and the state transition matrix `efk.F_` are defined by 4x4 matrices. The laser measurement matrix `H_laser_` is defined by a 2x4 matrix.

```cpp
  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
   * Finish initializing the FusionEKF.
   *  Set the process and measurement noises
   */
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1000, 0,
             0, 0, 0, 1000;

  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
             0, 1, 0, 1,
             0, 0, 1, 0,
             0, 0, 0, 1;

  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;
```
### 2. Initialize with First Sensor Measurements



```cpp
if (!is_initialized_) {
  /**
   * Initialize the state ekf_.x_ with the first measurement.
   * Create the covariance matrix.
   * Convert radar from polar to cartesian coordinates.
   */

previous_timestamp_ = measurement_pack.timestamp_;

  // first measurement
  cout << "EKF: " << endl;
  ekf_.x_ = VectorXd(4);
  ekf_.x_ << 1, 1, 1, 1;

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    //  Convert radar from polar to cartesian coordinates and initialize state.
    // set the state with the initial location and zero velocity
    float r = measurement_pack.raw_measurements_[0];
    float theta = measurement_pack.raw_measurements_[1];

    float x = r*cos(theta);
    float y = r*sin(theta);

    ekf_.x_ << x,
               y,
               0,
               0;

  }
  else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
    // Initialize state.

  // set the state with the initial location and zero velocity
  ekf_.x_ << measurement_pack.raw_measurements_[0],
             measurement_pack.raw_measurements_[1],
             0,
             0;
  }

  // done initializing, no need to predict or update
  is_initialized_ = true;
  return;
}
```
---
