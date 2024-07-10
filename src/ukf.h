#ifndef UKF_H
#define UKF_H

#include "Eigen/Dense"
#include "measurement_package.h"
#include "ukf_sensor_correction.h"
#include <iostream>

using namespace std;

class UKF {
 public:
  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(MeasurementPackage meas_package);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double delta_t);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateLidar(MeasurementPackage meas_package);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(MeasurementPackage meas_package);

  void SetCorrelationMatrix(const Eigen::MatrixXd & Xsig_pred, ukf_sensor_correction & sensorCorrection, Eigen::VectorXd & x);

  void PredictMeasurement(const Eigen::MatrixXd & Xsig_pred, ukf_sensor_correction & sensorCorrection);

  void CorrectState(const MatrixXd & Xsig_pred, ukf_sensor_correction & sensorCorrection);

  void AugmentedSigmaPoints();

  void SigmaPointPrediction();

  void PredictMeanAndCovariance();

  UKF & operator=(const UKF & other);


  // initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  // if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  // if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  // state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  Eigen::VectorXd x_aug;

  Eigen::MatrixXd P_aug;

  Eigen::MatrixXd Q_;

  Eigen::MatrixXd R_radar;

  Eigen::MatrixXd R_lidar;

  // predicted sigma points matrix

  Eigen::MatrixXd Xsig_aug_;
  
  Eigen::MatrixXd Xsig_pred_;

  ukf_sensor_correction lidarCorrection;

  ukf_sensor_correction radarCorrection;

  int sigmaPointCount;

  // time when the state is true, in us
  long long time_us_;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  // Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  // Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  // Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  // Radar measurement noise standard deviation radius in m
  double std_radr_;

  // Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  // Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  // Weights of sigma points
  Eigen::VectorXd weights_;

  // State dimension
  int n_x_;

  // Augmented state dimension
  int n_aug_;

  int n_z_radar;

  int n_z_lidar;

  // Sigma point spreading parameter
  double lambda_;

  long previous_timestamp_;

  friend class ukf_sensor_correction;

  float dt;
};

#endif  // UKF_H