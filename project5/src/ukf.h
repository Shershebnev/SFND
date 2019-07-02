#ifndef UKF_H
#define UKF_H

#include <iostream>
#include "Eigen/Dense"
#include "measurement_package.h"

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
   * Augmented sigma points generation
   * @param Xsig_out matrix of sigma points
   */
  void AugmentedSigmaPoints(Eigen::MatrixXd* Xsig_out);

  /**
   * Sigma point prediction
   * @param Xsig_aug matrix of augmented sigma points
   * @param delta time difference
   * @param Xsig_out matrix of predicted sigma points
   */
  void SigmaPointPrediction(const Eigen::MatrixXd &Xsig_aug, const double delta, Eigen::MatrixXd* Xsig_out);

  /**
   * Calculate mean and covariance of predicted sigma points
   * @param Xsig_pred matrix of predicted sigma points
   * @param x_out predicted state mean vector
   * @param P_out predicted state covariance matrix
   */
  void PredictMeanAndCovariance(const Eigen::MatrixXd &Xsig_pred, Eigen::VectorXd* x_out, Eigen::MatrixXd* P_out);

  /**
   * Predict radar measurements using predicted sigma points
   * @param z_out predicted measurement mean
   * @param S_out predicted measurement covariance
   * @param Tc_out matrix for cross correlation
   */
  void PredictRadarMeasurement(Eigen::VectorXd* z_out, Eigen::MatrixXd* S_out, Eigen::MatrixXd *Tc_out);

  /**
   * Predict lidar measurements using predicted sigma points
   * @param z_out predicted measurement mean
   * @param S_out predicted measurement covariance
   * @param Tc_out matrix for cross correlation
   */
  void PredictLidarMeasurement(Eigen::VectorXd* z_out, Eigen::MatrixXd* S_out, Eigen::MatrixXd *Tc_out);

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
  void UpdateLidar(MeasurementPackage meas_package, const Eigen::VectorXd &z_pred, const Eigen::MatrixXd &S, const Eigen::MatrixXd &Tc);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(MeasurementPackage meas_package, const Eigen::VectorXd &z_pred, const Eigen::MatrixXd &S, const Eigen::MatrixXd &Tc);


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

  // predicted sigma points matrix
  Eigen::MatrixXd Xsig_pred_;

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

  // Sigma point spreading parameter
  double lambda_;

  // radar measurement dimension
  int n_z_radar_;

  // lidar measurement dimension
  int n_z_lidar_;
};

#endif  // UKF_H
