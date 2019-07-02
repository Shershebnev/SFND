#include "ukf.h"
#include "Eigen/Dense"


/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  n_x_ = 5;
  n_aug_ = 7;

  // initial state vector
  x_ = Eigen::VectorXd::Zero(n_x_);

  // initial covariance matrix
  P_ = Eigen::MatrixXd::Zero(n_x_, n_x_);

  Xsig_pred_ = Eigen::MatrixXd::Zero(n_x_, 2 * n_aug_ + 1);
  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 3; //30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.7; //30;
  
  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

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
   * End DO NOT MODIFY section for measurement noise values 
   */
  
  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */
  lambda_ = 3 - n_aug_;
  weights_ = Eigen::VectorXd::Zero(2 * n_aug_ + 1);
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  for (int i = 1; i < 2 * n_aug_ + 1; i++) {
	  weights_(i) = 0.5 / (n_aug_ + lambda_);
  }
  n_z_radar_ = 3;
  n_z_lidar_ = 2;
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */
	if (!is_initialized_) {
		P_ << 1, 0, 0, 0, 0,
			  0, 1, 0, 0, 0,
			  0, 0, 1, 0, 0,
			  0, 0, 0, 1, 0,
			  0, 0, 0, 0, 1;
		if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
			float rho = meas_package.raw_measurements_(0);
			float phi = meas_package.raw_measurements_(1);
			float rhod = meas_package.raw_measurements_(2);

			float px = rho * std::cos(phi);
			float py = rho * std::sin(phi);
			float vx = rhod * std::cos(phi);
			float vy = rhod * std::sin(phi);
			x_ << px, py, std::sqrt(std::pow(vx, 2) + std::pow(vy, 2)), 0, 0;
		} else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
			x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0, 0;
		}
		is_initialized_ = true;
		time_us_ = meas_package.timestamp_;
		return;
	}
	Prediction((meas_package.timestamp_ - time_us_) / 1000000.0);
	if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
		Eigen::VectorXd z_pred = Eigen::VectorXd::Zero(n_z_radar_);
		Eigen::MatrixXd S = Eigen::MatrixXd::Zero(n_z_radar_, n_z_radar_);
		Eigen::MatrixXd Tc = Eigen::MatrixXd::Zero(n_x_, n_z_radar_);
		PredictRadarMeasurement(&z_pred, &S, &Tc);
		UpdateRadar(meas_package, z_pred, S, Tc);
		time_us_ = meas_package.timestamp_;
	} else if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
		Eigen::VectorXd z_pred = Eigen::VectorXd::Zero(n_z_lidar_);
		Eigen::MatrixXd S = Eigen::MatrixXd::Zero(n_z_lidar_, n_z_lidar_);
		Eigen::MatrixXd Tc = Eigen::MatrixXd::Zero(n_x_, n_z_lidar_);
		PredictLidarMeasurement(&z_pred, &S, &Tc);
		UpdateLidar(meas_package, z_pred, S, Tc);
		time_us_ = meas_package.timestamp_;
	}
}

void UKF::AugmentedSigmaPoints(Eigen::MatrixXd* Xsig_out) {
	Eigen::VectorXd x_aug = Eigen::VectorXd(n_aug_);
	Eigen::MatrixXd P_aug = Eigen::MatrixXd(n_aug_, n_aug_);
	Eigen::MatrixXd Xsig_aug = Eigen::MatrixXd(n_aug_, 2 * n_aug_ + 1);

	x_aug.head(n_x_) = x_;
	x_aug(n_x_) = 0;
	x_aug(n_x_ + 1) = 0;

	P_aug.fill(0.0);
	P_aug.topLeftCorner(n_x_, n_x_) = P_;
	P_aug(n_x_, n_x_) = std::pow(std_a_, 2);
	P_aug(n_x_ + 1, n_x_ + 1) = std::pow(std_yawdd_, 2);

	Eigen::MatrixXd L = P_aug.llt().matrixL();

	Xsig_aug.col(0) = x_aug;
	for (int i = 0; i < n_aug_; i++) {
		Xsig_aug.col(i + 1) = x_aug + std::sqrt(lambda_ + n_aug_) * L.col(i);
		Xsig_aug.col(i + 1 + n_aug_) = x_aug - std::sqrt(lambda_ + n_aug_) * L.col(i);
	}
	*Xsig_out = Xsig_aug;
}

void UKF::SigmaPointPrediction(const Eigen::MatrixXd &Xsig_aug, const double delta, Eigen::MatrixXd* Xsig_out) {
	Eigen::MatrixXd Xsig_pred = Eigen::MatrixXd::Zero(n_x_, 2 * n_aug_ + 1);
	double px, py, v, yaw, yawd, nu_a, nu_yawdd, px_p, py_p, v_p, yaw_p, yawd_p;
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {
		px = Xsig_aug(0, i);
		py = Xsig_aug(1, i);
		v = Xsig_aug(2, i);
		yaw = Xsig_aug(3, i);
		yawd = Xsig_aug(4, i);
		nu_a = Xsig_aug(5, i);
		nu_yawdd = Xsig_aug(6, i);

		if (std::fabs(yawd) > 0.001) {
			px_p = px + v / yawd * (std::sin(yaw + yawd * delta) - std::sin(yaw));
			py_p = py + v / yawd * (std::cos(yaw) - std::cos(yaw + yawd * delta));
		} else {
			px_p = px + v * delta * std::cos(yaw);
			py_p = py + v * delta * std::sin(yaw);
		}
		v_p = v;
		yaw_p = yaw + yawd * delta;
		yawd_p = yawd;

		// add noise
		px_p += 0.5 * nu_a * std::pow(delta, 2) * std::cos(yaw);
		py_p += 0.5 * nu_a * std::pow(delta, 2) * std::sin(yaw);
		v_p += nu_a * delta;
		yaw_p += 0.5 * nu_yawdd * std::pow(delta, 2);
		yawd_p += nu_yawdd * delta;

		Xsig_pred(0, i) = px_p;
		Xsig_pred(1, i) = py_p;
		Xsig_pred(2, i) = v_p;
		Xsig_pred(3, i) = yaw_p;
		Xsig_pred(4, i) = yawd_p;
	}
	*Xsig_out = Xsig_pred;
}

void UKF::PredictMeanAndCovariance(const Eigen::MatrixXd &Xsig_pred, Eigen::VectorXd* x_out, Eigen::MatrixXd* P_out) {
	Eigen::VectorXd x = Eigen::VectorXd(n_x_);
	Eigen::MatrixXd P = Eigen::MatrixXd(n_x_, n_x_);

	x.fill(0.0);
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {
		x += weights_(i) * Xsig_pred.col(i);
	}

	P.fill(0.0);
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {
		Eigen::VectorXd x_diff = Xsig_pred.col(i) - x;
		while (x_diff(3) > M_PI) x_diff(3) -= 2 * M_PI;
		while (x_diff(3) < -M_PI) x_diff(3) += 2 * M_PI;

		P = P + weights_(i) * x_diff * x_diff.transpose();
	}
	*x_out = x;
	*P_out = P;
}

void UKF::PredictRadarMeasurement(Eigen::VectorXd* z_out, Eigen::MatrixXd* S_out, Eigen::MatrixXd *Tc_out) {
	Eigen::MatrixXd Zsig = Eigen::MatrixXd(n_z_radar_, 2 * n_aug_ + 1);
	Eigen::VectorXd z_pred = Eigen::VectorXd(n_z_radar_);
	Eigen::MatrixXd S = Eigen::MatrixXd(n_z_radar_, n_z_radar_);
	Eigen::MatrixXd Tc = Eigen::MatrixXd(n_x_, n_z_radar_);
	double px, py, v, yaw, yawd;

	for (int i = 0; i < 2 * n_aug_ + 1; i++) {
		px = Xsig_pred_(0, i);
		py = Xsig_pred_(1, i);
		v = Xsig_pred_(2, i);
		yaw = Xsig_pred_(3, i);
		yawd = Xsig_pred_(4, i);

		Zsig(0, i) = std::sqrt(std::pow(px, 2) + std::pow(py, 2));
		Zsig(1, i) = std::atan2(py, px);
		Zsig(2, i) = (px * std::cos(yaw) * v + py * std::sin(yaw) * v) / std::sqrt(std::pow(px, 2) + std::pow(py, 2));
	}
	z_pred.fill(0.0);
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {
		z_pred += weights_(i) * Zsig.col(i);
	}
	S.fill(0.0);
	Tc.fill(0.0);
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {
		Eigen::VectorXd z_diff = Zsig.col(i) - z_pred;
		while (z_diff(1) > M_PI) z_diff(1) -= 2 * M_PI;
		while (z_diff(1) < -M_PI) z_diff(1) += 2 * M_PI;
		S += weights_(i) * z_diff * z_diff.transpose();
		Eigen::VectorXd x_diff = Xsig_pred_.col(i) - x_;
		Tc += weights_(i) * x_diff * z_diff.transpose();
	}

	Eigen::MatrixXd R = Eigen::MatrixXd(n_z_radar_, n_z_radar_);
	R << std::pow(std_radr_, 2), 0, 0,
		 0, std::pow(std_radphi_, 2), 0,
		 0, 0, std::pow(std_radrd_, 2);
	S += R;

	*z_out = z_pred;
	*S_out = S;
	*Tc_out = Tc;
}

void UKF::PredictLidarMeasurement(Eigen::VectorXd* z_out, Eigen::MatrixXd* S_out, Eigen::MatrixXd* Tc_out) {
	Eigen::MatrixXd Zsig = Eigen::MatrixXd(n_z_lidar_, 2 * n_aug_ + 1);
	Eigen::VectorXd z_pred = Eigen::VectorXd(n_z_lidar_);
	Eigen::MatrixXd S = Eigen::MatrixXd(n_z_lidar_, n_z_lidar_);
	Eigen::MatrixXd Tc = Eigen::MatrixXd(n_x_, n_z_lidar_);

	for (int i = 0; i < 2 * n_aug_ + 1; i++) {
		Zsig(0, i) = Xsig_pred_(0, i);
		Zsig(1, i) = Xsig_pred_(1, i);
	}

	z_pred.fill(0.0);
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {
		z_pred += weights_(i) * Zsig.col(i);
	}
	S.fill(0.0);
	Tc.fill(0.0);
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {
		Eigen::VectorXd z_diff = Zsig.col(i) - z_pred;
		S += weights_(i) * z_diff * z_diff.transpose();
		Eigen::VectorXd x_diff = Xsig_pred_.col(i) - x_;
		Tc += weights_(i) * x_diff * z_diff.transpose();
	}

	Eigen::MatrixXd R = Eigen::MatrixXd(n_z_lidar_, n_z_lidar_);
	R << std::pow(std_laspx_, 2), 0,
	     0, std::pow(std_laspy_, 2);
	S += R;

	*z_out = z_pred;
	*S_out = S;
	*Tc_out = Tc;
}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */

	Eigen::MatrixXd Xsig_out = Eigen::MatrixXd(2 * n_aug_ + 1, n_aug_);
	Eigen::MatrixXd Xsig_pred = Eigen::MatrixXd(n_x_, 2 * n_aug_ + 1);
	// generate sigma points
	AugmentedSigmaPoints(&Xsig_out);
	// predict sigma points
	SigmaPointPrediction(Xsig_out, delta_t, &Xsig_pred);
	// predict mean and covariance
	Eigen::VectorXd x_pred = Eigen::VectorXd(n_x_);
	Eigen::MatrixXd P_pred = Eigen::MatrixXd(n_x_, n_x_);
	PredictMeanAndCovariance(Xsig_pred, &x_pred, &P_pred);
	x_ = x_pred;
	P_ = P_pred;
	Xsig_pred_ = Xsig_pred;
}

void UKF::UpdateLidar(MeasurementPackage meas_package, const Eigen::VectorXd &z_pred, const Eigen::MatrixXd &S, const Eigen::MatrixXd &Tc) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */
	Eigen::VectorXd z = Eigen::VectorXd::Zero(n_z_lidar_);
	z << meas_package.raw_measurements_(0), meas_package.raw_measurements_(1);
	Eigen::MatrixXd K = Tc * S.inverse();
	Eigen::VectorXd z_diff = z - z_pred;
	x_ += K * (z - z_pred);
	P_ -= K * S * K.transpose();
}

void UKF::UpdateRadar(MeasurementPackage meas_package, const Eigen::VectorXd &z_pred, const Eigen::MatrixXd &S, const Eigen::MatrixXd &Tc) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */
	Eigen::VectorXd z = Eigen::VectorXd::Zero(n_z_radar_);
	z << meas_package.raw_measurements_(0), meas_package.raw_measurements_(1), meas_package.raw_measurements_(2);
	Eigen::MatrixXd K = Tc * S.inverse();
	Eigen::VectorXd z_diff = z - z_pred;
	while (z_diff(1) > M_PI) z_diff(1) -= 2 * M_PI;
	while (z_diff(1) < -M_PI) z_diff(1) += 2 * M_PI;
	x_ += K * (z - z_pred);
	P_ -= K * S * K.transpose();
}
