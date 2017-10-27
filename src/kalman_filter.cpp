#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
}

void KalmanFilter::Predict() {
  // update state
  x_ = F_ * x_;

  // update state co-variance
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  // convert state x+1 into measurement space of z
  VectorXd z_pred(3);
  float range = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
  // range aka distance (rho)
  z_pred(0) = range;
  // bearing aka angle (phi)
	z_pred(1) = atan2(x_(1), x_(0));
  // radial velocity (rho dot)
  z_pred(2) = (x_(0)*x_(2) + x_(1)*x_(3)) / range;
	VectorXd y = z - z_pred;
	y(1) = NormalizeAngle(y(1));
  
  // H should be set to Hj in FusionEKF before this function call
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	// new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}
