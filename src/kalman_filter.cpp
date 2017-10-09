#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;


KalmanFilter::KalmanFilter() = default;

KalmanFilter::~KalmanFilter() = default;

MatrixXd KalmanFilter::I_ = MatrixXd::Identity(4, 4);
Tools KalmanFilter::tools_ = Tools();

void KalmanFilter::Init(VectorXd &x_in,       MatrixXd &P_in,       MatrixXd &F_in, MatrixXd &H_in,
                        MatrixXd &R_lidar_in, MatrixXd &R_radar_in, MatrixXd &Q_in) {
  x_       = x_in;
  P_       = P_in;
  F_       = F_in;
  H_       = H_in;
  R_lidar_ = R_lidar_in;
  R_radar_ = R_radar_in;
  Q_       = Q_in;
}

void KalmanFilter::Predict() {
  x_ = F_ * x_; // u is zero vector; omitted for optimization purposes
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  UpdateCommon(z, H_, R_lidar_, false);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  MatrixXd Hj = tools_.CalculateJacobian(x_);

  UpdateCommon(z, Hj, R_radar_, true);
}

void KalmanFilter::UpdateCommon(const VectorXd &z, const MatrixXd &H, const MatrixXd &R, bool is_ekf) {
  VectorXd y;

  if (is_ekf) {
    // using equations for extended kalman filter

    // convert radar measurements from cartesian coordinates (x, y, vx, vy) to polar (rho, phi, rho_dot).
    VectorXd x_polar = tools_.ConvertFromCartesianToPolarCoords(x_);
    y = z - x_polar;

    // normalize the angle between -pi to pi
    while(y(1) > M_PI){
      y(1) -= 2 * M_PI;
    }

    while(y(1) < -M_PI){
      y(1) += 2 * M_PI;
    }
  } else {
    // using equations for kalman filter
    y = z - H * x_;
  }

  MatrixXd Ht = H.transpose();
  MatrixXd S = H * P_ * Ht + R;
  MatrixXd Si = S.inverse();
  MatrixXd K =  P_ * Ht * Si;

  // new state
  x_ = x_ + (K * y);
  P_ = (I_ - K * H) * P_;
}
