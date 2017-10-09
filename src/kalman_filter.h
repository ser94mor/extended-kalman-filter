#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"
#include "tools.h"

class KalmanFilter {
public:

  // state vector
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // state transition matrix
  Eigen::MatrixXd F_;

  // process covariance matrix
  Eigen::MatrixXd Q_;

  // measurement matrix
  Eigen::MatrixXd H_;

  // measurement covariance matrix for lidar
  Eigen::MatrixXd R_lidar_;

  // measurement covariance matrix for radar
  Eigen::MatrixXd R_radar_;

  /**
   * Constructor
   */
  KalmanFilter();

  /**
   * Destructor
   */
  virtual ~KalmanFilter();

  /**
   * Init Initializes Kalman filter
   * @param x_in        Initial state
   * @param P_in        Initial state covariance
   * @param F_in        Transition matrix
   * @param H_in        Measurement matrix
   * @param R_lidar_in  Measurement covariance matrix for lidar
   * @param R_radar_in  Measurement covariance matrix for radar
   * @param Q_in        Process covariance matrix
   */
  void Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in,
      Eigen::MatrixXd &H_in, Eigen::MatrixXd &R_lidar_in, Eigen::MatrixXd &R_radar_in, Eigen::MatrixXd &Q_in);

  /**
   * Predicts the state and the state covariance using the process model.
   */
  void Predict();

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  void Update(const Eigen::VectorXd &z);

  /**
   * Updates the state by using Extended Kalman Filter equations.
   * @param z The measurement at k+1
   */
  void UpdateEKF(const Eigen::VectorXd &z);

private:

  /**
   * Method implementing a common part between Update() and UpdateEKF() methods.
   * @param z       the measurement at k+1
   * @param H       either measurement matrix (for lidar) or Jacobian (for radar)
   * @param R       measurement covariance matrix
   * @param is_ekf  boolean indication whether to use equations for EKF or not
   */
  void UpdateCommon(const VectorXd &z, const MatrixXd &H, const MatrixXd &R, bool is_ekf);

  // identity matrix
  static Eigen::MatrixXd I_;

  // tools class instance
  static Tools tools_;

};

#endif /* KALMAN_FILTER_H_ */
