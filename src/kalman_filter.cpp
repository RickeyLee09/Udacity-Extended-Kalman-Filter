#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
    x_ = F_ * x_;
  	P_ = F_ * P_ * F_.transpose() + Q_;
  
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  VectorXd y = z - H_ * x_;
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();
  x_ = x_ + K * y;
  MatrixXd I = MatrixXd::Identity(H_.cols(), H_.cols());
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  float rho_pred = sqrt(px * px + py * py);
  float phi_pred = 0.0;
  
  if (fabs(px) > 0.001){
  	phi_pred = atan2(py, px);
  }else{
    return;
  }
  
  float rhodot_pred = 0.0;
  if (fabs(rho_pred) > 0.001){
    rhodot_pred = (px * vx + py * vy) / rho_pred;
  }else{
    return;
  }
  
  VectorXd z_pred(3);
  z_pred << rho_pred, phi_pred, rhodot_pred;
  
  VectorXd y = z - z_pred;
  if (y[1] < -M_PI){
    y[1] += 2 * M_PI;
  }else if (y[1] > M_PI){
    y[1] -= 2 * M_PI;
  }
  
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd K = P_ * Ht * S.inverse();
  x_ = x_ + K * y;
  MatrixXd I = MatrixXd::Identity(H_.cols(), H_.cols());
  P_ = (I - K * H_) * P_;
}
