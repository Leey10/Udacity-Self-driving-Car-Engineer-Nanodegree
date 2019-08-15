#include "kalman_filter.h"
#define _USE_MATH_DEFINES
 
#include <cmath>

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
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ *Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  VectorXd y = z - H_ * x_;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ *Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;
  x_ = x_ + K * y;
  
  int x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size,x_size);
  P_ = (I - K * H_) * P_;
  
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  double px = x_[0];
  double py = x_[1];
  double vx = x_[2];
  double vy = x_[3];
  
  double rho = sqrt( px*px + py*py);
  
  if (rho < 0.001){
   rho = 0.001; 
  }
  double phi = atan2(py, px);
  double drho = (px*vx + py*vy)/rho;
  
  VectorXd xp = VectorXd(3);
  xp << rho, phi, drho;
  VectorXd y = z - xp;
  
  //Normalize the angle between -pi to pi
  while (y(1) < -M_PI)
    y(1) += 2 * M_PI;
  while (y(1) > M_PI)
    y(1) -= 2 * M_PI;
  
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ *Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;
  x_ = x_ + K * y;
  
  int x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size,x_size);
  P_ = (I - K * H_) * P_;
}
