/*
 * online_pelvis_xy_calculator.cpp
 *
 *  Created on: Feb 23, 2018
 *      Author: jaysong
 */

#include "heroehs_online_walking_pattern_generator/online_pelvis_xy_calculator.h"

using namespace heroehs;


OnlinePelvisXYCalculator::OnlinePelvisXYCalculator()
{
  lipm_height_m_ = 0.7;

  control_time_sec_ = 0.008;
  preview_time_sec_ = 1.6;

  preview_size_ = round(preview_time_sec_/control_time_sec_);

  k_s_ = 0;
  sum_of_zmp_x_ = 0;
  sum_of_zmp_y_ = 0;
  sum_of_cx_ = 0;
  sum_of_cy_ = 0;
}

OnlinePelvisXYCalculator::~OnlinePelvisXYCalculator()
{  }

void OnlinePelvisXYCalculator::initialize(double lipm_height_m, double preview_time_sec, double control_time_sec)
{
  lipm_height_m_ = lipm_height_m;
  preview_time_sec_ = preview_time_sec;
  control_time_sec_ = control_time_sec;

  double t = control_time_sec_;
  double g = 9.8;
  A_.resize(3,3); b_.resize(3,1); c_.resize(1,3);
  A_ << 1,  t, t*t/2.0,
      0,  1,   t,
      0,  0,   1;
  b_(0,0) = t*t*t/6.0;
  b_(1,0) =   t*t/2.0;
  b_(2,0) =     t;

  c_(0,0) = 1; c_(0,1) = 0; c_(0,2) = -lipm_height_m_/g;

  preview_size_ = round(preview_time_sec_/control_time_sec_);

  k_s_ = 0;
  sum_of_zmp_x_ = 0;
  sum_of_zmp_y_ = 0;
  sum_of_cx_ = 0;
  sum_of_cy_ = 0;


  //Preview Gain Calculation
  Eigen::MatrixXd tempA = Eigen::MatrixXd::Zero(4,4);
  Eigen::MatrixXd tempb = Eigen::MatrixXd::Zero(4,1);
  Eigen::MatrixXd tempc = Eigen::MatrixXd::Zero(1,4);

  tempA.coeffRef(0,0) = 1;
  tempA.block<1,3>(0,1) = c_*A_;
  tempA.block<3,3>(1,1) = A_;

  tempb.coeffRef(0,0) = (c_*b_).coeff(0,0);
  tempb.block<3,1>(1,0) = b_;

  tempc.coeffRef(0,0) = 1;

  double Q_e = 1, R = 1e-6;//1.0e-6;
  double Q_x = 0;
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(4,4);

  Q.coeffRef(0,0) = Q_e;
  Q.coeffRef(1,1) = Q_e;
  Q.coeffRef(2,2) = Q_e;
  Q.coeffRef(3,3) = Q_x;

  double matrix_A[] = {tempA.coeff(0,0), tempA.coeff(1,0), tempA.coeff(2,0), tempA.coeff(3,0),
      tempA.coeff(0,1), tempA.coeff(1,1), tempA.coeff(2,1), tempA.coeff(3,1),
      tempA.coeff(0,2), tempA.coeff(1,2), tempA.coeff(2,2), tempA.coeff(3,2),
      tempA.coeff(0,3), tempA.coeff(1,3), tempA.coeff(2,3), tempA.coeff(3,3)};
  int row_A = 4, col_A = 4;

  double matrix_B[] = {tempb.coeff(0,0), tempb.coeff(1,0), tempb.coeff(2,0), tempb.coeff(3,0)};
  int row_B = 4, col_B = 1;

  double matrix_Q[] = {Q.coeff(0,0), Q.coeff(1,0), Q.coeff(2,0), Q.coeff(3,0),
      Q.coeff(0,1), Q.coeff(1,1), Q.coeff(2,1), Q.coeff(3,1),
      Q.coeff(0,2), Q.coeff(1,2), Q.coeff(2,2), Q.coeff(3,2),
      Q.coeff(0,3), Q.coeff(1,3), Q.coeff(2,3), Q.coeff(3,3)};
  int row_Q = 4, col_Q = 4;

  double matrix_R[] = {R};
  int row_R = 1, col_R = 1;

  double *matrix_K = (double*)malloc(100*sizeof(double));
  int row_K, col_K;

  double *matrix_P = (double*)malloc(100*sizeof(double));
  int row_P, col_P;

  double *matrix_E_real = (double*)malloc(100*sizeof(double));
  double *matrix_E_imag = (double*)malloc(100*sizeof(double));
  int row_E, col_E;

  robotis_framework::ScilabOptimization::initialize();
  robotis_framework::ScilabOptimization::solveRiccatiEquation(matrix_K, &row_K, &col_K,
      matrix_P, &row_P, &col_P,
      matrix_E_real, matrix_E_imag, &row_E, &col_E,
      matrix_A, row_A, col_A,
      matrix_B, row_B, col_B,
      matrix_Q, row_Q, col_Q,
      matrix_R, row_R, col_R);

  robotis_framework::ScilabOptimization::terminate();

  Eigen::MatrixXd K_, P_;

  K_.resize(row_K,col_K);
  P_.resize(row_P,col_P);

  for(int j=0 ; j<row_K ; j++)
  {
    for(int i=0 ; i<col_K ; i++)
      K_.coeffRef(j,i) = matrix_K[i*row_K+j];
  }

  for(int j=0; j<row_P; j++)
  {
    for(int i=0 ; i<col_P; i++)
      P_.coeffRef(j,i) = matrix_P[i*row_P+j];
  }

  free(matrix_K);
  free(matrix_P);
  free(matrix_E_real);
  free(matrix_E_imag);

  k_s_ = K_.coeff(0,0);
  k_x_.resize(1,3);
  k_x_ << K_.coeff(0,1), K_.coeff(0,2), K_.coeff(0,3);

  f_.resize(1, preview_size_);

  Eigen::MatrixXd mat_R = Eigen::MatrixXd::Zero(1,1);
  mat_R.coeffRef(0,0) = R;

  Eigen::MatrixXd tempCoeff1 = mat_R + ((tempb.transpose() * P_) * tempb);
  Eigen::MatrixXd tempCoeff1_inv = tempCoeff1.inverse();
  Eigen::MatrixXd tempCoeff2 = tempb.transpose();
  Eigen::MatrixXd tempCoeff3 = Eigen::MatrixXd::Identity(4,4);
  Eigen::MatrixXd tempCoeff4 = P_*tempc.transpose();

  f_.block<1,1>(0,0) = ((tempCoeff1_inv*tempCoeff2)* tempCoeff3) * tempCoeff4;

  for(int i = 1; i < preview_size_; i++)
  {
    tempCoeff3 = tempCoeff3*((tempA - tempb*K_).transpose());
    f_.block<1,1>(0,i) = ((tempCoeff1_inv*tempCoeff2)* tempCoeff3) * tempCoeff4;
  }

  u_x.resize(1,1);
  u_y.resize(1,1);
  u_x.fill(0);
  u_y.fill(0);

  x_lipm_.fill(0.0);       y_lipm_.fill(0.0);
}

void OnlinePelvisXYCalculator::reInitialize(double lipm_height_m, double preview_time_sec, double control_time_sec)
{
  initialize(lipm_height_m, preview_time_sec, control_time_sec);
}

void OnlinePelvisXYCalculator::reInitialize()
{
  sum_of_zmp_x_ = 0;
  sum_of_zmp_y_ = 0;
  sum_of_cx_ = 0;
  sum_of_cy_ = 0;
  x_lipm_.fill(0.0);       y_lipm_.fill(0.0);
}


void OnlinePelvisXYCalculator::calcNextPelvisXY(const Eigen::VectorXd& reference_zmp_x,  const Eigen::VectorXd& reference_zmp_y)
{
  //  u_x(0,0) = -k_s_*(sum_of_cx_ - sum_of_zmp_x_) - (k_x_(0,0)*x_lipm_(0,0) + k_x_(0,1)*x_lipm_(1,0) + k_x_(0,2)*x_lipm_(2,0)) + f_*reference_zmp_x;
  //  u_y(0,0) = -k_s_*(sum_of_cy_ - sum_of_zmp_y_) - (k_x_(0,0)*y_lipm_(0,0) + k_x_(0,1)*y_lipm_(1,0) + k_x_(0,2)*y_lipm_(2,0)) + f_*reference_zmp_y;

  u_x(0,0) = -k_s_*(sum_of_cx_ - sum_of_zmp_x_) - (k_x_(0,0)*x_lipm_(0,0) + k_x_(0,1)*x_lipm_(1,0) + k_x_(0,2)*x_lipm_(2,0)) + (f_*reference_zmp_x).coeff(0,0);
  u_y(0,0) = -k_s_*(sum_of_cy_ - sum_of_zmp_y_) - (k_x_(0,0)*y_lipm_(0,0) + k_x_(0,1)*y_lipm_(1,0) + k_x_(0,2)*y_lipm_(2,0)) + (f_*reference_zmp_y).coeff(0,0);

  x_lipm_ = A_*x_lipm_ + b_*u_x;
  y_lipm_ = A_*y_lipm_ + b_*u_y;

  sum_of_cx_ += c_(0,0)*x_lipm_(0,0) +  c_(0,1)*x_lipm_(1,0) +  c_(0,2)*x_lipm_(2,0);
  sum_of_cy_ += c_(0,0)*y_lipm_(0,0) +  c_(0,1)*y_lipm_(1,0) +  c_(0,2)*y_lipm_(2,0);

  sum_of_zmp_x_ += reference_zmp_x.coeff(0);
  sum_of_zmp_y_ += reference_zmp_y.coeff(0);
}



