/*
 * heroehs_pd_balance_controller.h
 *
 *  Created on: 2018. 2. 27.
 *      Author: Crowban
 */

#ifndef HEROEHS_PD_BALANCE_CONTROLLER_HEROEHS_PD_BALANCE_CONTROLLER_H_
#define HEROEHS_PD_BALANCE_CONTROLLER_HEROEHS_PD_BALANCE_CONTROLLER_H_

#include "robotis_math/robotis_math.h"
#include "heroehs_math/heroehs_math.h"

namespace heroehs
{

class BalanceControlError
{
public:
  static const int NoError = 0;
  static const int BalanceLimit = 2;
};

class BalanceControlUsingPDController
{
public:
  BalanceControlUsingPDController();
  ~BalanceControlUsingPDController();

  void initialize(const double control_cycle_sec);

  void setGyroBalanceEnable(bool enable);
  void setOrientationBalanceEnable(bool enable);
  void setForceTorqueBalanceEnable(bool enable);

  void process(int *balance_error, Eigen::MatrixXd *robot_to_cob_modified, Eigen::MatrixXd *robot_to_right_foot_modified, Eigen::MatrixXd *robot_to_left_foot_modified);

  void setDesiredPose(const Eigen::MatrixXd &robot_to_cob, const Eigen::MatrixXd &robot_to_right_foot, const Eigen::MatrixXd &robot_to_left_foot);

  // all arguments are with respect to robot coordinate.
  void setDesiredCOBGyro(double gyro_roll, double gyro_pitch);
  void setDesiredCOBOrientation(double cob_orientation_roll, double cob_orientation_pitch);
  void setDesiredFootForceTorque(double r_force_x_N,      double r_force_y_N,       double r_force_z_N,
                                 double r_torque_roll_Nm, double r_torque_pitch_Nm, double r_torque_yaw_Nm,
                                 double l_force_x_N,      double l_force_y_N,       double l_force_z_N,
                                 double l_torque_roll_Nm, double l_torque_pitch_Nm, double l_torque_yaw_Nm);

  // with respect to robot coordinate.
  void setCurrentGyroSensorOutput(double gyro_roll, double gyro_pitch);
  void setCurrentOrientationSensorOutput(double cob_orientation_roll, double cob_orientation_pitch);

  // with respect to robot coordinate.
  void setCurrentFootForceTorqueSensorOutput(double r_force_x_N,      double r_force_y_N,       double r_force_z_N,
                                             double r_torque_roll_Nm, double r_torque_pitch_Nm, double r_torque_yaw_Nm,
                                             double l_force_x_N,      double l_force_y_N,       double l_force_z_N,
                                             double l_torque_roll_Nm, double l_torque_pitch_Nm, double l_torque_yaw_Nm);


  // set maximum adjustment
  void setMaximumAdjustment(double cob_x_max_adjustment_m,  double cob_y_max_adjustment_m,  double cob_z_max_adjustment_m,
                            double cob_roll_max_adjustment_rad, double cob_pitch_max_adjustment_rad, double cob_yaw_max_adjustment_rad,
                            double foot_x_max_adjustment_m, double foot_y_max_adjustment_m, double foot_z_max_adjustment_m,
                            double foot_roll_max_adjustment_rad, double foot_pitch_max_adjustment_rad, double foot_yaw_max_adjustment_rad);

  //Manual Adjustment
  void setCOBManualAdjustment(double cob_x_adjustment_m, double cob_y_adjustment_m, double cob_z_adjustment_m);
  double getCOBManualAdjustmentX();
  double getCOBManualAdjustmentY();
  double getCOBManualAdjustmentZ();

  // damping controllers
  PDController foot_roll_gyro_ctrl_;
  PDController foot_pitch_gyro_ctrl_;
  PDController foot_roll_angle_ctrl_;
  PDController foot_pitch_angle_ctrl_;

  PDController right_foot_force_z_ctrl_;
  PDController left_foot_force_z_ctrl_;

  PDController right_foot_force_x_ctrl_;
  PDController right_foot_force_y_ctrl_;
  PDController right_foot_torque_roll_ctrl_;
  PDController right_foot_torque_pitch_ctrl_;

  PDController left_foot_force_x_ctrl_;
  PDController left_foot_force_y_ctrl_;
  PDController left_foot_torque_roll_ctrl_;
  PDController left_foot_torque_pitch_ctrl_;


  LowPassFilter roll_gyro_lpf_;
  LowPassFilter pitch_gyro_lpf_;

  LowPassFilter roll_angle_lpf_;
  LowPassFilter pitch_angle_lpf_;

  LowPassFilter right_foot_force_x_lpf_;
  LowPassFilter right_foot_force_y_lpf_;
  LowPassFilter right_foot_force_z_lpf_;
  LowPassFilter right_foot_torque_roll_lpf_;
  LowPassFilter right_foot_torque_pitch_lpf_;

  LowPassFilter left_foot_force_x_lpf_;
  LowPassFilter left_foot_force_y_lpf_;
  LowPassFilter left_foot_force_z_lpf_;
  LowPassFilter left_foot_torque_roll_lpf_;
  LowPassFilter left_foot_torque_pitch_lpf_;

private:
  int balance_control_error_;
  double control_cycle_sec_;

  // balance enable
  double gyro_enable_;
  double orientation_enable_;
  double ft_enable_;

  // desired pose
  Eigen::MatrixXd desired_robot_to_cob_;
  Eigen::MatrixXd desired_robot_to_right_foot_;
  Eigen::MatrixXd desired_robot_to_left_foot_;

  // sensed values
  double current_gyro_roll_rad_per_sec_, current_gyro_pitch_rad_per_sec_;

  double current_orientation_roll_rad_, current_orientation_pitch_rad_;

  double current_right_fx_N_,  current_right_fy_N_,  current_right_fz_N_;
  double current_right_tx_Nm_, current_right_ty_Nm_, current_right_tz_Nm_;
  double current_left_fx_N_,   current_left_fy_N_,   current_left_fz_N_;
  double current_left_tx_Nm_,  current_left_ty_Nm_,  current_left_tz_Nm_;

  // manual cob adjustment
  double cob_x_manual_adjustment_m_;
  double cob_y_manual_adjustment_m_;
  double cob_z_manual_adjustment_m_;

  // result of balance control
  double foot_roll_adjustment_by_gyro_roll_;
  double foot_pitch_adjustment_by_gyro_pitch_;

  double foot_roll_adjustment_by_orientation_roll_;
  double foot_pitch_adjustment_by_orientation_pitch_;

  double r_foot_z_adjustment_by_force_z_;
  double l_foot_z_adjustment_by_force_z_;

  double r_foot_x_adjustment_by_force_x_;
  double r_foot_y_adjustment_by_force_y_;
  double r_foot_roll_adjustment_by_torque_roll_;
  double r_foot_pitch_adjustment_by_torque_pitch_;

  double l_foot_x_adjustment_by_force_x_;
  double l_foot_y_adjustment_by_force_y_;
  double l_foot_roll_adjustment_by_torque_roll_;
  double l_foot_pitch_adjustment_by_torque_pitch_;

  // sum of results of balance control
  Eigen::VectorXd pose_cob_adjustment_;
  Eigen::VectorXd pose_right_foot_adjustment_;
  Eigen::VectorXd pose_left_foot_adjustment_;

  Eigen::MatrixXd mat_robot_to_cob_modified_;
  Eigen::MatrixXd mat_robot_to_right_foot_modified_;
  Eigen::MatrixXd mat_robot_to_left_foot_modified_;

  // maximum adjustment
  double cob_x_adjustment_abs_max_m_;
  double cob_y_adjustment_abs_max_m_;
  double cob_z_adjustment_abs_max_m_;
  double cob_roll_adjustment_abs_max_rad_;
  double cob_pitch_adjustment_abs_max_rad_;
  double cob_yaw_adjustment_abs_max_rad_;

  double foot_x_adjustment_abs_max_m_;
  double foot_y_adjustment_abs_max_m_;
  double foot_z_adjustment_abs_max_m_;
  double foot_roll_adjustment_abs_max_rad_;
  double foot_pitch_adjustment_abs_max_rad_;
  double foot_yaw_adjustment_abs_max_rad_;
};

}



#endif /* HEROEHS_PD_BALANCE_CONTROLLER_HEROEHS_PD_BALANCE_CONTROLLER_H_ */
