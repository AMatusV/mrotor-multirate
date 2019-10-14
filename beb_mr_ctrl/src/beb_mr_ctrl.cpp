#include <boost/make_shared.hpp>

#include <geometry_msgs/Twist.h>
#include <angles/angles.h>
#include "beb_mr_ctrl/beb_mr_ctrl.h"


namespace beb_mr_ctrl
{

BebMrCtrl::BebMrCtrl(ros::NodeHandle &nh)
  : nh_(nh),
    nh_priv_("~"),
    nh_pid_px_(nh_priv_, "pid_forward"),
    nh_pid_py_(nh_priv_, "pid_lateral"),
    nh_pid_yaw_(nh_priv_, "pid_yaw"),
    nh_pid_alt_(nh_priv_, "pid_alt"),
    sub_ctrl_enable_(nh_.subscribe("ctrl_enable", 1, &BebMrCtrl::CtrlEnableCallback, this)),
    sub_setpoint_pose_(nh_.subscribe("setpoint_pose", 1, &BebMrCtrl::SetpointPoseCallback, this)),
    sub_model_reset_(nh_.subscribe("model_reset", 1, &BebMrCtrl::ModelResetCallback, this)),
    pub_ctrl_cmd_vel_(nh_.advertise<geometry_msgs::Twist>("bebop/cmd_vel", 1)),
    pub_debug_(nh_.advertise<beb_mr_ctrl::Debug>("debug", 1)),
    sub_slam_pose_(nh_.subscribe("slam_pose", 1, &BebMrCtrl::SlamPoseCallback, this)),
    beb_param_recv_(false),
    slam_recv_time_(0),
    setpoint_recv_time_(0),
    pid_px_(new control_toolbox::Pid()),
    pid_py_(new control_toolbox::Pid()),
    pid_yaw_(new control_toolbox::Pid()),
    pid_alt_(new control_toolbox::Pid()),
    ctrl_enabled_(false),
    reset_counter_(0)
{
  util::GetParam(nh_priv_, "model_mr_factor", param_mr_factor_, 2);
  util::GetParam(nh_priv_, "model_delay_s", param_time_delay_, 0.1);
  util::GetParam(nh_priv_, "update_freq", param_update_freq_, 40.0);

  util::GetParam(nh_priv_, "max_linear_vel", param_max_linear_vel_, 2.0);
  util::GetParam(nh_priv_, "min_alt", param_min_alt_, 0.4);
  util::GetParam(nh_priv_, "max_alt", param_max_alt_, 2.5);

  util::GetParam(nh_priv_, "feedback_pred_factor", param_feedback_pred_factor_, 0.2);
  util::GetParam(nh_priv_, "delay_compensation_factor", param_delay_compensation_factor_, 0.7);

  util::GetParam(nh_priv_, "safety_send_zero", param_safety_send_zero_, true);
  util::GetParam(nh_priv_, "zero_xy_hover", param_xy_hover, true);
  util::GetParam(nh_priv_, "manual_override_sim", param_man_ovrd_sim_, false);
  util::GetParam(nh_priv_, "simulation_flag", param_sim_flag_, false);
  
  if (!param_safety_send_zero_)
  {
    ROS_WARN("[MCTL] Parameter safety_send_zero is disabled.");
  }

  if (param_sim_flag_) {
    ROS_WARN("[MCTL] Simulation flag enabled, loading user-defined drone parameters.");
    beb_maxtilt_rad_ = angles::from_degrees(20.0);
    beb_max_speed_vert_m_ = 1.0;
    beb_max_speed_rot_rad_ = angles::from_degrees(100.0);
    beb_param_recv_ = true;
  }

  ROS_ASSERT(param_feedback_pred_factor_ > 0.0 && param_feedback_pred_factor_ <= 1.0);
  ROS_ASSERT(param_delay_compensation_factor_ > 0.0 && param_delay_compensation_factor_ <= 1.0);

  // We initialize PIDs through its nodehandle constructor,
  // The following will set some default values for the parameters if the user
  // does not specify them. This plays nice with Dynamic Reconfigure
  nh_pid_px_.setParam("p", nh_pid_px_.param("p", 0.1));
  nh_pid_px_.setParam("i", nh_pid_px_.param("i", 0.0));
  nh_pid_px_.setParam("d", nh_pid_px_.param("d", 0.01));
  nh_pid_px_.setParam("i_clamp", nh_pid_px_.param("i_clamp", 0.02));

  nh_pid_py_.setParam("p", nh_pid_py_.param("p", 0.1));
  nh_pid_py_.setParam("i", nh_pid_py_.param("i", 0.0));
  nh_pid_py_.setParam("d", nh_pid_py_.param("d", 0.01));
  nh_pid_py_.setParam("i_clamp", nh_pid_py_.param("i_clamp", 0.02));

  nh_pid_yaw_.setParam("p", nh_pid_yaw_.param("p", 0.5));
  nh_pid_yaw_.setParam("i", nh_pid_yaw_.param("i", 0.0));
  nh_pid_yaw_.setParam("d", nh_pid_yaw_.param("d", 0.01));
  nh_pid_yaw_.setParam("i_clamp", nh_pid_yaw_.param("i_clamp", 0.02));

  nh_pid_alt_.setParam("p", nh_pid_alt_.param("p", 0.3));
  nh_pid_alt_.setParam("i", nh_pid_alt_.param("i", 0.0));
  nh_pid_alt_.setParam("d", nh_pid_alt_.param("d", 0.02));
  nh_pid_alt_.setParam("i_clamp", nh_pid_alt_.param("i_clamp", 0.02));

  ROS_ASSERT(pid_px_ && pid_py_ && pid_yaw_ && pid_alt_);

  pid_px_->init(nh_pid_px_);
  pid_py_->init(nh_pid_py_);
  pid_yaw_->init(nh_pid_yaw_);
  pid_alt_->init(nh_pid_alt_);

  // AMV
  model_ = boost::make_shared<beb_mr_ctrl::SystODEs>(param_mr_factor_);
  
  //subsync_bebop_.registerCallback(boost::bind(&BebMrCtrl::BebopSyncCallback, this, _1, _2, _3));

  if (param_man_ovrd_sim_) {  // In case of manual simulation override we make the thrust equatl to
    double inputs[4] = {mass*grav, 0.0, 0.0, 0.0};  // weight of the vehicle

    model_->Reset(model_->GetStates(), inputs);
    //double* st = model_->GetStates();
    //double* in = model_->GetInputs();    

    /*ROS_WARN_STREAM("[MCTL] Initial states: "
                    << state[0] << " " << state[1] << " " <<state[2]);
    ROS_WARN_STREAM("[MCTL] Initial states: "
                    << st[0] << " " << st[1] << " " <<st[2]);
    ROS_WARN_STREAM("[MCTL] Initial inputs: "
                    << in[0] << " " << in[1] << " " << in[2] << " "<< in[3]);*/
  }
  
  beb_pos_curr[0] = 0.0;
  beb_pos_curr[1] = 0.0;
  beb_pos_curr[2] = 0.0;
  beb_ang_curr[0] = 0.0;
  beb_ang_curr[1] = 0.0;
  beb_ang_curr[2] = 0.0;
}


void BebMrCtrl::SlamPoseCallback(const geometry_msgs::PoseStampedConstPtr& pose_ptr)
{
  slam_recv_time_ = ros::Time::now();

  if (!beb_param_recv_)
  {
    // This is sketchy, I need to find a way to get these params
    if (!util::GetParam(nh_, "/bebop/bebop_driver/PilotingSettingsMaxTiltCurrent", 
                        beb_maxtilt_rad_))
    {
      return;
    }
    beb_maxtilt_rad_ = angles::from_degrees(beb_maxtilt_rad_);
    if (!util::GetParam(nh_, "/bebop/bebop_driver/SpeedSettingsMaxVerticalSpeedCurrent", 
                        beb_max_speed_vert_m_))
    {
      return;
    }
    if (!util::GetParam(nh_, "/bebop/bebop_driver/SpeedSettingsMaxRotationSpeedCurrent", 
                        beb_max_speed_rot_rad_))
    {
      return;
    }
    beb_max_speed_rot_rad_ = angles::from_degrees(beb_max_speed_rot_rad_);

    // use first yaw as ref point
    beb_param_recv_ = true;
  }

  // AMV
  tf::Quaternion qc(pose_ptr->pose.orientation.x, pose_ptr->pose.orientation.y,
                    pose_ptr->pose.orientation.z, pose_ptr->pose.orientation.w);
  tf::Matrix3x3 mc(qc);
  double cam_roll_rad, cam_pitch_rad, cam_yaw_rad; 
  mc.getRPY(cam_roll_rad, cam_pitch_rad, cam_yaw_rad);

  // Camera angle transformation performed in the ORB-SLAM2 node
  // Estimate linear and angular velocities
  beb_pos_prev[0] = beb_pos_curr[0];  
  beb_pos_prev[1] = beb_pos_curr[1];
  beb_pos_prev[2] = beb_pos_curr[2];
  beb_ang_prev[0] = beb_ang_curr[0];
  beb_ang_prev[1] = beb_ang_curr[1];
  beb_ang_prev[2] = beb_ang_curr[2];
  
  beb_pos_curr[0] = - pose_ptr->pose.position.y;
  beb_pos_curr[1] = - pose_ptr->pose.position.x;
  beb_pos_curr[2] = - pose_ptr->pose.position.z;
  beb_ang_curr[0] = - cam_pitch_rad;
  beb_ang_curr[1] = - cam_roll_rad;
  beb_ang_curr[2] = - cam_yaw_rad;
  
  double* so_states = model_->GetStates();
  so_states[0] = beb_pos_curr[0];
  so_states[1] = beb_pos_curr[1];
  so_states[2] = beb_pos_curr[2];

  //so_states[6] = FILTER_SMALL_VALS(beb_roll_rad_, 0.02);
  //so_states[7] = FILTER_SMALL_VALS(beb_pitch_rad_, 0.02);
  so_states[6] = beb_ang_curr[0];
  so_states[7] = beb_ang_curr[1];
  so_states[8] = beb_ang_curr[2];
  
  ros::Duration sl_dt = slam_recv_time_ - slam_last_time_;
  if (sl_dt.toSec() > 1.0/(2.0*param_update_freq_)) {  // Condition to avoid division by zero
    so_states[3] = (beb_pos_curr[0] - beb_pos_prev[0])/sl_dt.toSec();
    so_states[4] = (beb_pos_curr[1] - beb_pos_prev[1])/sl_dt.toSec();
    so_states[5] = (beb_pos_curr[2] - beb_pos_prev[2])/sl_dt.toSec();
    so_states[9] = (beb_ang_curr[0] - beb_ang_prev[0])/sl_dt.toSec();
    so_states[10] = (beb_ang_curr[1] - beb_ang_prev[1])/sl_dt.toSec();
    so_states[11] = (beb_ang_curr[2] - beb_ang_prev[2])/sl_dt.toSec();
  }
  slam_last_time_ = ros::Time::now();
  
  // Debug message update
  msg_debug_.beb_posx = so_states[0]; 
  msg_debug_.beb_posy = so_states[1];
  msg_debug_.beb_posz = so_states[2];
  msg_debug_.beb_velx = so_states[3];
  msg_debug_.beb_vely = so_states[4];
  msg_debug_.beb_velz = so_states[5];
  msg_debug_.beb_roll_rad = so_states[6];
  msg_debug_.beb_pitch_rad = so_states[7];
  msg_debug_.beb_yaw_rad = so_states[8];
  msg_debug_.beb_p_rads = so_states[9];
  msg_debug_.beb_q_rads = so_states[10];
  msg_debug_.beb_r_rads = so_states[11];

  /*ROS_WARN_STREAM("[MCTL] Curr beb pos: "  // AMV
                  << so_states[0] << " " << so_states[1] << " " << so_states[2]);
  ROS_WARN_STREAM("[MCTL] Curr beb vel: "
                  << so_states[3] << " " << so_states[4] << " " << so_states[5]);
  ROS_WARN_STREAM("[MCTL] Curr beb rpy: "  
                  << angles::to_degrees(so_states[6]) << " " 
                  << angles::to_degrees(so_states[7]) << " " 
                  << angles::to_degrees(so_states[8]));
  ROS_WARN_STREAM("[MCTL] Curr beb pqr : "
                  << so_states[9] << " " << so_states[10] << " " << so_states[11]);*/

  //if (ctrl_enabled_ || param_man_ovrd_sim_) {  // Do not simulate if controller is disabled
  if (ctrl_enabled_ && param_man_ovrd_sim_) {
    model_->Simulate(param_time_delay_, 0.005);

    // Apply delay compensation factor (captures overshoot error in descritization)
    //so_states = model_->GetStates();  // not necessary with pointers
    //so_states[0] = so_states[0] * param_delay_compensation_factor_;  // x, y
    //so_states[1] = so_states[1] * param_delay_compensation_factor_;
    so_states[3] = so_states[3] * param_delay_compensation_factor_;  // vx, vy
    so_states[4] = so_states[4] * param_delay_compensation_factor_;
    so_states[9] = so_states[9] * param_delay_compensation_factor_;  // p, q
    so_states[10] = so_states[10] * param_delay_compensation_factor_;
    
    //model_->Reset(so_states, model_->GetInputs());  // not necessary with pointers
  }

  /*ROS_WARN_STREAM("[MCTL] Sim beb pos: "  // AMV
                  << so_states[0] << " " << so_states[1] << " " << so_states[2]);
  ROS_WARN_STREAM("[MCTL] Sim beb vel: "
                  << so_states[3] << " " << so_states[4] << " " << so_states[5]);
  ROS_WARN_STREAM("[MCTL] Sim beb rpy: "  
                  << angles::to_degrees(so_states[6]) << " " 
                  << angles::to_degrees(so_states[7]) << " " 
                  << angles::to_degrees(so_states[8]));
  ROS_WARN_STREAM("[MCTL] Sim beb pqr : "
                  << so_states[9] << " " << so_states[10] << " " << so_states[11]);*/

  // AMV
  const ros::Duration& slam_cb_lag = ros::Time::now() - slam_recv_time_;
  ROS_WARN_STREAM("[MCTL] Slam cb lag: " << slam_cb_lag);

  // Debug message update
  msg_debug_.beb_sync_time = slam_recv_time_;
  msg_debug_.beb_sync_lag = ros::Duration(0.0);
  msg_debug_.pred_delay_posx = so_states[0];
  msg_debug_.pred_delay_posy = so_states[1];
  msg_debug_.pred_delay_posz = so_states[2];
  msg_debug_.pred_delay_velx = so_states[3];
  msg_debug_.pred_delay_vely = so_states[4];
  msg_debug_.pred_delay_velz = so_states[5];
  msg_debug_.pred_delay_roll_rad = so_states[6];
  msg_debug_.pred_delay_pitch_rad = so_states[7];
  msg_debug_.pred_delay_yaw_rad = so_states[8];
  msg_debug_.pred_delay_p_rads = so_states[9];
  msg_debug_.pred_delay_q_rads = so_states[10];
  msg_debug_.pred_delay_r_rads = so_states[11];
}

// AMV
void BebMrCtrl::CtrlEnableCallback(const std_msgs::Bool& enable_msg)
{
  ctrl_enabled_ = enable_msg.data;

  //double inputs[4] = {mass*grav, 0.0, 0.0, 0.0};
  //double state[12] = {0.0, 0.0, beb_alt_m_, };
  ROS_WARN_STREAM("[MCTL] Enable callback, ctrl_enabled_: " << ctrl_enabled_);
  reset_counter_ = 0;
}

// AMV
void BebMrCtrl::ModelResetCallback(const std_msgs::Empty& empty_msg)
{
  model_->Reset();
  ctrl_enabled_ = false;
  ROS_WARN_STREAM("[MCTL] Model reset callback, ctrl_enabled_: " << ctrl_enabled_);

  if (param_man_ovrd_sim_) {  // In case of manual simulation override we make the thrust equatl to
    double inputs[4] = {mass*grav, 0.0, 0.0, 0.0};  // the weight of the vehicle
    model_->Reset(model_->GetStates(), inputs);  
  }
}

// AMV
void BebMrCtrl::SetpointPoseCallback(const geometry_msgs::PoseConstPtr& pose_ptr)
{
  setpoint_recv_time_ = ros::Time::now();
  setpoint_pose_ = *pose_ptr;
  
  ROS_WARN_STREAM("[MCTL] Setpoint: " 
                  << setpoint_pose_.position.x << " "
                  << setpoint_pose_.position.y << " "
                  << setpoint_pose_.position.z << " "
                  << setpoint_pose_.orientation.z);

  // Update debug msg
  msg_debug_.setpoint_time = setpoint_recv_time_;
  msg_debug_.setpoint_lag = ros::Duration(0.0);
  msg_debug_.setpoint = setpoint_pose_;
}


bool BebMrCtrl::Update()
{
  if (!beb_param_recv_)
  {
    ROS_WARN_THROTTLE(1, "[MCTL] Bebop params are not set!");
    return false;
  }

  // This condition is already checked by the main loop for safety
  const ros::Time& t_now = ros::Time::now();
  const ros::Duration& feedback_lag = t_now - slam_recv_time_;
  const ros::Duration& setpoint_lag = t_now - setpoint_recv_time_;
  msg_debug_.beb_sync_lag = feedback_lag;
  msg_debug_.setpoint_lag = setpoint_lag;

  if (feedback_lag.toSec() > 1.0) return false;

  // AMV
  // CLAMP Input Setpoints
  /*setpoint_cmd_vel.linear.x = CLAMP(setpoint_cmd_vel.linear.x, -param_max_linear_vel_, param_max_linear_vel_);
  setpoint_cmd_vel.linear.y = CLAMP(setpoint_cmd_vel.linear.y, -param_max_linear_vel_, param_max_linear_vel_);
  setpoint_cmd_vel.linear.z = CLAMP(setpoint_cmd_vel.linear.z, -beb_max_speed_vert_m_, beb_max_speed_vert_m_);
  setpoint_cmd_vel.angular.z = CLAMP(setpoint_cmd_vel.angular.z, -beb_max_speed_rot_rad_, beb_max_speed_rot_rad_);*/

  // PID Control Loop
  ros::Duration dt = t_now - pid_last_time_;
  if (dt.toSec() > (2.0 / param_update_freq_))
  {
    pid_last_time_ = ros::Time::now();
    dt = ros::Duration(0.0);
    pid_px_->reset();
    pid_py_->reset();
    pid_alt_->reset();
    pid_yaw_->reset();
  }

  // AMV 
  //setpoint.orientation.z is an angle in radians
  double* so_states = model_->GetStates();
  double pos_errors[3];  // Array of position errors: x, y, z
  pos_errors[0] = setpoint_pose_.position.x - so_states[0];
  pos_errors[1] = setpoint_pose_.position.y - so_states[1];
  pos_errors[2] = setpoint_pose_.position.z - so_states[2];
  const double cmdX = pid_px_->computeCommand(pos_errors[0], dt);  // pitch_ref
  const double cmdY = pid_py_->computeCommand(pos_errors[1], dt);  // roll_ref
  const double vyaw_ref = pid_yaw_->computeCommand(angles::normalize_angle(
                              setpoint_pose_.orientation.z - so_states[8]), dt);
  double vz_ref = pid_alt_->computeCommand(pos_errors[2], dt); 
 
  //TO-DO: try with beb_yaw_rad_
  double pitch_ref = sqrt(cmdX*cmdX + cmdY*cmdY)*cos(atan2(cmdY, cmdX) - so_states[8]);  
  double roll_ref = sqrt(cmdX*cmdX + cmdY*cmdY)*sin(atan2(cmdY, cmdX) - so_states[8]);

  // Position control bypass
  //const double pitch_ref = 0.0;
  //const double roll_ref = 0.0;
  //const double vz_ref = 0.0;

  // Convert PID output  into normalized cmd_vel (-1 -> 1)
  util::ResetCmdVel(ctrl_twist_);
  ctrl_twist_.linear.x =  pitch_ref / beb_maxtilt_rad_;
  ctrl_twist_.linear.y =  roll_ref / beb_maxtilt_rad_;
  ctrl_twist_.angular.z = vyaw_ref / beb_max_speed_rot_rad_;
  ctrl_twist_.linear.z = vz_ref / beb_max_speed_vert_m_;

  // CLAMP and filter output
  ctrl_twist_.linear.x = CLAMP(ctrl_twist_.linear.x, -1.0, 1.0);
  ctrl_twist_.linear.y = CLAMP(ctrl_twist_.linear.y, -1.0, 1.0);
  ctrl_twist_.linear.z = CLAMP(ctrl_twist_.linear.z, -1.0, 1.0);
  ctrl_twist_.angular.z = CLAMP(ctrl_twist_.angular.z, -1.0, 1.0);

  FILTER_SMALL_VALS(ctrl_twist_.linear.x, 0.01);
  FILTER_SMALL_VALS(ctrl_twist_.linear.y, 0.01);
  FILTER_SMALL_VALS(ctrl_twist_.linear.z, 0.01);
  FILTER_SMALL_VALS(ctrl_twist_.angular.z, 0.01);
  
  if (param_xy_hover && (fabs(pos_errors[0]) < 5e-2) && (fabs(pos_errors[1]) < 5e-2))
  {
    ROS_WARN_ONCE("[MCTL] Parameter xy_hover is enabled and the condition is met, sending vx=0, vy=0");
    ctrl_twist_.linear.x = 0.0;
    ctrl_twist_.linear.y = 0.0;
  }

  if (so_states[2] < param_min_alt_ && ctrl_twist_.linear.z < 0.0)
  {
    ROS_WARN_STREAM_THROTTLE(1, "[MCTL] Minimum altitude safety is triggered at the altitude of " 
                             << so_states[2] << ". Going down is blocked.");
    ctrl_twist_.linear.z = 0.0;
  }

  if (so_states[2] > param_max_alt_ && ctrl_twist_.linear.z > 0.0)
  {
    ROS_WARN_STREAM_THROTTLE(1, "[MCTL] Maximum altitude safety is triggered at the altitude of " 
                             << so_states[2] << ". Going up is blocked.");
    ctrl_twist_.linear.z = 0.0;
  }

  // AMV
  // TO-DO: Implement low-pass filtering
  // Compute derivatives and estimate the inputs (force and torques)
  // Arrays are used to store current and previous values
  roll_cmd_[1] = roll_cmd_[0];
  //roll_cmd_[0] = ctrl_tiwst_.linear.y*beb_maxtilt_rad_;
  roll_cmd_[0] = CLAMP(ctrl_twist_.linear.y*beb_maxtilt_rad_, -beb_maxtilt_rad_, 
                        beb_maxtilt_rad_) * param_feedback_pred_factor_;
  pitch_cmd_[1] = pitch_cmd_[0];
  //pitch_cmd_[0] = ctrl_twist_.linear.x*beb_maxtilt_rad_;
  pitch_cmd_[0] = CLAMP(ctrl_twist_.linear.x*beb_maxtilt_rad_, -beb_maxtilt_rad_, 
                        beb_maxtilt_rad_) * param_feedback_pred_factor_;
  vyaw_cmd_[1] = vyaw_cmd_[0];
  //vyaw_cmd_[0] = ctrl_twist_.angular.z*beb_max_speed_rot_rad_;
  vyaw_cmd_[0] = CLAMP(ctrl_twist_.angular.z*beb_max_speed_rot_rad_, -beb_max_speed_rot_rad_, 
                       beb_max_speed_rot_rad_) * param_feedback_pred_factor_;
  vz_cmd_[1] = vz_cmd_[0];
  //vz_cmd_[0] = ctrl_twist_.linear.z*beb_max_speed_vert_m_;
  vz_cmd_[0] = CLAMP(ctrl_twist_.linear.z*beb_max_speed_vert_m_, -beb_max_speed_vert_m_, 
                     beb_max_speed_vert_m_) * param_feedback_pred_factor_;

  double roll_dderiv, pitch_dderiv, vyaw_deriv, vz_deriv;
  double inputs[4];
  if (dt.toSec() > 1.0/(2.0*param_update_freq_)) {  // Condition to avoid division by zero
  
    roll_deriv_[1] = roll_deriv_[0];
    roll_deriv_[0] = (roll_cmd_[0] - roll_cmd_[1])/dt.toSec();
    pitch_deriv_[1] = pitch_deriv_[0];
    pitch_deriv_[0] = (pitch_cmd_[0] - pitch_cmd_[1])/dt.toSec();

    roll_dderiv = (roll_deriv_[0] - roll_deriv_[1])/dt.toSec();
    pitch_dderiv = (pitch_deriv_[0] - pitch_deriv_[1])/dt.toSec();
    vyaw_deriv = (vyaw_cmd_[0] - vyaw_cmd_[1])/dt.toSec();
    vz_deriv = (vz_cmd_[0] - vz_cmd_[1])/dt.toSec();
  
    inputs[0] = mass*(grav + vz_deriv);
    inputs[1] = Ixx*roll_dderiv;
    inputs[2] = Iyy*pitch_dderiv;
    inputs[3] = Izz*vyaw_deriv;

  // Simulate Bebop's dynamics given the inptus to update (predict) feedback
  // this to compensate for low frequency velocity feedback. We start from the 
  // current estimate of the model anted forward it in time for dt seconds, so 
  // next time this function is called, the updated feedback is ready to be 
  // used. When the feedback comes from Bebop, it will reset the model. We again 
  // convert from normalized values to angles to take into effect any 
  // clamp/filtering effects. Also apply the feedback prediction factor which 
  // indicates to what extent the desired state will be achieved during the 
  // course of dt
  //model_->Reset(sta_arr[0], inputs[0]);
    if (param_man_ovrd_sim_) {
      model_->Reset(so_states, inputs);
      model_->Simulate(dt.toSec(), 0.005);
    }
  }

  pid_last_time_ = ros::Time::now();

  pub_ctrl_cmd_vel_.publish(ctrl_twist_);

  //ROS_WARN_STREAM("[MCTL] Inputs: "  // AMV
  //                << inputs[0] << " " << inputs[1] << " " << inputs[2] << " " << inputs[3]); 
  //ROS_WARN_STREAM("[MCTL] Sim ctl RPY: "
  //                << so_states[6] << " " << so_states[7] << " " << so_states[8]);
  /*ROS_WARN_STREAM("[MCTL] cmd: "
                  << ctrl_twist_.linear.x << " " << ctrl_twist_.linear.y << " "
                  << ctrl_twist_.linear.z << " " << ctrl_twist_.angular.z);*/

  //ROS_DEBUG_STREAM("[MCTL] CMD_VEL for: " << ctrl_twist_.linear.x <<
  //                 " left: " << ctrl_twist_.linear.y <<
  //                " up: " << ctrl_twist_.linear.z <<
  //                 " cw: " << ctrl_twist_.angular.z);

  // Update debug message
  msg_debug_.setpoint_filt = setpoint_pose_;  // AMV
  msg_debug_.pred_posx = so_states[0];
  msg_debug_.pred_posy = so_states[1];
  msg_debug_.pred_posz = so_states[2];
  msg_debug_.pred_velx = so_states[3];
  msg_debug_.pred_vely = so_states[4];
  msg_debug_.pred_velz = so_states[5];
  msg_debug_.pred_roll_rad = so_states[6];
  msg_debug_.pred_pitch_rad = so_states[7];
  msg_debug_.pred_yaw_rad = so_states[8];
  msg_debug_.pred_p_rads = so_states[9];
  msg_debug_.pred_q_rads = so_states[10];
  msg_debug_.pred_r_rads = so_states[11];
  return true;
}


void BebMrCtrl::Reset()
{
  util::ResetCmdVel(ctrl_twist_);
  
  // Publish the command only once when the controller is disabled
  if (param_safety_send_zero_ && reset_counter_ < 2) {
    roll_cmd_[0] = 0;  // Reset cmds, derivatives, and model
    pitch_cmd_[0] = 0;
    vyaw_cmd_[0] = 0;
    vz_cmd_[0] = 0;
    roll_deriv_[0] = 0;
    pitch_deriv_[0] = 0;
    double inputs[4] = {mass*grav, 0.0, 0.0, 0.0};
    double states[12] = {0.0};
    model_->Reset(states, inputs);

    pub_ctrl_cmd_vel_.publish(ctrl_twist_);
  }
} 


void BebMrCtrl::Spin()
{
  ROS_INFO("[MCTL] Spinnig");

  // Safety
  Reset();

  ros::Rate loop_rate(param_update_freq_);

  pid_last_time_ = ros::Time::now();
  while (ros::ok())
  {
    try
    {
      bool do_reset = false;
      bool ctrl_success = false;

      // AMV
      if (!ctrl_enabled_) {
        ROS_WARN_THROTTLE(10.0, "[MCTL] Controller is disabled.");
        do_reset = true;
        reset_counter_++;
      }
      else if ((ros::Time::now() - slam_recv_time_).toSec() > 0.15)
      {
        ROS_WARN_THROTTLE(10.0, "[MCTL] State feedback is older than 0.1 s! Resetting.");
        ctrl_enabled_ = false;  // Disable controller
        do_reset = true;
        reset_counter_ = 0;
      }

      if (do_reset)
      {
        Reset();
      }
      else
      {
        ctrl_success = Update();
      }

      msg_debug_.control_active = ctrl_success;
      msg_debug_.header.stamp = ros::Time::now();
      pub_debug_.publish(msg_debug_);

      ros::spinOnce();
      if (!loop_rate.sleep())
      {
        ROS_WARN_STREAM("[MCTL] Missed loop update rate of " << param_update_freq_);
      }
    }
    catch (const std::runtime_error& e)
    {
      ROS_ERROR_STREAM("[MCTL] Runtime Exception: " << e.what());
      Reset();
    }
  }
}

}  // namespace bebop_mr_ctrl

