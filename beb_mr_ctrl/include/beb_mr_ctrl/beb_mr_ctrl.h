#ifndef BEB_MR_CTRL
#define BEB_MR_CTRL

#include <boost/shared_ptr.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <geometry_msgs/Twist.h>
#include <control_toolbox/pid.h>
// AMV
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

/*#include <bebop_msgs/Ardrone3PilotingStateAltitudeChanged.h>
#include <bebop_msgs/Ardrone3PilotingStateAttitudeChanged.h>
#include <bebop_msgs/Ardrone3PilotingStateSpeedChanged.h>*/

#include "beb_mr_ctrl/Debug.h"
#include "beb_mr_ctrl/syst_odes.h"


namespace beb_mr_ctrl
{

namespace util {

inline void ResetCmdVel(geometry_msgs::Twist& v)
{
  v.linear.x = 0.0;
  v.linear.y = 0.0;
  v.linear.z = 0.0;
  v.angular.x = 0.0;
  v.angular.y = 0.0;
  v.angular.z = 0.0;
}

template<typename T>
bool GetParam(const ros::NodeHandle& nh, const::std::string& key, T& val)
{
  if (nh.getParam(key, val))
  {
    ROS_INFO_STREAM("[MCTL] Param " << key << " : " << val);
    return true;
  }
  ROS_WARN_STREAM("[MCTL] Param " << key << " not found/set.");
  return false;
}

template<typename T>
bool GetParam(const ros::NodeHandle& nh, const::std::string& key, T& val, const T& default_val)
{
  nh.param(key, val, default_val);
  ROS_INFO_STREAM("[MCTL] Param " << key << " : " << val);
}

}  // namespace util


/*typedef message_filters::sync_policies::ApproximateTime<
    bebop_msgs::Ardrone3PilotingStateAltitudeChanged,
    bebop_msgs::Ardrone3PilotingStateAttitudeChanged,
    bebop_msgs::Ardrone3PilotingStateSpeedChanged> BebopSyncPolicy_t;*/


class BebMrCtrl
{
protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  // This is required for proper initialization of PID dynamic reconfigure
  ros::NodeHandle nh_pid_px_;
  ros::NodeHandle nh_pid_py_;
  ros::NodeHandle nh_pid_yaw_;
  ros::NodeHandle nh_pid_alt_;

  /*message_filters::Subscriber<bebop_msgs::Ardrone3PilotingStateAltitudeChanged> sub_bebop_alt_;
  message_filters::Subscriber<bebop_msgs::Ardrone3PilotingStateAttitudeChanged> sub_bebop_att_;
  message_filters::Subscriber<bebop_msgs::Ardrone3PilotingStateSpeedChanged> sub_bebop_speed_;
  message_filters::Synchronizer<BebopSyncPolicy_t> subsync_bebop_;*/
  ros::Subscriber sub_slam_pose_;

  ros::Subscriber sub_ctrl_enable_;  // AMV
  ros::Subscriber sub_setpoint_pose_;
  ros::Publisher pub_ctrl_cmd_vel_;
  ros::Subscriber sub_model_reset_; 
  ros::Publisher pub_debug_;

  geometry_msgs::Pose setpoint_pose_;
  geometry_msgs::Twist ctrl_twist_;
  //geometry_msgs::Twist ctrl_twist_prev_;

  // Bebop dynamic models for pitch->vx and roll->vy
  boost::shared_ptr<SystODEs> model_;

  // Bebop internal params (driver reads/sets them)
  bool beb_param_recv_;
  double beb_maxtilt_rad_;
  double beb_max_speed_vert_m_;
  double beb_max_speed_rot_rad_;

  // Params
  /* If any of these two params are set, the corresponding
   * degree of freedom is conrolled in positional space (not velocity space).
   * Input setpoint is treated as an absolute value
   * */
  bool param_abs_yaw_ctrl_;
  bool param_abs_alt_ctrl_;

  int param_mr_factor_;
  double param_time_delay_;
  double param_update_freq_;
  bool param_safety_send_zero_;
  bool param_xy_hover;

  double param_max_linear_vel_;  // m/s
  double param_min_alt_;
  double param_max_alt_;
  double param_feedback_pred_factor_;
  double param_delay_compensation_factor_;

  ros::Time slam_recv_time_;
  ros::Time slam_last_time_;
  
  // Bebop States
  double beb_pos_prev[3];
  double beb_ang_prev[3];
  double beb_pos_curr[3];
  double beb_ang_curr[3];

  // PID Controllers
  ros::Time setpoint_recv_time_;
  ros::Time pid_last_time_;
  boost::shared_ptr<control_toolbox::Pid> pid_px_;
  boost::shared_ptr<control_toolbox::Pid> pid_py_;
  boost::shared_ptr<control_toolbox::Pid> pid_yaw_;
  boost::shared_ptr<control_toolbox::Pid> pid_alt_;

  beb_mr_ctrl::Debug msg_debug_;

  // Internal
  //double beb_vx_pred_m_;
  //double beb_vy_pred_m_;

  // AMV
  bool ctrl_enabled_;
  bool param_man_ovrd_sim_;
  bool param_sim_flag_;
  unsigned char reset_counter_;
  double roll_cmd_[2];
  double pitch_cmd_[2];
  double vyaw_cmd_[2];
  double vz_cmd_[2];
  double roll_deriv_[2];
  double pitch_deriv_[2];

  void SlamPoseCallback(const geometry_msgs::PoseStampedConstPtr& pose_ptr);

  // PID control happens here
  void CtrlEnableCallback(const std_msgs::Bool& enable_msg);  // AMV
  void SetpointPoseCallback(const geometry_msgs::PoseConstPtr& pose_ptr);
  void ModelResetCallback(const std_msgs::Empty& empty_msg);

  bool Update();

public:
  BebMrCtrl(ros::NodeHandle& nh);

  void Reset();
  virtual void Spin();
};

}  // namespace beb_mr_ctrl

#endif  // BEB_MR_CTRL
