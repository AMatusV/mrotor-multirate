# header time is set to when we send the message (update loop_
Header header

bool control_active

# bebop sync time is set when we receive a state
time beb_sync_time

# the lag between header.stamp and bebop_sync_time
duration beb_sync_lag

# Orientation is used as Euler angles in radians (w is not used)
float32 beb_posx
float32 beb_posy
float32 beb_posz
float32 beb_velx
float32 beb_vely
float32 beb_velz
float32 beb_roll_rad
float32 beb_pitch_rad
float32 beb_yaw_rad
float32 beb_p_rads
float32 beb_q_rads
float32 beb_r_rads

# only transimission delay compensated
float32 pred_delay_posx
float32 pred_delay_posy
float32 pred_delay_posz
float32 pred_delay_velx
float32 pred_delay_vely
float32 pred_delay_velz
float32 pred_delay_roll_rad
float32 pred_delay_pitch_rad
float32 pred_delay_yaw_rad
float32 pred_delay_p_rads
float32 pred_delay_q_rads
float32 pred_delay_r_rads

# transmission delay + feedback lag compensated
float32 pred_posx
float32 pred_posy
float32 pred_posz
float32 pred_velx
float32 pred_vely
float32 pred_velz
float32 pred_roll_rad
float32 pred_pitch_rad
float32 pred_yaw_rad
float32 pred_p_rads
float32 pred_q_rads
float32 pred_r_rads

time setpoint_time
duration setpoint_lag

geometry_msgs/Pose setpoint
geometry_msgs/Pose setpoint_filt
