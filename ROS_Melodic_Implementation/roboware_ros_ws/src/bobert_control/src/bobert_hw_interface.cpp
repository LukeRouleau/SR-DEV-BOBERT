
#include <bobert_control/bobert_hw_interface.h>

#define DEG_TO_RAD 0.01745329251
#define RAD_TO_DEG 57.2957795131

namespace bobert_ns
{
bobertHWInterface::bobertHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model)
  : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
{
  telemetry_sub = nh.subscribe("/teensy/bobertTelemetry", 1, &bobertHWInterface::telemetryCallback, this);

  cmd_pub = nh.advertise<bobert_control::armCmd>("/teensy/armCmd", 1);

  ROS_INFO("bobertHWInterface constructed");
}

void bobertHWInterface::telemetryCallback(const bobert_control::bobertTelemetry::ConstPtr &msg){

  for(int i = 0; i < num_joints_; i++){
    joint_position_[i] = msg->angle[i]*DEG_TO_RAD;
    //joint_velocity_[i] = msg->vel[i]*DEG_TO_RAD;
  }  

  //bufferHealth = msg->bufferHealth;

}

void bobertHWInterface::init()
{
  // Call parent class version of this function
  GenericHWInterface::init();

  //joint_position_prev_.resize(joint_position_.size());

  ROS_INFO("bobertHWInterface Ready.");
}

void bobertHWInterface::read(ros::Duration& elapsed_time)
{
  // No need to read since our write() command populates our state for us
  ros::spinOnce();
}

void bobertHWInterface::write(ros::Duration& elapsed_time)
{
  // Safety
  //enforceLimits(elapsed_time);
  static bobert_control::armCmd arm_cmd;

  for(int i = 0; i < num_joints_; i++){
    arm_cmd.angle[i] = joint_position_command_[i]*RAD_TO_DEG;
    //arm_cmd.vel[i] = joint_velocity_command_[i]*RAD_TO_DEG;
  }

  cmd_pub.publish(arm_cmd);

}

/*
void bobertHWInterface::write(ros::Duration& elapsed_time)
{
  
  static bobert_control::armCmd cmd_;

  bool changed = false;
  for(int i = 0; i < num_joints_; i++){
    if(joint_position_prev_[i] != joint_position_command_[i]){
      changed = true;
      break;
    }
  }

  if(changed){
    for(int i = 0; i < num_joints_; i++){
      cmd_.angle[i] = joint_position_command_[i] * RAD_TO_DEG;
      //cmd_.vel[i]= (fabs(joint_position_command_[i]-joint_position_prev_[i])*RAD_TO_DEG)/elapsed_time.toSec(); 
      // (must be positive for aubo) joint_velocity_command_[i]*RAD_TO_DEG; joint_velocity_command_[i] calculate my own velocities
    }
    if(bufferHealth<DESIRED_BUFFERED_POINTS>){
      cmd_.msg_ctr = cmd_.msg_ctr + 1;
      cmd_pub.publish(cmd_);
    }
  }

}
*/

void bobertHWInterface::enforceLimits(ros::Duration& period)
{
  // Enforces position and velocity
  //pos_jnt_sat_interface_.enforceLimits(period);
}

}  // namespace bobert_ns
