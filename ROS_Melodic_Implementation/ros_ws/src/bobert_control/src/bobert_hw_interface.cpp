
#include <bobert_control/bobert_hw_interface.h>

namespace bobert_ns
{
bobertHWInterface::bobertHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model)
  : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
{
  
}

void bobertHWInterface::init()
{
  // Call parent class version of this function
  GenericHWInterface::init();

  ROS_INFO("bobertHWInterface Ready.");
}

void bobertHWInterface::read(ros::Duration& elapsed_time)
{
  // No need to read since our write() command populates our state for us
}

void bobertHWInterface::write(ros::Duration& elapsed_time)
{
  // Safety
  //enforceLimits(elapsed_time);
  
}

void bobertHWInterface::enforceLimits(ros::Duration& period)
{
  // Enforces position and velocity
  //pos_jnt_sat_interface_.enforceLimits(period);
}

}  // namespace bobert_ns
