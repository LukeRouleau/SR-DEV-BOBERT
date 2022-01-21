
#ifndef BOBERT_HW_INTERFACE_H
#define BOBERT_HW_INTERFACE_H

#include <ros_control_boilerplate/generic_hw_interface.h>
#include <bobert_control/armCmd.h>
#include <bobert_control/bobertTelemetry.h>

namespace bobert_ns
{
/** \brief Hardware interface for a robot */
class bobertHWInterface : public ros_control_boilerplate::GenericHWInterface
{
public:
  /**
   * \brief Constructor
   * \param nh - Node handle for topics.
   */
  bobertHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model = NULL);

  /** \brief Initialize the robot hardware interface */
  virtual void init();

  /** \brief Read the state from the robot hardware. */
  virtual void read(ros::Duration& elapsed_time);

  /** \brief Write the command to the robot hardware. */
  virtual void write(ros::Duration& elapsed_time);

  /** \breif Enforce limits for all values before writing */
  virtual void enforceLimits(ros::Duration& period);

protected:
  
  ros::Subscriber telemetry_sub;
  void telemetryCallback(const bobert_control::bobertTelemetry::ConstPtr &msg);

  ros::Publisher cmd_pub;
  

};  // class

}  // namespace ros_control_boilerplate

#endif
