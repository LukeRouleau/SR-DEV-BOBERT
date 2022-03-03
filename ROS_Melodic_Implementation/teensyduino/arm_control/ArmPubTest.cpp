#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"

#include <sstream>

using namespace std;

std_msgs::Float64 servo_positions[6];

ros::Publisher pubs[6];

void callback(const sensor_msgs::JointState &msg){
	for(int i = 0; i < 6; i++){
		servo_positions[i].data = msg.positions[i];
		pubs[i].publish(servo_positions[i]);
	}
}

int main(int argc, char **argv){

    ros::init(argc, argv, "teensy_node");
  
    ros::NodeHandle n;
  
    //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  
    //ros::Rate loop_rate(10);
  
    /*int count = 0;
    while (ros::ok()){
      std_msgs::String msg;
  
      std::stringstream ss;
      ss << "hello world " << count;
      msg.data = ss.str();
  
      ROS_INFO("%s", msg.data.c_str());
  
      chatter_pub.publish(msg);
  
      ros::spinOnce();
  
      loop_rate.sleep();
      ++count;
    }*/
	
	pubs[0] = n.advertise<std_msgs::Float64>("servo_position_0", 100);
	pubs[1] = n.advertise<std_msgs::Float64>("servo_position_1", 100);
	pubs[2] = n.advertise<std_msgs::Float64>("servo_position_2", 100);
	pubs[3] = n.advertise<std_msgs::Float64>("servo_position_3", 100);
	pubs[4] = n.advertise<std_msgs::Float64>("servo_position_4", 100);
	pubs[5] = n.advertise<std_msgs::Float64>("servo_position_5", 100);
	
	ros::Rate loop_rate(10);
  
	for(int i = 0; i < 6; i++){
		servo_positions[i].data = 0;
	}
  
	//ros::Subscriber sub = n.subscribe("bobert/joint_states", 10000, callback);

	while(ros::ok()){
		
		for(int i = 0; i < 6; i++){
			if(servo_positions[i].data >= 170){
				servo_positions[i].data = 10;
			}
			else{
				servo_positions[i].data += 10;
			}
		}
		
		for(int i = 0; i < 6; i++){
			pubs[i].publish(servo_positions[i]);
		}

		ros::spinOnce();
		
		loop_rate.sleep();
		
	}
	
	ros::spin();
  
    return 0;
}