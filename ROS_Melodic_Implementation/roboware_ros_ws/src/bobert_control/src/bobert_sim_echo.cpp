#include <ros/ros.h>
#include <bobert_control/armCmd.h>
#include <bobert_control/bobertTelemetry.h>

ros::Publisher telem_pub;

void cmdCallback(const bobert_control::armCmd::ConstPtr &msg){
    
    static bobert_control::bobertTelemetry telem;

    for(int i = 0; i < msg->angle.size(); i++){
        telem.angle[i] = msg->angle[i];
        //telem.vel[i] = msg->vel[i];
    }

    telem_pub.publish(telem);

}

int main(int argc, char ** argv){
    ros::init(argc, argv, "bobert_sim_echo");

    ros:: NodeHandle n;

    ros::Subscriber cmd_sub = n.subscribe("teensy/armCmd", 10, cmdCallback);

    telem_pub = n.advertise<bobert_control::bobertTelemetry>("teensy/bobertTelemetry", 10);

    ros::spin();
}