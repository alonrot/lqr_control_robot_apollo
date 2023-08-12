#include "ros/ros.h" 
#include "ros/master.h"
#include "std_msgs/Float32.h"

#define PUBLISH_TOPIC "pole_balancing/pole_angle_visualization"
#define FREQUENCY 100
#define SPEED 0.004;

int main(int argc, char** argv){

  // initialize ros stuff
  ros::init(argc,argv,"pole_fake_angle");
  ros::NodeHandle node;
  ros::Publisher publisher = node.advertise<std_msgs::Float32>(PUBLISH_TOPIC,5);

  // loop
  ros::Rate loop_rate(FREQUENCY);
  float angle = 0;
  float pole_angle = 0;
  std_msgs::Float32 pole_angle_msgs;
  while (ros::ok()){
    angle+=SPEED;
    pole_angle = 0.5*cos(angle);
    pole_angle_msgs.data = pole_angle;
    publisher.publish(pole_angle_msgs);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}


