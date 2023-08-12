#include "pole_angle_measurement/subscriber_pole_angle.hpp" // TOPIC is defined here

#ifdef __XENO__
#include <native/task.h>
#include <sys/mman.h>
#endif

#define RATE 200
#define TOPIC "pole_angle"

int run(int argc, char** argv){


  // starting node
  ros::init(argc,argv,"pole_angle_broadcast_test");

  struct rosrt::InitOptions options;
  options.pubmanager_thread_name = "pole_angle_broadcast_test";

  rosrt::init(options);

  ros::NodeHandle node;

  // ros::Publisher publisher = node.advertise<std_msgs::Float64>(TOPIC,0);
  rosrt::Publisher<std_msgs::Float64> pub(node.advertise<std_msgs::Float64>(TOPIC, 1), 10, std_msgs::Float64());

  // ros publisher of the angle
  std_msgs::Float64Ptr msg = pub.allocate();

  // setting loop rate
  ros::Rate loop_rate(RATE);

  float c = 0.0;

  // publishing at desired rate
  while ( ros::ok() ){
    // msg.data = (double)( rand() % 100 ) ;
    msg->data = c++;
    // msg->data = 0.0;
    pub.publish(msg);
    // ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;

}


int main(int argc, char** argv){
#ifdef __XENO__
  mlockall(MCL_CURRENT | MCL_FUTURE);
#endif
  run(argc,argv);
}