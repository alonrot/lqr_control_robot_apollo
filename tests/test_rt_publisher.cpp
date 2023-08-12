#include "ros/ros.h"
#include "ros/param.h"
#include "rosrt/rosrt.h"
#include <std_msgs/Float64.h>
#include <string>

#ifdef __XENO__
#include <native/task.h>
#include <sys/mman.h>
#include <xenomai/rtdk.h> // Contains the definition of rt_printf()
#endif

#include <stdlib.h> // std::stoi, std::stof...


int run(int argc, char** argv){

  // As contradictory as it sounds, the SimpleNode_RT (rosrt::init) node will handle the unavoidable real-time switches
  // The other one, SimpleNode (ros::init) will be promoted as a real-time thread through rt_task_shadow()
  // The real-time/non-real time switches can be seen in a xenomai machine with: watch -n 0,1 "cat /proc/xenomai/stat"

  argc = 1;
  char name[] = "SimpleNode";
  argv[0] = name;

  // Real-time node options:
  struct rosrt::InitOptions options_rt;
  options_rt.pubmanager_thread_name = "SimpleNode_RT"; // The name has to be different. The default is "rosrt_pubmanager"

  // Initialize non-real time ros node:
  ros::init(argc,argv,name); // This name has to be the same as in rt_task_shadow()

  // Initialize real-time ros node:
  rosrt::init(options_rt);

  // Node handle:
  ros::NodeHandle node;

  // Initialize publisher, using the non-default constructor: 
  rosrt::Publisher<std_msgs::Float64> publisher(node.advertise<std_msgs::Float64>("dummy_double", 1), 10, std_msgs::Float64());

  // Reset thread info:
  rosrt::resetThreadAllocInfo();

  // ros publisher of the angle
  std_msgs::Float64Ptr msg = publisher.allocate();

  // setting loop rate
  ros::Rate loop_rate(1000);

  double counter = 0.0;

  // publishing at desired rate
  while ( ros::ok() ){
    if(msg){
      counter += 1.3;
      msg->data = counter;
      publisher.publish(msg);
    }

    #ifdef __XENO__
      rt_task_yield();
      rt_task_sleep(1000000); // ns
    #else
      
      // Either this:
      // ros::spinOnce();
      // ros::WallDuration(1).sleep();

      // Or this:
      ros::spinOnce();
      loop_rate.sleep();

      // Note: 
      // ros::spin() calls ros::spinOnce() repeatedly in a loop. To be used when there's no loop.

    #endif
  }
  
  return 0;
  
}


int main(int argc, char** argv){
  #ifdef __XENO__
    mlockall(MCL_CURRENT | MCL_FUTURE);
    rt_task_shadow(NULL, "SimpleNode", 1, 0);
  #endif
    run(argc,argv);
}


