#include "pole_balancing_apollo/feedback_control.h"
#include <gtest/gtest.h>



TEST(Ros_GainsReader_tests, ros_gains_reader_test){   

  // initializing ros stuff
  int argc = 1;   
  char name[] = "ros_gains_reader_test";   
  char* argv[1];   
  argv[0] = name;   
  ros::init(argc,argv,"ros_gains_reader_test");   
  ros::NodeHandle nh;   

  // gain publisher will publish gains
  ros::Publisher gain_publisher = n.advertize<std_msgs::Float64MultiArray>("gains",1);
  std::vector<double> gains;
  for(int i=0;i<3;i++) gains.push_back(1);
  std_msgs::Float64MultiArray msg;

  // what we test. should subscribe to the gains
  ROS_GainsReader gains_reader("gains");

  // checking gains reader get the right gains
  for(int i=0;i<3;i++){

    gains[3] = i;
    msg.data = gains;
    gain_publisher.publish(msg);

    ros::Duration(0.2)::sleep();
    
    std::vector<double> read_gains;
    gains_reader.get(read_gains);

    for(int i=0;i<3;i++){
      ASSERT_EQ(gains[i],read_gains[i]);
    }
    

  }


 }
