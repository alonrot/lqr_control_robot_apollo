#include "vicon_subscriber/vicon_object.hpp"
#include "ros/ros.h"
#include "ros/param.h"
#include "rosrt/rosrt.h"
#include <std_msgs/Float64.h>
#include <string>

#ifdef __XENO__
#include <native/task.h>
#include <sys/mman.h>
#endif

#include <stdlib.h> // std::stoi, std::stof...

#define PI 3.14159265358979323846
#define RATE 200 


static inline double angle_estimation(Eigen::Matrix3d rotation_base,
				      Eigen::Matrix3d rotation_pole){

  double phi_pole;

  Eigen::Vector3d base_origin = rotation_base.eulerAngles(0,1,2);
  Eigen::Matrix3d pole_to_base_rotation_matrix = rotation_base.transpose()*rotation_pole;
  Eigen::Vector3d pole_angles = pole_to_base_rotation_matrix.eulerAngles(0,1,2);

  if ( pole_angles[1] > PI/2 )
    phi_pole = - (pole_angles[1] - PI);
  else if ( pole_angles[1] < -PI/2 )
    phi_pole = - (pole_angles[1] + PI);
  else
    phi_pole = pole_angles[1];
  
  return phi_pole;

}

static double angle_estimation(boost::shared_ptr<Vicon_object> &base, 
			       boost::shared_ptr<Vicon_object> &pole){

  return angle_estimation(base->get_rotation(),pole->get_rotation());

}



int run(int argc, char** argv){

  // Define external input arguments:
  std::string node_name;
  std::string topic_name_pole_angle;
  std::string topic_name_tracker_base;
  std::string topic_name_tracker_pole;
  int looping_rate;

  printf("[INFO] ** Starting NODE Publisher_pole_angle\n");
  printf("[INFO] ** Number of passed input arguments: %i\n",argc);
  int narg_expected = 5;

  // Parse input arguments:
  if(argc < narg_expected+1){
    printf("[ERROR] ** Missing input arguments...\n");
    printf("[ERROR] ** Exit the node...\n");
    return 0;
  }
  else{
    node_name = argv[1];
    topic_name_pole_angle = argv[2];
    topic_name_tracker_base = argv[3];
    topic_name_tracker_pole = argv[4];
    looping_rate = std::stoi(argv[5]); // string to int
  }

  // Info:
  std::cout << "[INFO] ** node_name =               " << node_name << std::endl;
  std::cout << "[INFO] ** topic_name_pole_angle =   " << topic_name_pole_angle << std::endl;
  std::cout << "[INFO] ** topic_name_tracker_base = " << topic_name_tracker_base << std::endl;
  std::cout << "[INFO] ** topic_name_tracker_pole = " << topic_name_tracker_pole << std::endl;
  std::cout << "[INFO] ** looping_rate =            " << looping_rate << std::endl;

  // starting node
  ros::init(argc,argv,node_name);

  struct rosrt::InitOptions options;
  options.pubmanager_thread_name = node_name;

  rosrt::init(options);

  ros::NodeHandle node;

  // waiting for all required topics to be broadcasted
  std::vector<std::string> required_topics;
  required_topics.push_back(topic_name_tracker_base);
  required_topics.push_back(topic_name_tracker_pole);
  ros::Rate wait_rate(10);
  while (!are_required_topics_broadcasted(required_topics) && ros::ok() ){
    wait_rate.sleep();
    ros::spinOnce();
  }
  if(!ros::ok()) return 0;

  //every required topics are broadcasted, let's subscribe to them
  for(int i=0;i<required_topics.size();i++){
    Vicon_object::add(node,i,required_topics[i]);
  }

  //getting corresponding instances of vicon_object, which subscribed to their related topic
  //and update their state accordingly
  boost::shared_ptr<Vicon_object> base = Vicon_object::get(0);
  boost::shared_ptr<Vicon_object> pole = Vicon_object::get(1);

  // ros::Publisher publisher = node.advertise<std_msgs::Float64>(TOPIC,0);
  rosrt::Publisher<std_msgs::Float64> publisher(node.advertise<std_msgs::Float64>(topic_name_pole_angle, 1), 10, std_msgs::Float64());

  // ros publisher of the angle
  std_msgs::Float64Ptr msg = publisher.allocate();

  // setting loop rate
  ros::Rate loop_rate(looping_rate);

  // publishing at desired rate
  while ( ros::ok() ){
    msg->data = angle_estimation(base,pole);
    publisher.publish(msg);
    ros::spinOnce();
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