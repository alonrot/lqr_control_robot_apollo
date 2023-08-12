#include "pole_angle_measurement/subscriber_pole_angle.hpp"

// ------------------------------------- RosRT_Subscriber_pole_angle -------------------------------------

RosRT_Subscriber_pole_angle::RosRT_Subscriber_pole_angle(std::string topic_name):error(false),error_message(""){
  
  // Arguments for node:
  int argc = 1;
  char* argv[1];
  // Copy over string to char:
  std::string node_name("RosRT_Subscriber_pole_angle");
  argv[0] = new char[node_name.size() + 1];
  std::copy(node_name.begin(), node_name.end(), argv[0]);
  argv[0][node_name.size()] = '\0';

  ros::init(argc,argv,node_name);
  this->handle = new ros::NodeHandle();

  struct rosrt::InitOptions options;
  options.pubmanager_thread_name = node_name;
  
  rosrt::init(options);
  this->subscriber = new rosrt::Subscriber<std_msgs::Float64>(3, *handle, topic_name);
  
  this->prev_data = 0.0;
}

double RosRT_Subscriber_pole_angle::get()
{

  double current_angle;

  if (ros::ok()){

    this->msg = subscriber->poll();
    
    if (this->msg)
    {
      // We store the current data point in case in the next call to get() (from SL), there is no new data yet.
      current_angle = this->msg->data;
      this->prev_data = current_angle;
      return current_angle;
    }
    else
    {
      return this->prev_data;
    }
  }
  this->error_message = "ROS core stopped";
  this->error = true;
}

bool RosRT_Subscriber_pole_angle::has_error(){
  return this->error;
}

std::string RosRT_Subscriber_pole_angle::get_error(){
  return this->error_message;
}

// ------------------------------------- Constant_pole_angle (testing purposes) -------------------------------------

Constant_pole_angle::Constant_pole_angle(double value){
  this->value = value;
}

double Constant_pole_angle::get(){
  return this->value;
}

bool Constant_pole_angle::has_error(){
  return false;
}

std::string Constant_pole_angle::get_error(){
  return "no error";
}



