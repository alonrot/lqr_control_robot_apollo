#ifndef SUBSCRIBER_POLE_ANGLE
#define SUBSCRIBER_POLE_ANGLE


#include "ros/ros.h"
#include "ros/param.h"
#include "rosrt/rosrt.h"
#include <std_msgs/Float64.h>
#include <string>

#ifdef __XENO__
#include <native/task.h>
#include <sys/mman.h>
#endif

// Class with virtual objects, that speak for some desired high-level functionalities.
// Regardless of the source of the pole angle (e.g., VICON or Kinect), these high level
// functions are always likely to be implemented.
class Subscriber_pole_angle {
public:
  virtual ~Subscriber_pole_angle(){}
  virtual double get()=0;
  virtual bool has_error()=0;
  virtual std::string get_error()=0;
};

// This class inherites the previos one, and implements the virtual functions.
// It reads the pole angle from a VICON system
class RosRT_Subscriber_pole_angle : public Subscriber_pole_angle {
public:
  RosRT_Subscriber_pole_angle(std::string topic_name);
  double get();
  bool has_error();
  std::string get_error();
private:
  std_msgs::Float64ConstPtr msg;
  ros::NodeHandle *handle;
  rosrt::Subscriber<std_msgs::Float64> *subscriber;
  bool error;
  std::string error_message;

  // We store the last measured angle to publish it in case we have no new data:
  double prev_data;
};

// This class inherites the previos one, and implements the virtual functions.
// It reads a constant value sent from an artificial ros topic, for testing purposes.
class Constant_pole_angle : public Subscriber_pole_angle {
public:
  Constant_pole_angle(double value);
  double get();
  bool has_error();
  std::string get_error();
private:
  double value;
};



#endif // SUBSCRIBER_POLE_ANGLE
