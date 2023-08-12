#pragma once

#include <string>
#include <stdexcept>

#include "rosrt/rosrt.h"
#include "ros/ros.h" 
#include <std_msgs/Float64MultiArray.h>

namespace feedback_control {

  /* controllers --------------------------------------------------------------------------------------*/

  class FeedbackController {
  public:
    FeedbackController(std::vector<double> default_F);
    FeedbackController(void);
    void set_saturation(double saturation);
    void reset_gains();
    void set_gains(std::vector<double> F);
    int get_dimension();
    double get_control(std::vector<double> input);
    virtual void reset()=0;
  protected:
    virtual double compute_control(std::vector<double> input)=0;
    std::vector<double> current_F;
    std::vector<double> default_F;
    double saturation;
  };


  class PIDController : public FeedbackController {

  public:
    PIDController();

  };


  class LQRController : public FeedbackController {
  public:
    LQRController(std::vector<double> default_F);
    void reset();
  protected:
    double compute_control(std::vector<double> input);
  };

  
  // expected input: time_step, angle, velocity, cart_plane_x, cart_plan_xd
  // expected 5 gains : for angle, velocity, cart_plan_x, cart_plan_xd and integral term
  // adviced usage: use compute_default_control functions defined in this file
  class PoleBalancingDefaultController : public LQRController {
  public:
    PoleBalancingDefaultController(std::vector<double> default_F);
    void reset();
  protected:
    double compute_control(std::vector<double> input); // override LQRController function
    double integral;
  };

  // convenience function to make sure PoleBalancingDefaultController is used correctly and in a reader friendly way
  double compute_default_control(std::shared_ptr<PoleBalancingDefaultController> controller,
				 double time_step, double angle, double velocity, double cart_plan_x, double cart_plan_xd);

  /* Gains readers ---------------------------------------------------------------------------------------------*/

  class GainsReader {
  public:
    virtual bool get(std::vector<double> &gains) = 0;
  };

  class ROS_GainsReader : public GainsReader {
  public:
    ROS_GainsReader(std::string topic);
    bool get(std::vector<double> &gains);
  private:
    std_msgs::Float64MultiArrayConstPtr msg;
    ros::NodeHandle *handle;
    rosrt::Subscriber<std_msgs::Float64MultiArray> *subscriber;
  };



}

