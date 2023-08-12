#ifndef __GAINS_COLLECTOR__
#define __GAINS_COLLECTOR__

#include <string>
#include <vector>
#include "rosrt/rosrt.h"
#include "ros/ros.h" 
#include "tools/PrintThis.hpp"

// Messages:
#include <std_msgs/UInt8.h>
#include <pole_balancing_apollo/Controller.h>

  class GainsCollector {
  public:
    GainsCollector(){};
    virtual ~GainsCollector(){}
    virtual bool get(std::vector<double> &gains) = 0;
  };

  class LQRGainsCollector : public GainsCollector {
  public:
    LQRGainsCollector(std::string topic,
                      std::string topic_controller_ack,
                      int controller_ack_id,
                      int Nx, int Nu);
    virtual ~LQRGainsCollector(){}
    bool get(std::vector<double> &gains);
    void parse_gains(std::vector<double> & gains);
  private:
    pole_balancing_apollo::ControllerConstPtr msg_controller;
    std::shared_ptr<ros::NodeHandle> nh;
    rosrt::Subscriber<pole_balancing_apollo::Controller> sub_controller;
    std::vector<std::vector<double> > K;
    rosrt::Publisher<std_msgs::UInt8> pub_ack;
    std_msgs::UInt8Ptr msg_ack;
    PrintThis verbosity;
    int Nx;
    int Nu;
    char buff[200];
  };

#endif // __GAINS_COLLECTOR__