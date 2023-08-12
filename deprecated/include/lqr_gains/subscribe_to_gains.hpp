#ifndef SUBSCRIBE_TO_GAINS_H
#define SUBSCRIBE_TO_GAINS_H


#include "ros/ros.h"
#include "ros/param.h"
#include "rosrt/rosrt.h"
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <string>

#include <iostream>
#include <fstream>
#include <string>

#define RATE 1000

class SubscribeToGains {
  public:
    virtual const double * get_gains()=0;
    virtual bool    correctly_read()=0;
};

// This class inherites the previos one, and implements the virtual functions.
// It reads a constant value sent from an artificial ros topic, for testing purposes.
class SubscribeToGainsLQR : public SubscribeToGains {
  public:
    SubscribeToGainsLQR(char * NODE, char * TOPIC);
    // ~SubscribeToGainsLQR();
    const double *  get_gains(void);
    bool correctly_read(void);
  private:
    char * TOPIC;
    char * NODE;

    std_msgs::Float64MultiArrayConstPtr msg;
    ros::NodeHandle * handle;
    rosrt::Subscriber<std_msgs::Float64MultiArray> * subscriber;

};

#endif /* SUBSCRIBE_TO_GAINS_H */
