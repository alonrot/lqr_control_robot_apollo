#include "lqr_gains/subscribe_to_gains.hpp"

#ifdef __XENO__
  #include <native/task.h>
  #include <sys/mman.h>
#endif

int main(int argc, char** argv)
{
  #ifdef __XENO__
    mlockall(MCL_CURRENT | MCL_FUTURE);
  #endif

  // Name of the ROS node:
  char NODE[50];

  // Name of the ROS topics:
  char SAFE_GAINS_TOPIC[50];
  char SEARCH_GAINS_TOPIC[50];

  // Name of the ROS node:
  sprintf(NODE,"%s","LQR_gains_subscriber");

  // Set the topic names:
  sprintf(SAFE_GAINS_TOPIC,"%s","/LQRgains/safe");
  sprintf(SEARCH_GAINS_TOPIC,"%s","/LQRgains/search");

  // LQR gains:
  const double * lqr_safe_gains   = new double;
  const double * lqr_search_gains = new double;

  // const string 

  // Object node for collecting LQR gains:
  SubscribeToGains * collect_safe    = new SubscribeToGainsLQR(NODE,SAFE_GAINS_TOPIC);
  SubscribeToGains * collect_search  = new SubscribeToGainsLQR(NODE,SEARCH_GAINS_TOPIC);

  ros::Rate loop_rate(RATE);

  while ( ros::ok() )
  {

    if( collect_safe->correctly_read() ){
      lqr_safe_gains = collect_safe->get_gains();
      std::cout << "Sanity check: lqr_safe_gains[0] = " << lqr_safe_gains[0] << std::endl;
    }

    if( collect_search->correctly_read() ){
      lqr_search_gains = collect_search->get_gains();
      std::cout << "Sanity check: lqr_search_gains[0] = " << lqr_search_gains[0] << std::endl;
    }

    // ros::spinOnce();
    loop_rate.sleep();

  }

}