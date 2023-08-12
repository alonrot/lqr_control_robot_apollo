#include "lqr_gains/subscribe_to_gains.hpp"

// using namespace std;

SubscribeToGainsLQR::SubscribeToGainsLQR(char * NODE, char * TOPIC)
{

  this->NODE = NODE;
  this->TOPIC = TOPIC;

  int argc = 1; char name[] = "SL"; char* argv[1]; argv[0] = name;

  ros::init(argc,argv,this->NODE);
  this->handle = new ros::NodeHandle();

  struct rosrt::InitOptions options;
  options.pubmanager_thread_name = this->NODE;
  
  rosrt::init(options);
  this->subscriber = new rosrt::Subscriber<std_msgs::Float64MultiArray>(10, *(this->handle), this->TOPIC); 

  // this->msg.data.resize(5);

}

// SubscribeToGainsLQR::~SubscribeToGainsLQR()
// {

// }

bool SubscribeToGainsLQR::correctly_read(void)
{

  if (ros::ok()){

    this->msg = subscriber->poll();

    if (!this->msg){
      return false;
    }
    else{
      // std::cout << "Gains correctly read" << std::endl;
    }
    
  }
  else {
  	// std::cout << "ROS core stopped" << std::endl;
  	return false;
  }

  return true;
}

const double * SubscribeToGainsLQR::get_gains(void)
{

	const double * lqr_gains_prt = new double;

  lqr_gains_prt = &this->msg->data[0];

  return lqr_gains_prt;
}