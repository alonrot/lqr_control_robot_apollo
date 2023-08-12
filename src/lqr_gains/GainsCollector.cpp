#include "lqr_gains/GainsCollector.hpp"

LQRGainsCollector::LQRGainsCollector( std::string topic_controller,
                                      std::string topic_controller_ack,
                                      int controller_ack_id,
                                      int Nx, int Nu){

  int argc = 1;
  char name[] = "LQRGainsCollector";
  char* argv[1];
  argv[0] = name;
  ros::init(argc,argv,name);
  struct rosrt::InitOptions options;
  options.pubmanager_thread_name = name; 
  this->nh = std::make_shared<ros::NodeHandle>();
  rosrt::init(options);

  // Subscribe to gains:
  this->sub_controller.initialize(100);
  this->sub_controller.subscribe(*this->nh,topic_controller);

  // Acknowledgment:
  this->pub_ack.initialize(this->nh->advertise<std_msgs::UInt8>(topic_controller_ack, 0), 10, std_msgs::UInt8());
  this->msg_ack = this->pub_ack.allocate();
  this->msg_ack->data = controller_ack_id;

  // Controller dimensions:
  this->Nx = Nx;
  this->Nu = Nu;

  // Initialize:
  this->K.resize(this->Nu);
  for(size_t i=0;i<this->Nu;++i)
    this->K[i].resize(this->Nx);

}

bool
LQRGainsCollector::get(std::vector<double> &gains){

  if (ros::ok()){

    // Poll:
    this->msg_controller = sub_controller.poll();

    // If message found, parse gains:
    if (this->msg_controller) { 

      // Get the gains:
      this->parse_gains(gains);

      // Send acknowledgment:
      this->pub_ack.publish(this->msg_ack);
      
      return true;
    }
    return false;
  }
}

void
LQRGainsCollector::parse_gains(std::vector<double> & gains){

  if(this->msg_controller->full_verbosity){
    this->verbosity.print("@LQRGainsCollector::parse_gains - Request for a new experiment...\n");
    this->verbosity.print("Saving internally the controller\n");
  }

  // Fill in gain:
  for(size_t i=0;i<this->Nu;++i){
    for(size_t j=0;j<this->Nx;++j){
      this->K[i][j] = this->msg_controller->controller_gain[ this->Nx * i + j ];
    }
  }

  // Parse gains:
  if(this->Nu == 1){
    for(size_t j=0;j<this->Nx;++j){
      gains[j] = this->K[0][j];
    }
  }
  else{
    sprintf(this->buff,"ERROR: @LQRGainsCollector::parse_gains - No gain collector defined for Nu != 1, Nu = %i",(int)this->Nu);
    this->verbosity.print(this->buff);
  }

  return;
}