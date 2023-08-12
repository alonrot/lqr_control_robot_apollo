#include "lqr_gains/gather_LQR_gains_to_publish.hpp"

GatherLQRGainsToPublish::GatherLQRGainsToPublish(ReadLQRGains * safe, ReadLQRGains * search, std::string NODE_NAME)
{
	this->safe 		= safe;
	this->search 	= search;
  this->NODE_NAME.assign(NODE_NAME);
}

GatherLQRGainsToPublish::~GatherLQRGainsToPublish()
{}

bool GatherLQRGainsToPublish::run(void)
{

  // Get node name (has to be a std::string):
  std::string node_name(this->NODE_NAME);

  // Arguments for node:
  int argc = 1;
  char* argv[1];
  // Copy over string to char:
  argv[0] = new char[node_name.size() + 1];
  std::copy(node_name.begin(), node_name.end(), argv[0]);
  argv[0][node_name.size()] = '\0';

  // Initialize node:
  ros::init(argc,argv,node_name);

  struct rosrt::InitOptions options;
  options.pubmanager_thread_name = node_name;

  rosrt::init(options);

  ros::NodeHandle node;

  rosrt::Publisher<std_msgs::Float64MultiArray> pub_safe(node.advertise<std_msgs::Float64MultiArray>(safe->TOPIC, 1), 10, std_msgs::Float64MultiArray());
  rosrt::Publisher<std_msgs::Float64MultiArray> pub_search(node.advertise<std_msgs::Float64MultiArray>(search->TOPIC, 1), 10, std_msgs::Float64MultiArray());

  // ros publisher of the LQR gains
  std_msgs::Float64MultiArrayPtr msg_safe 		= pub_safe.allocate();
  std_msgs::Float64MultiArrayPtr msg_search 	= pub_search.allocate();

  // Resize the message:
  msg_safe->data.resize(safe->n_gains);
  msg_search->data.resize(search->n_gains);

  // Layout:
  msg_safe->layout.dim.resize(1);
  msg_safe->layout.dim[0].label = this->safe->fname;
  msg_safe->layout.dim[0].size = 5;

  msg_search->layout.dim.resize(1);
  msg_search->layout.dim[0].label = this->search->fname;
  msg_search->layout.dim[0].size = 5;

  // setting loop rate
  ros::Rate loop_rate(RATE);

  printf("Publishing LQR gains...\n");
  // publishing at desired rate
  while ( ros::ok() )
  {

    // Check whether the file is ready yet or not:
    if ( safe->get_gains() )
    {
      // Fill the data field:
      for (int i=0;i<safe->n_gains;++i)
        // msg_safe->data.push_back(safe->set_of_gains[i]);
        msg_safe->data[i] = safe->set_of_gains[i];

      // std::cout << "msg_safe->data[0] = " << msg_safe->data[0] << std::endl;

      // Publish data
      pub_safe.publish(msg_safe);
    }
    else
      printf("File %s not ready yet... (being written by another process, or erased)\n",this->safe->fname);

    // Check whether the file is ready yet or not:
    if ( search->get_gains() )
    {
      // Fill the data field:
      for (int i=0;i<search->n_gains;++i)
        // msg_search->data.push_back(search->set_of_gains[i]);
        msg_search->data[i] = search->set_of_gains[i];

      // Publish data
      pub_search.publish(msg_search);
    }
    else
      printf("File %s not ready yet... (being written by another process, or erased)\n",this->search->fname);

    // ros::spinOnce();
    loop_rate.sleep();

  }
  
  return true;
}