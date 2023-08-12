#include <iostream>
#include "parameters_initialization.hpp"

// using namespace std;

int run(int argc, char** argv)
{

	int i;

	// Parameters initialization:
	ParametersInitialization * par = new ParametersInitialization();

	// Create node (needed for ros::ok())
	ros::init(argc,argv,"test_parameters_initialization");
  struct rosrt::InitOptions options;
  options.pubmanager_thread_name = "test_parameters_initialization";
  rosrt::init(options);
  ros::NodeHandle node;

  // Loop rate:
  ros::Rate loop_rate(par->task_servo_rate);

  while ( ros::ok() )
  {

		printf("F[0] = %f\n",par->F[0]);

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