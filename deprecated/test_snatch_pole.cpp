#include "snatch_pole.hpp"

// using namespace std;

int run(int argc, char** argv)
{
	
	int i;
  double u_send;
  CartesianState endeffector_state_raw, endeffector_state_measured;

	// SL Options:
	Options_SL * opts_SL = new Options_SL;
	opts_SL->task_servo_rate = 1000;

	// Initialize task parameters:
	std::shared_ptr<ParametersInitialization> par;
	par.reset(new ParametersInitialization(opts_SL));

	// Modify some options:
	par->simulate_noise = FALSE;
	par->sim_flag = TRUE;
	par->VISION_LATENCY = 0;

	// Read:
	std::shared_ptr<SnatchPole> snatch_pole_v;
	snatch_pole_v.reset(new SnatchPoleVicon(par));

	// Random number generator:
  std::random_device rd;
  std::mt19937 rnd_generator(rd()); 
  std::normal_distribution<double> gaussian_noise(0.0,1.0);

	// Create node (needed for ros::ok())
	ros::init(argc,argv,"test_snatch_pole");
  struct rosrt::InitOptions options;
  options.pubmanager_thread_name = "test_snatch_pole";
  rosrt::init(options);
  ros::NodeHandle node;

  // Loop rate:
  ros::Rate loop_rate(par->task_servo_rate);

  while ( ros::ok() ){

  	// // Compute u_send from controller:
  	u_send = 0.001;

  	// Read endeffector state from the robot:
	  for(i=0;i<N_CART;++i){
	  	endeffector_state_raw.x[i] 		= gaussian_noise(rnd_generator);
	  	endeffector_state_raw.xd[i] 	= gaussian_noise(rnd_generator);
	  	endeffector_state_raw.xdd[i] 	= gaussian_noise(rnd_generator);
	  }

	  // Get sensor readings:
	  snatch_pole_v->update(endeffector_state_raw,u_send);

	  // Print meaningful public variables:
	  printf("measured_pole_angle = %f\n",snatch_pole_v->get_measured_pole_angle());
	  printf("visualization_pole_angle = %f\n",snatch_pole_v->get_visualization_pole_angle());
	  endeffector_state_measured = snatch_pole_v->get_measured_endeffector();
	  printf("endeffector_state_measured.x[0] = %f\n",endeffector_state_measured.x[0]);

    // ros::spinOnce();
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