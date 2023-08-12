#include "estimate_states.hpp"

#ifdef __XENO__
#include <native/task.h>
#include <sys/mman.h>
#endif

int run(int argc, char** argv)
{
	
	int i;
  double * control_input;
  std::shared_ptr<CartesianState> endeffector_state_measured;
  std::vector<double> states(N_STATES);

	// Initialize task parameters:
	std::shared_ptr<ParametersInitialization> par;
	par.reset(new ParametersInitialization());
	std::shared_ptr<CartesianState> endeffector_state_raw;
	endeffector_state_raw.reset(new CartesianState());
	control_input = new double;
	*control_input = 0.001;

	// Modify some options:
	par->simulate_noise = FALSE;
	par->sim_flag = TRUE;
	par->VISION_LATENCY = 0;

	// Measurements object:
	std::shared_ptr<Measurements> measurements;

	// Read in simulation:
	if( par->sim_flag )
		measurements.reset(new SnatchPoleSimulation(par->sigma_n,par->vision_Ts,par->VISION_LATENCY,
																								par->ENDEFF_LATENCY,par->time_step,par->g_n,par->pole_para,
																								endeffector_state_raw,
																								control_input));
	else
		measurements.reset(new SnatchPoleVicon(par->sign_conv,par->initial_angle_offset,
																						endeffector_state_raw));

	// Initialize observe states:
	std::shared_ptr<EstimateStates> state_estimation;
	state_estimation.reset(new EstimateStatesPole(par->plane_angle,
                                                par->time_step,
                                                par->which_filter_pole,
                                                par->which_filter_xdd,
                                                par->integral_action,
                                                par->endeff_offset));

	// Random number generator:
  std::random_device rd;
  std::mt19937 rnd_generator(rd()); 
  std::normal_distribution<double> gaussian_noise(0.0,1.0);

	// Create node (needed for ros::ok())
	ros::init(argc,argv,"test_measure_and_estimate");
  struct rosrt::InitOptions options;
  options.pubmanager_thread_name = "test_measure_and_estimate";
  rosrt::init(options);
  ros::NodeHandle node;

  // Loop rate:
  ros::Rate loop_rate(par->task_servo_rate);

  while ( ros::ok() ){

  	// Compute control_input from controller:
  	*control_input = 0.001;

  	// Read endeffector state from the robot:
	  for(i=0;i<N_CART;++i){
	  	endeffector_state_raw->x[i]		= gaussian_noise(rnd_generator);
	  	endeffector_state_raw->xd[i] 	= gaussian_noise(rnd_generator);
	  	endeffector_state_raw->xdd[i]	= gaussian_noise(rnd_generator);
	  }

	  // Get sensor readings:
	  measurements->update_measurements();

	  // Print meaningful public variables:
	  printf("measured_pole_angle = %f\n",measurements->measured_pole_angle);
	  printf("visualization_pole_angle = %f\n",measurements->visualization_pole_angle);
	  endeffector_state_measured = measurements->endeffector_state_measured;
	  printf("endeffector_state_measured.x[0] = %f\n",endeffector_state_measured->x[0]);

  	// Get the states:
  	state_estimation->update_states(measurements);
		state_estimation->get_states(states);

		printf("states[PHI] 	= %f\n",states[PHI]);
		printf("states[PHID] 	= %f\n",states[PHID]);
		printf("states[S] 		= %f\n",states[S]);
		printf("states[SD] 		= %f\n",states[SD]);
		printf("states[INT_] 	= %f\n",states[INT_]);
		printf("\n");

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