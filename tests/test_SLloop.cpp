#include "estimate_states.hpp"
#include "pole_balancing_apollo/feedback_control.hpp"

#ifdef __XENO__
#include <native/task.h>
#include <sys/mman.h>
#endif

int run(int argc, char** argv)
{
	
	// User-defined variables:
	int i;
	std::shared_ptr<CartesianState> endeffector_state_raw = NULL;
	double * control_input = NULL;
	double xdd_plane_filtered;

	// Declare objects:
	std::shared_ptr<ParametersInitialization> par;
	std::shared_ptr<Measurements> measurements;
	std::shared_ptr<EstimateStates> state_estimation;
	std::shared_ptr<feedback_control::FeedbackController> controller;

	// Initialize:
	par.reset(new ParametersInitialization());
		// Modify some parameters:
		par->simulate_noise = TRUE;
		par->sim_flag = FALSE;
		par->VISION_LATENCY = 0;

	// Declare:
	endeffector_state_raw.reset(new CartesianState());
	control_input = new double;
	*control_input = 0.001;
	std::vector<double> F_safe(N_STATES);
	std::vector<double> F_search(N_STATES);
	std::vector<double> states(N_STATES);

	// Initialization:
	for(i=0;i<N_STATES;++i){
		F_safe[i] = par->F[i];
		F_search[i] = par->F[i]+2;
	}

	// Simulation/real robot:
	if( par->sim_flag )
		measurements.reset(new SnatchPoleSimulation(par->sigma_n,par->vision_Ts,par->VISION_LATENCY,
																								par->ENDEFF_LATENCY,par->time_step,par->g_n,par->pole_para,
																								endeffector_state_raw,
																								control_input));
	else
		measurements.reset(new SnatchPoleVicon(par->sign_conv,par->initial_angle_offset,
																						endeffector_state_raw));

	// Initialize observe states:
	state_estimation.reset(new EstimateStatesPole(par->plane_angle,
                                                par->time_step,
                                                par->which_filter_pole,
                                                par->which_filter_xdd,
                                                par->integral_action,
                                                par->endeff_offset));

	// Initialize controller:
	controller.reset(new feedback_control::LQRController(F_safe));
	controller->set_gains(F_search);
	controller->set_saturation(par->u_sat);

	// Test controller:
	for(i=0;i<5;++i)
		printf("F_search[%i] = %f\n",i,F_search[i]);

	// Test controller:
	for(i=0;i<5;++i)
		printf("F_safe[%i] = %f\n",i,F_safe[i]);

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

  printf("Running loop...\n");

  while ( ros::ok() ){

  	// Read endeffector state from the robot:
	  for(i=0;i<N_CART;++i){
	  	endeffector_state_raw->x[i]		= gaussian_noise(rnd_generator);
	  	endeffector_state_raw->xd[i] 	= gaussian_noise(rnd_generator);
	  	endeffector_state_raw->xdd[i]	= gaussian_noise(rnd_generator);
	  }

	  // Get sensor readings:
	  measurements->update_measurements();

  	// Get the states:
  	state_estimation->update_states(measurements);
		state_estimation->get_states(states);

		for(i=0;i<N_STATES;++i)
			printf("states[%i] = %f\n",i,states[i]);

		// Compute control input:
		*control_input = controller->get_control(states);
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