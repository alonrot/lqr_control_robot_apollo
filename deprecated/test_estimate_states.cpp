#include "estimate_states.hpp"

#ifdef __XENO__
#include <native/task.h>
#include <sys/mman.h>
#endif

int run(int argc, char** argv)
{

	int i;
	// double measured_pole_angle;
	// std::shared_ptr<CartesianState> endeffector_state_measured;
	// endeffector_state_measured.reset(new CartesianState());
	double xdd_plane_filt = 0.0;
	std::vector<double> states(N_STATES);

	// Initialize task parameters:
	std::shared_ptr<ParametersInitialization> par;
	par.reset(new ParametersInitialization());

	// Initialize observe states:
	std::shared_ptr<EstimateStates> state_estimation;
	state_estimation.reset(new EstimateStatesPole(par->plane_angle,
																								par->time_step,
																								par->which_filter_pole,
																								par->which_filter_xdd,
																								par->integral_action,
																								par->endeff_offset));

	// Measurements:
	std::shared_ptr<Measurements> measurements;
	measurements.reset(new MeasurementsTest());
	measurements->endeffector_state_measured->x[0] = 0.01;

	// Random number generator:
  std::random_device rd;
  std::mt19937 rnd_generator(rd()); 
  std::normal_distribution<double> gaussian_noise(0.0,1.0);

	// Create node (needed for ros::ok())
	ros::init(argc,argv,"test_observe_states");
  struct rosrt::InitOptions options;
  options.pubmanager_thread_name = "test_observe_states";
  rosrt::init(options);
  ros::NodeHandle node;

  // Loop rate:
  ros::Rate loop_rate(par->task_servo_rate);

  while ( ros::ok() ){

  	// Artificially measure:
  	measurements->measured_pole_angle += 0.001;

  	// Update state estimation from measurements:
  	state_estimation->update_states(measurements);

  	// Get the states:
		state_estimation->get_states(states);
		xdd_plane_filt = state_estimation->xdd_plane_filtered;

		printf("states[PHI] 	= %f\n",states[PHI]);
		printf("states[PHID] 	= %f\n",states[PHID]);
		printf("states[S] 		= %f\n",states[S]);
		printf("states[SD] 		= %f\n",states[SD]);
		printf("states[INT_] 	= %f\n",states[INT_]);
		printf("\n");
		printf("xdd_plane_filt = %f\n",xdd_plane_filt);

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