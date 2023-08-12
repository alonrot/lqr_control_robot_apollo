/*!=============================================================================
  ==============================================================================

  \file    parameters_initialization.hpp

  \author  Alonso Marco
  \date    Jan. 2017

  ==============================================================================
  \remarks
  
  ============================================================================*/

#ifndef INCLUDE_PARAMETERS_INITIALIZATION_H
#define INCLUDE_PARAMETERS_INITIALIZATION_H

#include "tools/tools.hpp"
#include "pole_balancing_apollo/pole.h"
#include "lqr_gains/subscribe_to_gains.hpp"

class ParametersInitialization{

public:
	ParametersInitialization();
	~ParametersInitialization();
	void apply_user_changes(void);
	int sim_flag;
	int simulate_noise; 	// Simulate noise
	double initial_angle; // Pole angle offset introduced by the user in simulation
 	int VISION_LATENCY; //meas	// Number of time steps needed for reading a new angle. This number simulates the delay of the image processing. 
	int ENDEFF_LATENCY; //meas  // Number of time steps needed for reading a new angle. This number simulates the delay of the image processing. 
	double total_et; // Total experiment time
	double total_tt; // Total task time
	int    which_filter_pole;
	int    which_filter_xdd;
	// int    which_filter_xd;
	// int    which_filter_x;
	int    which_vision_Ts; // Vision system's sampling time selector
	int    vision_Ts; //meas
	double    u_sat; // Control saturation
	int    change_safety;
	double    box_length_thres;
	double box_width_thres;
	double box_height_thres;
	int    integral_action;
	int    only_recording;
	int using_head;
	double HR_offset_user;
	int     change_safe_set;  
	int  transient_time_ms; 
	int   back_to_safety_after_experiments; 
	int change_cart_gains; 
	int     control_change;   
	int     record_data;      
	double   xdd_safe;
	double  start_time;
	double   x_safe[N_STATES];
	double   KP_pose[N_CART];
	double   KI_pose[N_CART];
	double   KD_pose[N_CART];
	double   F[N_CONTROLLER];
	double   F_safe[N_CONTROLLER];
	double   F_search[N_CONTROLLER];
	std::shared_ptr<SubscribeToGains> lqr_search;
	double   ufb_pose_KP[apollo_par::N_DOFS];
	double   ufb_pose_KD[apollo_par::N_DOFS];
	double   ufb_pose_KI[apollo_par::N_DOFS];
	double 	 int_state_pose[apollo_par::N_DOFS];
	double 	endeff_offset[N_CART];
	int 	firsttime_;
	int 	save_in_next;
	int 	cstatus_pose[apollo_par::n_endeffs+1];
	int 	task_servo_rate;
	int 	win_length;
	double** 	integrator_window;
	int 	ind_win;
	double 	sign_conv;
	double  time_step;
	double 	start_time_exp;
	double 	task_time;
	double 	elapsed_time_run;
	int sw2safe;
	int reset_;
	int hold_;
	int exp_running;
	int unstability_timer_ms;
	int recovering_from_unstability;
	double plane_angle;
	CartesianState cart_base; // End-effector position, velocity and acceleration in the robot base's frame:
	CartesianState cart_plane; // End-effector position, velocity and acceleration in the pole plane's frame:
	CartesianState cart_des_plane; // Desired end-effector position, velocity and acceleration in the pole plane's frame:
	CartesianState cart_tar_plane; // Target end-effector position, velocity and acceleration in the pole plane's frame:
	CartesianState cart_tar_world_prev; // Target end-effector position, velocity and acceleration in the pole plane's frame:
	double   theta_pan;
	double   theta_tilt;
	double 	 cart_xdd_plane_filt;
	double 	 initial_angle_offset;
	double 	 g_n;
	PoleParameters	pole_para; //meas
	PoleGraphics		pole_grap;
	Pose 						pole_pose;
	double current_angle_measured;
	double angle_visualization;
	double 	int_state;
	double 	sigma_n; //meas
	KineticState vis_states_filtered;
	double u_to_be_sent;	
	double u_send;
	int ES_iter_counter;
	double joint_error[apollo_par::R_WAA];
	double u_i[apollo_par::R_WAA];
	double u_pd[apollo_par::R_WAA];
	double controller_gain_th[apollo_par::N_DOFS];
	double controller_gain_thd[apollo_par::N_DOFS];
	double controller_gain_int[apollo_par::N_DOFS];

};


#endif /* INCLUDE_PARAMETERS_INITIALIZATION_H */