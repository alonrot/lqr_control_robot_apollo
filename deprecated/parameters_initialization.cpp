/*!=============================================================================
  ==============================================================================

  \file    parameters_initialization.cpp

  \author  Alonso Marco
  \date    Jan. 2017

  ==============================================================================
  \remarks
  
  ============================================================================*/

#include "parameters_initialization.hpp"

ParametersInitialization::ParametersInitialization()
{
  int i,j;

  printf(" ** Initializing parameters...\n");

  /* Initial values */
  // Automatic switch when working with the real robot:
  #ifdef __XENO__
    this->sim_flag = FALSE;
    this->total_et = 20;
    this->using_head = TRUE;
  #else
    this->sim_flag = TRUE;
    this->total_et = 5;
    this->using_head = FALSE;
  #endif

    this->simulate_noise    = TRUE;
    this->initial_angle     = 0.0;
    this->VISION_LATENCY    = 30;
    this->ENDEFF_LATENCY    = 0;
    this->total_tt          = 36000;
    this->which_filter_pole = 3;
    this->which_filter_xdd  = 5;
    // this->which_filter_xd   = 1;
    // this->which_filter_x    = 2;
    this->which_vision_Ts   = 1;
    this->vision_Ts         = 5;
    this->u_sat             = 3;
    this->change_safety     = FALSE;
    // Safety thresholds:
    this->box_length_thres  = 0.30;
    this->box_width_thres   = 0.10;
    this->box_height_thres  = 0.10;
    this->integral_action   = TRUE;
    this->only_recording    = FALSE;
    this->HR_offset_user    = 30;
    this->change_safe_set   = FALSE;
    this->transient_time_ms = 2000;
    this->back_to_safety_after_experiments = FALSE;
    this->change_cart_gains = FALSE;
    this->control_change    = FALSE;
    this->record_data       = TRUE;
    this->xdd_safe          = 5;
    Tools::vec_zero(this->x_safe,N_STATES);
    this->x_safe[PHI] = 2.5*M_PI/180;
    this->x_safe[S]   = this->box_length_thres - 0.05;
    this->start_time        = 0;
    double KP_val, KD_val, KI_val;
    KP_val  = 2000.0; KD_val  = 10.0; KI_val  = 100.0;  
    this->KP_pose[_x_] = KP_val; this->KP_pose[_y_] = KP_val; this->KP_pose[_z_] = KP_val;
    this->KI_pose[_x_] = KI_val; this->KI_pose[_y_] = KI_val; this->KI_pose[_z_] = KI_val;
    this->KD_pose[_x_] = KD_val; this->KD_pose[_y_] = KD_val; this->KD_pose[_z_] = KD_val;
    Tools::vec_zero(cart_tar_world_prev.x,N_CART);
    Tools::vec_zero(cart_tar_world_prev.xd,N_CART);
    Tools::vec_zero(cart_tar_world_prev.xdd,N_CART);
    this->firsttime_ = TRUE;
    this->save_in_next = FALSE;
    // Flag set to true when a new experiment is running:
    this->exp_running = FALSE; 

  // Cartesian feedback control variables:
    // Feedback torques:
    Tools::vec_zero(this->ufb_pose_KP,apollo_par::N_DOFS);
    Tools::vec_zero(this->ufb_pose_KD,apollo_par::N_DOFS);
    Tools::vec_zero(this->ufb_pose_KI,apollo_par::N_DOFS);
    Tools::vec_zero(this->int_state_pose,apollo_par::N_DOFS);

  // End-effector offset:
  Tools::vec_zero(this->endeff_offset,N_CART);

    // Number of cartesian DOFs:
    Tools::vec_zero(this->cstatus_pose,apollo_par::n_endeffs+1);
    // DOFs involved in the cartesian feedback control loop: [x,y,z] from right hand
    this->cstatus_pose[1] = 1; this->cstatus_pose[2] = 1; this->cstatus_pose[3] = 1;

  // We integrate over a window of time:
  int win_time                = 5; //Seconds
  this->task_servo_rate       = 1000;
  this->win_length            = win_time*(this->task_servo_rate);
  // End-effector position defined from the stang point, in the plane of movement frame's coordiantes:
  // for(i=0; i<N_CART ; ++i)
   // for(j=0; j<N_CART ; ++j)
     // cart_x_plane[i] += opts->rot_mat_[i][j] * cart_x_local[j];
  // this->integrator_window     = my_matrix(1,apollo_par::R_WAA,1,win_length);
  this->integrator_window = new double *[apollo_par::R_WAA];
  for (i = 0; i < apollo_par::R_WAA; i++)
      this->integrator_window[i] = new double[win_length];

  for (i = 0; i < apollo_par::R_WAA; i++)
    for (j = 0; j < win_length; j++)
      this->integrator_window[i][j] = 0.0;

  this->ind_win = 0;

  // Sign convention: when working on real-time, with the real robot, the sign of the angle acquired by
  // the vision system should be coherent with the position of the kinect: if the kinect is placed
  // in front of Apollo, then the sign must be (-). If the kinect is placed on its head, it must be (+).
  // Additionally, if working with VICON, we should pay attention to the axis convention. By now must be (+).
  this->sign_conv = +1;

  // Setting the time step:
  this->time_step = 1./(double)this->task_servo_rate;

  // Timers:
  // start_time          = 0.0;
  this->start_time_exp      = 0.0;
  this->task_time           = 0.0;
  this->elapsed_time_run    = 0.0;

    this->sw2safe = 0;
    this->reset_  = 0;
    this->hold_   = 0;

  // Unstability timer:
  this->unstability_timer_ms = 0;

  // Handling unstability:
  this->recovering_from_unstability = FALSE;

  // Angle of the plane of movement:
  // plane_angle = -HR_offset;
  this->plane_angle = -HR_offset_user*M_PI/180;

  // End-effector position, velocity and acceleration in the robot base's frame:
  Tools::vec_zero(this->cart_base.x,N_CART);
  Tools::vec_zero(this->cart_base.xd,N_CART);
  Tools::vec_zero(this->cart_base.xdd,N_CART);
  // cart_base = (SL_Cstate *) my_calloc(1,sizeof(SL_Cstate),MY_STOP);

  // End-effector position, velocity and acceleration in the pole plane's frame:
  Tools::vec_zero(this->cart_plane.x,N_CART);
  Tools::vec_zero(this->cart_plane.xd,N_CART);
  Tools::vec_zero(this->cart_plane.xdd,N_CART);
  // cart_plane = (SL_Cstate *) my_calloc(1,sizeof(SL_Cstate),MY_STOP);

  // Desired end-effector position, velocity and acceleration in the pole plane's frame:
  Tools::vec_zero(this->cart_des_plane.x,N_CART);
  Tools::vec_zero(this->cart_des_plane.xd,N_CART);
  Tools::vec_zero(this->cart_des_plane.xdd,N_CART);
  // cart_des_plane = (SL_Cstate *) my_calloc(1,sizeof(SL_Cstate),MY_STOP);

  // Target end-effector position, velocity and acceleration in the pole plane's frame:
  Tools::vec_zero(this->cart_tar_plane.x,N_CART);
  Tools::vec_zero(this->cart_tar_plane.xd,N_CART);
  Tools::vec_zero(this->cart_tar_plane.xdd,N_CART);
  Tools::vec_zero(this->cart_tar_world_prev.x,N_CART);
  Tools::vec_zero(this->cart_tar_world_prev.xd,N_CART);
  Tools::vec_zero(this->cart_tar_world_prev.xdd,N_CART);
  // cart_tar_plane = (SL_Cstate *) my_calloc(1,sizeof(SL_Cstate),MY_STOP);

  // Head angles:
  this->theta_pan  = 0.0;
  this->theta_tilt = 0.0;
  this->cart_xdd_plane_filt = 0.0;

  // Initial angles:
  this->initial_angle_offset  = 0.0;

  // System model variables and parameters:
  // Pole physical parameters (without the middle big ball, long version):
  this->g_n = 9.81;

  // Pole physical parameters:
  this->pole_para.length = 1.067;
  this->pole_para.mass = 0.2903;
  this->pole_para.center_of_mass = 0.636666256657899;
  this->pole_para.inertia =  0.117671340663077;
  this->pole_para.damping = 0.011603721783265;

  // Pole points needed to draw it:
  this->pole_grap.handle_height = 0.065;
  this->pole_grap.dist_bottom_ball = 0.525;

  // Integral action:
  this->int_state = 0.0;

  // Measurement noise std:
    // sigma_n = 0.0078; // Holger's noise (kinect)
    // sigma_n = 0.013111; // Maximum noise with the kinect placed far away 
    // sigma_n = 0.0095; // 30Hz, Distance Kinect-pole: 1m
    this->sigma_n = 0.000051985; // Vicon
    
  this->vis_states_filtered.th  = 0;
  this->vis_states_filtered.thd = 0;
  
  // Control inputs initialization:
  // u_del   = 0;
  this->u_to_be_sent  = 0;
  this->u_send  = 0;

  Tools::vec_zero(this->F_safe,N_CONTROLLER);
  Tools::vec_zero(this->F_search,N_CONTROLLER);
  Tools::vec_zero(this->F,N_CONTROLLER);

  /* Read initial LQR gains */
    char NODE[50], SEARCH_GAINS_TOPIC[50];
    sprintf(NODE,"%s","LQR_gains_subscriber");
    sprintf(SEARCH_GAINS_TOPIC,"%s","/LQRgains/search");
    lqr_search.reset(new SubscribeToGainsLQR(NODE,SEARCH_GAINS_TOPIC));

    // Gains reader:
    const double * read_search_gains = new double;

    // Loop constraints:
    bool  found     = false;
    int   count_max = 1000;
    int   count     = 0;

    // Loop at task_servo_rate:
    ros::Rate loop_rate(this->task_servo_rate);
    if(!ros::ok())
      printf(" ** For some reason ros::ok() = false... \n");
    while ( ros::ok() )
    {
      ++count;
      if( lqr_search->correctly_read() ){
        found = true;
        read_search_gains = lqr_search->get_gains();
        printf(" ** LQR Gains successfully read!...\n");
        break;
      }

      if( count == count_max){
        count = 0;
        printf(" ** Waiting for LQR gains to be read from ROS topic...\n");
      }

      loop_rate.sleep();
    }

    // Copy the new set of search gains:
    for(int i=0;i<N_CONTROLLER;++i)
      this->F_search[i] = read_search_gains[i];

    // Copy over into the main F gain:
    for(int i=0;i<N_CONTROLLER;++i)
      this->F[i] = this->F_search[i];


  /* Keep track of the PID control contributions */
    // Zero the vectors:
    Tools::vec_zero(this->joint_error,apollo_par::R_WAA);
    Tools::vec_zero(this->u_i,apollo_par::R_WAA);
    Tools::vec_zero(this->u_pd,apollo_par::R_WAA);

    // Controller gains:
    for(int i=0;i<apollo_par::N_DOFS;++i){
      controller_gain_th[i] = 0.0;
      controller_gain_thd[i] = 0.0;
      controller_gain_int[i] = 0.0;
    }

    // Read the control gains and max controls: TO BE DONE INSIDE SL
    // if (!read_gains("Gains.cf",controller_gain_th, controller_gain_thd, controller_gain_int))
      // return FALSE;

  // Entropy Search iteration counter:
  this->ES_iter_counter = -1;


  /**************************************************** END ****************************************************/

    // Feedback gains:
    // KP_pose     = my_vector(1,3);
    // KD_pose     = my_vector(1,3);
    // KI_pose     = my_vector(1,3);
    // double KP_val  = 1000.0;
    // double KI_val  = 1000.0;
    
    // for(i=0;i<N_CART;++i){
    //   cart_tar_world_prev.x[i] = 0.0; // pos_prev_x_SL pos_prev_y_SL
    //   cart_tar_world_prev.xd[i] = 0.0; // vel_prev_x_SL vel_prev_y_SL
    //   cart_tar_world_prev.xdd[i] = 0.0; // acc_prev_x_SL acc_prev_y_SL
    // }

  // // Trapezoidal integration variables:
  // pos_prev_x_SL = 0.0;
  // pos_prev_y_SL = 0.0;
  // vel_prev_x_SL = 0.0;
  // vel_prev_y_SL = 0.0;
  // acc_prev_x_SL = 0.0;
  // acc_prev_y_SL = 0.0;

  // // Setting to zero all the initial accelerations and velocities:
  // set_zero_dynamics();

  // /* zero the filters */
  // for (i=1; i<=n_dofs; ++i) 
  //   for (j=0; j<=FILTER_ORDER; ++j)
  //     fthdd[i].raw[j] = fthdd[i].filt[j] = 0;

  // Counters initialization:
  // vision_counter = 0;
  // global_counter = 0;

  // Kinect sampling time steps (at SL loop rate):
  // if ( double_rate )
  //   vision_Ts = 16;
  // else
    // vision_Ts = 33;

  // // Safety thresholds:
  // box_width_thres  = 0.10;
  // box_height_thres = 0.10;

  // Safety set for states and controller:

    // Initialization:
    // x_safe = my_vector(1,5); vec_zero(x_safe); xdd_safe = 0;
    
    // Angle threshold:
    // x_safe[1] = 2.5*M_PI/180; 
    
    // End-effector position threshold:
    // x_safe[3] = box_length_thres - 0.05;

    // End-effector acceleration threshold:
    // xdd_safe  = u_sat;
    // xdd_safe  = 5;


  // End-effector measured position, defined from the starting point, in the robot's frame:
  // cart_x_base = my_vector(1,3);

  // End-effector measured velocity, defined from the starting point, in the robot's frame:
  // cart_xd_base = my_vector(1,3);

  // End-effector measured acceleration, defined from the starting point, in the robot's frame:
  // cart_xdd_base = my_vector(1,3);

  // End-effector position dned from the starting point, in the plane of movement frame's coordiantes:
  // cart_x_plane = my_vector(1,3);

  // End-effector desired position defined from the starting point, in the plane of movement frame's coordiantes:
  // cart_des_x_plane = my_vector(1,3);

  // vec_zero(cart_x_base);
  // vec_zero(cart_xd_base);
  // vec_zero(cart_xdd_base);

  // vec_zero(cart_x_plane);
  // vec_zero(cart_des_x_plane);

  // // We define the rotation matrix from the local frame to the "plane of movement" frame:
  // rotation_angles     = my_vector(1,3);
  // rotation_angles[1]  = 0;
  // rotation_angles[2]  = 0;
  // rotation_angles[3]  = plane_angle;
  // rot_mat_            = my_matrix(1,3,1,3);
  // eulerToRotMat(rotation_angles,rot_mat_);

  // Position and velocity of the end-effector in the plane of movement:
  // cart_xd_plane  = 0.0;
  // cart_xdd_plane = 0.0;

    // Initialize the pole states always, although they are only used in simulation:
    // pole.theta   = initial_angle*M_PI/180;
    // pole.thetad  = 0.0;


    // // Discrete-time linearized system:
    // a1 = pole.mass_*g_n*pole.center_of_mass_/pole.inertia_;
    // a2 = pole.mass_*pole.center_of_mass_/pole.inertia_;
    // a3 = pole.damping_/pole.inertia_;

    // Ar  = my_matrix(1,2,1,2);   Br  = my_vector(1,2);   Cr  = my_vector(1,2);

    // Ar[1][1] = 1; Ar[1][2] = time_step;  Ar[2][1] = a1*time_step; Ar[2][2] = 1-a3*time_step;
    // Br[1] = 0; Br[2] = -a2*time_step;
    // Cr[1] = 1; Cr[2] = 0;

  // Constant measurement offset:
  // meas_offset = 0.6*M_PI/180;

  // Groundtruth:
  // angle_groundtruth = 0.0;

  // if ( double_rate )
  //   sigma_n = 0.0219;
  // else


  // Prediction states:
  // x_pred               = my_vector(1,2);

  // Butterworth filtering variables:
  // uptonow_measured_angle = 0;
  // uptonow_velocity       = 0;
  // vis_states_filtered    = my_vector(1,2);
  // filtered_angle         = 0;
  // previous_angle         = 0;
  // ind_vel                = 0;

  // Control queue:
  // u_queue = my_vector(1,PREDICTION_HORIZON);
  // vec_zero(u_queue);



  // // Control gain initialization:
  // if ( !control_gain_initialization() )
  //   return FALSE;

  // int Cont_dim  = 5;

  // // Initialization of the safe set of gains:
  // if( !copy_gains_from_file(fname_safe,F_safe) )
  //   return FALSE;

  // // Initialization of the set of gains for exploration:
  // if( !copy_gains_from_file(fname_search,F_search) )
  //   return FALSE;

  // Initialization of the current control gain with safe one:
  // vec_equal(F_safe,F);


  // // Filter variables:
  // angle_filt.x[0] = 0; angle_filt.x[1] = 0; angle_filt.x[2] = 0;
  // angle_filt.y[0] = 0; angle_filt.y[1] = 0; angle_filt.y[2] = 0;
  // vel_filt.x[0]   = 0;   vel_filt.x[1] = 0;   vel_filt.x[2] = 0;
  // vel_filt.y[0]   = 0;   vel_filt.y[1] = 0;   vel_filt.y[2] = 0;
  // acc_filt.x[0]   = 0;   acc_filt.x[1] = 0;   acc_filt.x[2] = 0;
  // acc_filt.y[0]   = 0;   acc_filt.y[1] = 0;   acc_filt.y[2] = 0;

  // // Update the structure bw_filter using the selected filter:
  // get_filter_parameters(which_filter_pole);

  // // Position filter:
  // angle_filt.a[0] = bw_filter.a[0]; angle_filt.a[1] = bw_filter.a[1]; angle_filt.a[2] = bw_filter.a[2];
  // angle_filt.b[0] = bw_filter.b[0]; angle_filt.b[1] = bw_filter.b[1]; angle_filt.b[2] = bw_filter.b[2]; 

  // // Velocity filter:
  // vel_filt.a[0] = bw_filter.a[0]; vel_filt.a[1] = bw_filter.a[1]; vel_filt.a[2] = bw_filter.a[2];
  // vel_filt.b[0] = bw_filter.b[0]; vel_filt.b[1] = bw_filter.b[1]; vel_filt.b[2] = bw_filter.b[2]; 

  // // Update the structure bw_filter using the selected filter:
  // get_filter_parameters(which_filter_xdd);

  // // End-effector acceleration filters:
  // acc_filt.a[0] = bw_filter.a[0]; acc_filt.a[1] = bw_filter.a[1]; acc_filt.a[2] = bw_filter.a[2];
  // acc_filt.b[0] = bw_filter.b[0]; acc_filt.b[1] = bw_filter.b[1]; acc_filt.b[2] = bw_filter.b[2]; 

  // // We select which function for getting new measurements we should use and initialize some variables:
  // if (sim_flag)
  // {
  //   // New measurements are read by simulating the latency by selecting this function:
  //   new_measurement_available = new_meas_simulation;

  //   // Latency simulation parameters:
  //   len = vision_Ts + MEAS_LATENCY;
  //   store_meas = my_vector(1,len);
  //   vec_zero(store_meas);
  //   ind_del = vision_Ts;
  //   ind = 0;
  // }
  // else
  // {
  //   // New measurements are read from the vision program by selecting this function:
  //   new_measurement_available = new_meas_realtime;

  //   // // Training the initial offset:
  //   // get_int("Do you want to train the offset of the initial angle by using the vision system?: ", train_init_angle, &train_init_angle); 
  //   // if ( train_init_angle )
  //   //   if (!compute_angle_offset())
  //   //     return FALSE;
  // }



  // // We capture the initial desired position of the cart (plotting purposes):
  // cart_des_x_offset = cart_des_state[1].x[_x_];
  // cart_des_y_offset = cart_des_state[1].x[_y_];
  // cart_des_z_offset = cart_des_state[1].x[_z_];

}

ParametersInitialization::~ParametersInitialization(){}

void
ParametersInitialization::apply_user_changes(void){

  // Current measured angle:
  this->current_angle_measured = this->initial_angle*M_PI/180;

  // Initialize the pole angle, for visualization:
  this->angle_visualization = this->initial_angle*M_PI/180;
}

// void     
// ParametersInitialization::vec_zero(double * a, int length)     
// {
//   int i;
//   for (i=0; i<length; ++i) 
//     a[i] = 0.0;
// }

// void     
// ParametersInitialization::vec_zero(int * a, int length)     
// {
//   int i;
//   for (i=0; i<length; ++i) 
//     a[i] = 0.0;
// }


