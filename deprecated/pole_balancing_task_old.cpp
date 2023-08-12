/*!=============================================================================
  ==============================================================================

  \file    pole_balancing_task.c

  \author  Alonso Marco
  \date    Apr. 2015

  ==============================================================================
  \remarks
  
  Makes Apollo to balance an inverted pendulum with an LQR controller, relying
  on a KF and on latency compensation.
  
  ============================================================================*/

#include "lqr_gains/subscribe_to_gains.hpp"
#include "pole_balancing_apollo/pole_angle.hpp"
#include "pole_balancing_apollo/pole_balancing_task.h"
#include "pole_balancing_apollo/butter_filt.hpp"
#include "pole_balancing_apollo/fake_sensor.hpp"


// ------------------------------------------------- global variables -------------------------------------------------//

static std::shared_ptr<Pole_angle> pole_angle_getter    = NULL; // object providing angle of pole during runtime
static std::shared_ptr<SuscribeToGains> collect_safe    = NULL; // object providing vector of safe gains during runtime
static std::shared_ptr<SuscribeToGains> collect_search  = NULL; // object providing vector of search gains during runtime

static std::shared_ptr<ButterFilt> bfilt_apos = NULL;
static std::shared_ptr<ButterFilt> bfilt_avel = NULL;
static std::shared_ptr<ButterFilt> bfilt_xacc = NULL;
static std::shared_ptr<ButterFilt> bfilt_xd   = NULL;
static std::shared_ptr<ButterFilt> bfilt_x    = NULL;

static std::shared_ptr<FakeSensor> sensor          = NULL;

static std::shared_ptr<FakeSensor> sensor_cart_x_base_X = NULL;
static std::shared_ptr<FakeSensor> sensor_cart_x_base_Y = NULL;
static std::shared_ptr<FakeSensor> sensor_cart_x_base_Z = NULL;

static std::shared_ptr<FakeSensor> sensor_cart_xd_base_X = NULL;
static std::shared_ptr<FakeSensor> sensor_cart_xd_base_Y = NULL;
static std::shared_ptr<FakeSensor> sensor_cart_xd_base_Z = NULL;

static std::shared_ptr<FakeSensor> sensor_cart_xdd_base_X = NULL;
static std::shared_ptr<FakeSensor> sensor_cart_xdd_base_Y = NULL;
static std::shared_ptr<FakeSensor> sensor_cart_xdd_base_Z = NULL;

static std::shared_ptr<FakeSensor> sensor_cart_x_local_X = NULL;
static std::shared_ptr<FakeSensor> sensor_cart_x_local_Y = NULL;
static std::shared_ptr<FakeSensor> sensor_cart_x_local_Z = NULL;


// --------------------------------------------------------------------------------------------------------------------//


static int 
init_pole_balancing_apollo(void)
{
  int j, i, ans, k;
  char string[100];

  /* Get pole angle: */
  pole_angle_getter.reset(new RosRT_pole_angle());

  /* Get LQR gains: */
    // Name of the ROS node:
    sprintf(NODE,"%s","LQR_gains_subscriber");

    // Set the topic names:
    sprintf(SAFE_GAINS_TOPIC,"%s","/LQRgains/safe");
    sprintf(SEARCH_GAINS_TOPIC,"%s","/LQRgains/search");

    // Initialize file names:
    sprintf(fname_safe,"LQR_gains_safe.cf");
    sprintf(fname_search,"LQR_gains_search.cf");

    // Object node for collecting LQR gains:
    collect_safe.reset(new CollectLQRGains(NODE,SAFE_GAINS_TOPIC));
    collect_search.reset(new CollectLQRGains(NODE,SEARCH_GAINS_TOPIC));

  // Real-time experiment or simulation:
  get_int("Please specify whether this is a simulation or a real-time experimet:\n    [1] Simulation\n    [0] Real-time experiment\nYour choice: ",sim_flag,&sim_flag);

  if ( sim_flag )
  {
    // Measurement noise:
    get_int("Do you want to simulate measurement noise?", simulate_noise, &simulate_noise);

    // Initial angle:
    get_double("Please, set an initial angle for the simulated pole [deg]: ", initial_angle, &initial_angle);

    // // Switch to 60Hz in simulation:
    // get_int("Do you want record data at 60Hz?", double_rate, &double_rate);

    // Simulated latency:
    get_int("Please, specify how much vision latency you want to simulate [ms]: ", MEAS_LATENCY, &MEAS_LATENCY);
    get_int("Please, specify how much tracking latency you want to simulate [ms]: ", CART_LATENCY, &CART_LATENCY);
  }
  
  get_double("Please, specify the duration of the whole task [s]: ",total_tt,&total_tt);
  get_double("Please, specify the duration of each experiment [s]: ",total_et,&total_et);

  // Select which filter should be applied:
  printf("2nd-order Butterworth filter for the pole states:\n");
  get_int("    Select the cutoff frequency of the that should be applied: \n    [1] 1.0Hz\n    [2] 5.0Hz\n    [3] 10.0Hz\n    [4] 15.0Hz\n    [5] 20.0Hz\n    Your choice: ", which_filter_pole, &which_filter_pole);

  printf("2nd-order Butterworth filter for the end-effector acceleration:\n");
  get_int("    Select the cutoff frequency of the that should be applied: \n    [1] 1.0Hz\n    [2] 5.0Hz\n    [3] 10.0Hz\n    [4] 15.0Hz\n    [5] 20.0Hz\n    Your choice: ", which_filter_xdd, &which_filter_xdd);

  // // Select which pole is being used:
  // get_int("Select which pole is being used:\n    [0] Short pole\n    [1] Long pole\n    Your choice: ", long_pole, &long_pole);

  // // Prediction horizon initialization:
  // get_int("Please, specify the prediction horizon [ms]: ", PREDICTION_HORIZON, &PREDICTION_HORIZON);

  // Control saturation:
  get_double("Please, set a maximum value for the end-effector's acceleration [m/s2]: ",u_sat,&u_sat);

  // Promts for changing the safety thresholds:
  get_int("Do you want to change the dimensions of the safety box?", change_safety, &change_safety);

  // Changing the limits of the safety box:
  if ( change_safety )
    get_double("Please, set half-the-length of the box: ",box_length_thres,&box_length_thres);

  // Considering an integral action:
  get_int("Do you want to activate the integral action?: ", integral_action, &integral_action);

  // Pre-experiment safety check:
  get_int("Please, select one of the options below:\n    [1] Set to zero the control commanded to the robot\n    [0] Balance the pole normally\nYour choice: ", only_recording, &only_recording);

  // Pre-experiment safety check:
  get_int("Do you want to use the head?\n    [1] Yes\n    [0] No\nYour choice: ", using_head, &using_head);

  // The robot is moved to its initial position:
  HR_offset_user = 30;
  if ( !move_to_initial_position() )
    return FALSE;

 // Move the head to the initial position:
  if ( using_head )
    if ( !move_head_to_initial_position() )
     return FALSE;

  ans = 999;
  while (ans == 999) {
    if (!get_int("Enter 1 when the gains in LQR_gains_safe.cf are correct, or anthing else to abort ...",ans,&ans))
      return FALSE;
  }
  if (ans != 1) 
    return FALSE;

  // We initialize the task parameters:
  printf("Parameters initialization...\n");
  if (!parameters_initialization())
    return FALSE;
  printf("Done!\n");

  // Promts for changing the safety thresholds:
  get_int("Do you want to change the safety constraints?", change_safe_set, &change_safe_set);

  // Changing the limits of the safety box:
  if ( change_safe_set )
  {
    double aux_pole_angle = x_safe[1]*180/M_PI;
    get_double("Please, set the pole angle limit [deg]: ",aux_pole_angle,&aux_pole_angle);
    x_safe[1] = aux_pole_angle/180*M_PI;
    get_double("Please, set the end-effector's position limit [m]: ",x_safe[3],&x_safe[3]);
    get_double("Please, set the end-effector's acceleration limit [m/s2]: ",xdd_safe,&xdd_safe);
  }

  // Promts for changing the allowed transient time between unstable and stable controller:
  get_int("Do you want to change the maximum allowed transient time?", transient_time_ms, &transient_time_ms);

  // Decide whether the safe controller should or not be loaded after each experiment. If not, the current F_search will be used.
  get_int("Should the safe controller be loaded right after each experiment?", back_to_safety_after_experiments, &back_to_safety_after_experiments);

  // // Changing the
  // get_int("Do you want to use the forgetting time-window", use_forg_factor, &use_forg_factor);

  // Prompts for modifying cartesian feedback loop gains:
  get_int("Do you want to change the cartesian feedback CL gains?", change_cart_gains, &change_cart_gains);

  if ( change_cart_gains )
  {
    get_double("Please, specify KP_pose[1]", KP_pose[1], &KP_pose[1]);
    get_double("Please, specify KP_pose[2]", KP_pose[2], &KP_pose[2]);
    get_double("Please, specify KP_pose[3]", KP_pose[3], &KP_pose[3]);
    get_double("Please, specify KD_pose[1]", KD_pose[1], &KD_pose[1]);
    get_double("Please, specify KD_pose[2]", KD_pose[2], &KD_pose[2]);
    get_double("Please, specify KD_pose[3]", KD_pose[3], &KD_pose[3]);
    get_double("Please, specify KI_pose[1]", KI_pose[1], &KI_pose[1]);
    get_double("Please, specify KI_pose[2]", KI_pose[2], &KI_pose[2]);
    get_double("Please, specify KI_pose[3]", KI_pose[3], &KI_pose[3]);
  }

  // Prompts for changing the control gain:
  get_int("Do you want to change the LQR control gain matrix?", control_change, &control_change);

  // Changing the control gain:
  if ( control_change )
  {
    k = 1;
    printf(WARN "Do not forget the minus sign!!!\n");
    while ( k<=F[0] )
    {
      printf("Changing F[%i]...\n",k);
      get_double("Please, set a new gain: ",F[k],&F[k]);
      ++k;
    }
  }

  // Promts for recording data:
  get_int("Do you want to record the data?", record_data, &record_data);

  // Preparing for the inverse kinematics of the endeffector:
    // Which dimensions of endeffector to be included in inverse kinematics?
    // All of right arm.
    for (i=1; i<=6; ++i)
      cstatus[i] = 1;

  /* do we really want to do this task? */
  ans = 999;
  while (ans == 999) {
    if (!get_int("Enter 1 to start to balance the pole or anthing else to abort ...",ans,&ans))
      return FALSE;
  }
  if (ans != 1) 
    return FALSE;
  
  // Recording data:
  if (record_data){
    scd();
    // sendMessageMotorServo("scdMotor",NULL,0);
  }

  // Reading the end-effector's initial position, right before starting the task, to be treated as an offset:
  cart_x_offset = cart_state[1].x[_X_];
  cart_y_offset = cart_state[1].x[_Y_];
  cart_z_offset = cart_state[1].x[_Z_];

  // Initialization of cart_target_state (otherwise the position starts at zero and it behaves funny):
  cart_target_state[1].x[_X_] = cart_x_offset;
  cart_target_state[1].x[_Y_] = cart_y_offset;
  cart_target_state[1].x[_Z_] = cart_z_offset;

  cart_target_state[1].xd[_X_] = 0.0;
  cart_target_state[1].xd[_Y_] = 0.0;
  cart_target_state[1].xd[_Z_] = 0.0;

  cart_target_state[1].xdd[_X_] = 0.0;
  cart_target_state[1].xdd[_Y_] = 0.0;
  cart_target_state[1].xdd[_Z_] = 0.0;


  bfilt_apos.reset(new ButterFilt(which_filter_pole));
  bfilt_avel.reset(new ButterFilt(which_filter_pole));
  bfilt_xacc.reset(new ButterFilt(which_filter_xdd));
  bfilt_xd.reset(new ButterFilt(which_filter_xd));
  bfilt_x.reset(new ButterFilt(which_filter_x));

  sensor.reset(new FakeSensor(MEAS_LATENCY));

  sensor_cart_x_base_X.reset(new FakeSensor(CART_LATENCY));
  sensor_cart_x_base_Y.reset(new FakeSensor(CART_LATENCY));
  sensor_cart_x_base_Z.reset(new FakeSensor(CART_LATENCY));

  sensor_cart_xd_base_X.reset(new FakeSensor(CART_LATENCY));
  sensor_cart_xd_base_Y.reset(new FakeSensor(CART_LATENCY));
  sensor_cart_xd_base_Z.reset(new FakeSensor(CART_LATENCY));

  sensor_cart_xdd_base_X.reset(new FakeSensor(CART_LATENCY));
  sensor_cart_xdd_base_Y.reset(new FakeSensor(CART_LATENCY));
  sensor_cart_xdd_base_Z.reset(new FakeSensor(CART_LATENCY));

  sensor_cart_x_local_X.reset(new FakeSensor(CART_LATENCY));
  sensor_cart_x_local_Y.reset(new FakeSensor(CART_LATENCY));
  sensor_cart_x_local_Z.reset(new FakeSensor(CART_LATENCY));
  

  // We initialize time counters:
  start_time          = task_servo_time;




  return TRUE;

}


static int 
run_pole_balancing_apollo(void)
{

  #ifdef __XENO__
    SRTIME wait_start = rt_timer_ticks2ns(rt_timer_read());
  #endif

  // We enter here right after detecting unstability, and only because an experiment was running:
  if (save_in_next && record_data)
  {
    stopcd();
    sendCommandLineCmd("saveData");
    save_in_next = FALSE;
  }

  // When this is uncommented, it doesn't get stucked at visualizePole() -> sendUserGraphics()
  double dummy = pole_angle_getter->get();

  // Cheking whether the task space limits are reached:
  if( !within_task_space_limits() )
    return FALSE;

  // Checking whether the joint limits are reached:
  if( !within_joint_limits() )
    return FALSE;

  // Stop the task based on time contraints:
  if( !stop_criteria() )
    return FALSE;

  // Wait some time until the transient is gone.
  wait_until_transient_gone();

  // Check whether the safety set for states and control input is about to be abandoned.
  if ( !recovering_from_unstability )
    within_safety_set();

  // Check the elapsed experiment time, and save data when total_tt reached:
  if (record_data && exp_running)
    check_elapsed_time_current_experiment();

  // Pole visualization:
  visualizePole(angle_visualization);

  // Update measurements:
  update_measurements();

  // Update vis_states_filtered[]
  compute_states();

  // Integrator update:
  if (integral_action)
    integrator_update();

  // Check whether the file LQR_gains_search.cf externally changed. If so, we start recording data from a new experiment:
  if ( detect_new_LQR() )
    start_new_experiment();

  // Control computation:
  u_to_be_sent = control_input_computation(vis_states_filtered);

  // Update the states of the simulated pole with its dynamics:
  if ( sim_flag )
    simulatePoleDynamics();

  // Apply the control to the real robot:
  if(!send_control())
    return FALSE;

 // Moving the head:
  if ( using_head )
    if(!move_head())
      return FALSE;

  #ifdef __XENO__
    SRTIME wait_end = rt_timer_ticks2ns(rt_timer_read());
    elapsed_time_run = rt_timer_ticks2ns( wait_end - wait_start)*0.001;
    
    if (elapsed_time_run > 900){
      rt_printf(CYAN "Above 900 us\n");
      rt_printf(BLACK "");
      printPrompt();
    }

  #endif

  return TRUE;
}


// Logic for updating the LQR gains only when the file LQR_gains_search.cf externally changes:
static int
detect_new_LQR(void)
{
  int external_change = FALSE;

  // Check whether the search gains are available on real time:
  if( collect_search->correctly_read() )
  {

    // Copy memory address:
    const double * read_search_gains = new double;
    read_search_gains = collect_search->get_gains();

    if ( *read_search_gains != F_search[1] )
    {
      external_change = TRUE;

      // Copy the new set of search gains:
      for(int i=0;i<F[0];++i)
        F_search[i+1] = *(read_search_gains + i);

      // Verbosity:
      #ifdef __XENO__
        rt_printf(BLUE "\n** LQR gains have changed inside %s !!!\n",fname_search);
        rt_printf("** LQR search gains succesfully collected!!!\n");
        rt_printf("   ( Sanity check: F_search[1] = %f )\n" BLACK,F_search[1]);
      #else
        printf(BLUE "\n** LQR gains have changed inside %s !!!\n",fname_search);
        printf("** LQR search gains succesfully collected!!!\n");
        printf("   ( Sanity check: F_search[1] = %f )\n" BLACK,F_search[1]);
      #endif
    }
  }

  return external_change;
}


static int
start_new_experiment(void)
{

  // Copy search gains to the current control gain vector:
  vec_equal(F_search,F);

  // We start recording data:
  if(record_data)
  {
    // Increment ES counter:
    ES_iter_counter++;

    // Verbosity:
    #ifdef __XENO__
      rt_printf(BLUE "\n\n");
      rt_printf("******************************************************\n");
      rt_printf("************** STARTING EXPERIMENT N. %i **************\n",ES_iter_counter);
      rt_printf("******************************************************\n");
      rt_printf(BLACK "\n");
    #else
      printf(BLUE "\n\n");
      printf("******************************************************\n");
      printf("************** STARTING EXPERIMENT N. %i **************\n",ES_iter_counter);
      printf("******************************************************\n");
      printf(BLACK "\n");
    #endif

    // Start recording data:
    start_time_exp = task_servo_time;
    scd();

    #ifdef __XENO__
      rt_printf(BLUE "** Recording experiment data for %f seconds...\n" BLACK,total_et);
    #else
      printf(BLUE "** Recording experiment data for %f seconds...\n" BLACK,total_et);
    #endif

    printPrompt_color();

    // Flag activated when the experiment is runnning:
    exp_running = TRUE;
  }

  return TRUE;
}

static int
change_pole_balancing_apollo(void)
{
  // // Check whether the search gains are available on real time:
  // if( collect_search->correctly_read() )
  // {
  //   // Copy memory address:
  //   const double * read_search_gains = new double;
  //   read_search_gains = collect_search->get_gains();

  //   // Copy the new set of search gains:
  //   for(int i=0;i<F[0];++i)
  //     F[i+1] = *(read_search_gains + i);

  //   // Verbosity:
  //   #ifdef __XENO__
  //     rt_printf(BLUE "\n** New set of LQR gains loaded from %s !!!\n",fname_search);
  //     rt_printf("** LQR search gains succesfully collected!!!\n");
  //     rt_printf("   ( Sanity check: F_search[1] = %f )\n" BLACK,F_search[1]);
  //   #else
  //     printf(BLUE "\n** New set of LQR gains loaded from %s !!!\n",fname_search);
  //     printf("** LQR search gains succesfully collected!!!\n");
  //     printf("   ( Sanity check: F_search[1] = %f )\n" BLACK,F_search[1]);
  //   #endif
  // }

  // // Verbosity:
  // #ifdef __XENO__
  //   rt_printf("\n\n");
  //   rt_printf("Nothing happens...\n");
  // #else
  //   printf("\n\n");
  //   printf("Nothing happens...\n");
  // #endif

  char flag_value[6];

  if ( record_data == TRUE ){
    record_data = FALSE;
    sprintf(flag_value,"FALSE");
  }
  else{
    record_data = TRUE;
    sprintf(flag_value,"TRUE");
  }

  // Verbosity:
  #ifdef __XENO__
    rt_printf("\n");
    rt_printf(BLUE "** Changing recording data flag to %s\n" BLACK,flag_value);
  #else
    printf("\n");
    printf(BLUE "** Changing recording data flag to %s\n" BLACK,flag_value);
  #endif

  return TRUE;
}


/*****************************************************************************
******************************************************************************
  \note   freeze_inactive_joints
  \date   Feb. 2015
  \author Alonso Marco

  \remarks
  
  All the joints that do not contribute to the task are set still to the initial position

******************************************************************************
  Parameters:  (i/o = input/output)

  none

 *****************************************************************************/
static int
freeze_inactive_joints (SL_DJstate jstate[])
{
  int i=0;

  // Set all target joints to zero. Exceptions are set seperately afterwards
  for (i=L_SFE; i<=N_ROBOT_DOFS; ++i)
  {
    if ( i != B_HN && i != B_HT && i != B_HR)
    {
      jstate[i].th = initial_position[i].th;
      jstate[i].thd = 0.0;
      jstate[i].thdd = 0.0;
      jstate[i].uff = 0.0;
    }
  }

  return TRUE;
}

static int
move_head(void)
{
  // Target computation:
  head_target_computation();

  // Inverse Kinematics:
  if ( !head_inverse_kinematics() ) {
    freeze();
    return FALSE;
  }

  return TRUE;
}

static void
head_target_computation(void)
{

  // Head position:
  double deltaX = cart_state[1].x[_X_] - joint_origin_pos[B_HR][_X_];
  double deltaY = cart_state[1].x[_Y_] - joint_origin_pos[B_HR][_Y_];
  double deltaZ = 0.3;

  // Computing the pan angle of the head:
  // theta_pan  =  atan2(deltaX,deltaY);
  theta_pan  =  atan2_save(deltaY,deltaX);

  // Computing the tilt angle of the head:
  // theta_tilt = -atan2(deltaZ,Norm(deltaX,deltaY));
  theta_tilt = -atan2_save(Norm(deltaX,deltaY),deltaZ);

}

static int
head_inverse_kinematics(void)
{
  int k;
  double auxd;
  int ind[2]; 
  double new_head_angle[3];
  double sec = 1.0;

  // Some coordinate transformations are needed:
  ind[0] = B_HN;
  ind[1] = B_HT;
  ind[2] = B_HR;

  new_head_angle[0] = cos(theta_pan)*theta_tilt;
  new_head_angle[1] = sin(theta_pan)*theta_tilt;
  new_head_angle[2] = theta_pan; 

  // Computing the desired kinematics for the head:
  for(k=0;k<=2;++k)
  {
    auxd = ( new_head_angle[k] - joint_des_state[ind[k]].th )*(double)task_servo_rate;
    joint_des_state[ind[k]].thdd = ( auxd - joint_des_state[ind[k]].thd )*(double)task_servo_rate;
    joint_des_state[ind[k]].thd  = auxd;
    joint_des_state[ind[k]].th   = new_head_angle[k];

    // Saturation:
    if ( joint_des_state[ind[k]].th > sec*joint_range[ind[k]][MAX_THETA] )
    {
        joint_des_state[ind[k]].th = sec*joint_range[ind[k]][MAX_THETA];
        joint_des_state[ind[k]].thd = 0.0;
        joint_des_state[ind[k]].thdd = 0.0;
    }
    if ( joint_des_state[ind[k]].th < sec*joint_range[ind[k]][MIN_THETA] )
    {
        joint_des_state[ind[k]].th = sec*joint_range[ind[k]][MIN_THETA];
        joint_des_state[ind[k]].thd = 0.0;
        joint_des_state[ind[k]].thdd = 0.0;
    }
  }

  return TRUE;
}

/*****************************************************************************
******************************************************************************
  \note   move_to_initial_position
  \date   Feb. 2015
  \author Alonso Marco

  \remarks
  
  Move the robot to the initial position: hands closed prepared for holding
  the pole, elbow in 90 degrees with respect to default position and
  end-effector frame perfectly aligned with local frame.

******************************************************************************
  Parameters:  (i/o = input/output)

  none

 *****************************************************************************/
static int
move_to_initial_position(void)
{

  int ans, i, is_reached;

  // We first initialize the offsets of each joint:
  // NOTE: we do not call this function anymore, since the initial position of the arm is hardcoded.
  // joint_offsets_initialization();

  // check whether any other task is running:
  if (strcmp(current_task_name,NO_TASK) != 0) {
    printf("New task can only be run if no other task is running!\n");
    return FALSE;
  }
  
  // Setting all target joints to the default posture
  bzero((char *)&(initial_position[1]),N_DOFS*sizeof(initial_position[1]));
  for (i=1; i<=N_DOFS; i++){
    initial_position[i].th = joint_default_state[i].th;
    initial_position[i].thd = 0;
    initial_position[i].thdd = 0;
    initial_position[i].uff = 0;
  }

  get_int("Moving the arm to the initial position:\n    [1] The pole must be placed on Apollo's hand\n    [0] The pole is already placed on its hand\nYour choice: ",prepare,&prepare);
  if(prepare) // We prepare the pendulum for the first time:
  {
    // Setting initial position:
      initial_position[R_SFE].th  =  0.081;
      initial_position[R_SAA].th  = -0.013;
      initial_position[R_HR].th   =  0.199;
      initial_position[R_EB].th   =  1.499;
      initial_position[R_WR].th   = -0.054;
      initial_position[R_WFE].th  = -0.323;
      initial_position[R_WAA].th  =  0.012;
      initial_position[R_FR].th   =  0;

/*      if ( using_head ){
        initial_position[B_HR].th = 0.730680;
        initial_position[B_HN].th = 0.136615;
      }
      else{
        // initial_position[B_HR].th = joint_default_state[B_HR].th;
        // initial_position[B_HN].th = joint_default_state[B_HN].th;
        initial_position[B_HR].th = -0.001;
        initial_position[B_HN].th = -0.001;
      }*/

      // Promt to user:
      ans = 999;
      while (ans == 999) {
        if (!get_int("Enter 1 to move to the initial position (with the hand opened), or anything else to abort ...",ans,&ans))
          return FALSE;
      }
      if (ans != 1)
        return FALSE;

      printf("\n    Moving to intial position with the hand opened...\n\n");

      // Move to starting position
      is_reached = FALSE;
      while (!is_reached)
      {
        // Checking whether any other task is running:
        if (strcmp(current_task_name,NO_TASK) != 0)
          taskDelay(ns2ticks(4000000000)); // 5s
        else
          // Go to the initial position (with Inverse Dynamics)
          is_reached = go_target_wait_ID(initial_position);
      }

      printf("\nInitial position reached!\n\n");

    // Closing the hand:
      // Promt to user:
      ans = 999;
      while (ans == 999) {
        if (!get_int("Prepare the pole and enter 1 to close the right hand, or anything else to abort ...",ans,&ans))
          return FALSE;
      }
      if (ans != 1)
        return FALSE;
      // Target with right-hand fingers closed:
      initial_position[R_LF].th = 2.43;
      initial_position[R_MF].th = 2.43;
      initial_position[R_RF].th = 2.43;

      printf("\n    Closing the hand...\n\n");
      // Closing the hand:
      is_reached = FALSE;
      while (!is_reached)
      {
        // Checking whether any other task is running:
        if (strcmp(current_task_name,NO_TASK) != 0)
          taskDelay(ns2ticks(3000000000)); // 2s
        else
          is_reached = go_target_wait_ID(initial_position);
      }

      printf("\nHand closed!\n\n");
  }
  else // Apollo is already at the initial position:
  {
    // Setting initial position:
      initial_position[R_SFE].th  =  0.081;
      initial_position[R_SAA].th  = -0.013;
      initial_position[R_HR].th   =  0.199;
      initial_position[R_EB].th   =  1.499;
      initial_position[R_WR].th   = -0.054;
      initial_position[R_WFE].th  = -0.323;
      initial_position[R_WAA].th  =  0.012;
      initial_position[R_FR].th   =  0;
      initial_position[R_LF].th   = 2.43;
      initial_position[R_MF].th   = 2.43;
      initial_position[R_RF].th   = 2.43;

/*      if ( using_head ){
        initial_position[B_HR].th = 0.730680;
        initial_position[B_HN].th = 0.136615;
      }
      else{
        // initial_position[B_HR].th = joint_default_state[B_HR].th;
        // initial_position[B_HN].th = joint_default_state[B_HN].th;
        initial_position[B_HR].th = -0.001;
        initial_position[B_HN].th = -0.001;
      }*/

    // Promt to user:
    ans = 999;
    while (ans == 999) {
      if (!get_int("Enter 1 to move to the initial position (with the hand closed), or anything else to abort ...",ans,&ans))
        return FALSE;
    }
    if (ans != 1)
      return FALSE;

    printf("\n    Moving to the intial position with the hand closed...\n\n");

    // Going to the initial position:
    is_reached = FALSE;
    while (!is_reached)
    {
      // Checking whether any other task is running:
      if (strcmp(current_task_name,NO_TASK) != 0)
        taskDelay(ns2ticks(5000000000)); // 2s
      else
        is_reached = go_target_wait_ID(initial_position);
    }

    printf("\nInitial position reached!\n\n");
  }

  return TRUE;
}

static int
move_head_to_initial_position(void)
{
  int is_reached, ans;
  double deltaX, deltaY, deltaZ;
  double theta_pan_ini,theta_tilt_ini;

  // Head position:
  deltaX = cart_state[1].x[_X_] - joint_origin_pos[B_HR][_X_];
  deltaY = cart_state[1].x[_Y_] - joint_origin_pos[B_HR][_Y_];
  deltaZ = 0.3;

  theta_pan_ini  =  atan2_save(deltaY,deltaX);
  theta_tilt_ini = -atan2_save(Norm(deltaX,deltaY),deltaZ);

  // initial_position[B_HR].th = atan2(deltaX,deltaY);
  // initial_position[B_HN].th = -atan2(deltaZ,Norm(deltaX,deltaY));
  initial_position[B_HN].th = cos(theta_pan_ini)*theta_tilt_ini;
  initial_position[B_HT].th = sin(theta_pan_ini)*theta_tilt_ini;
  initial_position[B_HR].th = theta_pan_ini;


  // Promt to user:
  ans = 999;
  while (ans == 999) {
    if (!get_int("Enter 1 to move the head to its initial position, or anything else to abort ...",ans,&ans))
      return FALSE;
  }
  if (ans != 1) 
    return FALSE;

  // Going to the initial position:
  printf("\n    Moving the head to its initial position with the hand closed...\n\n");
  is_reached = FALSE;
  while (!is_reached)
  {
    // Checking whether any other task is running:
    if (strcmp(current_task_name,NO_TASK) != 0)
      taskDelay(ns2ticks(4000000000)); // 2s
    else
      is_reached = go_target_wait_ID(initial_position);
  }

  printf("\nInitial position reached!\n\n");

  return TRUE;
}


static double
get_new_measurement(void)
{
  double angle_measured;
  if (simulate_noise)
    angle_measured  = gaussian(pole.theta, sigma_n);
  else
    angle_measured  = pole.theta;

  return angle_measured;
}


static int
parameters_initialization(void)
{
  int i,j;

  // Setting to zero all the initial accelerations and velocities:
  set_zero_dynamics();

  /* zero the filters */
  for (i=1; i<=n_dofs; ++i) 
    for (j=0; j<=FILTER_ORDER; ++j)
      fthdd[i].raw[j] = fthdd[i].filt[j] = 0;

  // Trapezoidal integration variables:
  pos_prev_x_SL = 0.0;
  pos_prev_y_SL = 0.0;
  vel_prev_x_SL = 0.0;
  vel_prev_y_SL = 0.0;
  acc_prev_x_SL = 0.0;
  acc_prev_y_SL = 0.0;
  firsttime_ = TRUE;

  save_in_next = FALSE;

  // Cartesian feedback control variables:
    // Feedback torques:
    ufb_pose_KP = my_vector(1,N_DOFS);
    ufb_pose_KD = my_vector(1,N_DOFS);
    ufb_pose_KI = my_vector(1,N_DOFS);
    int_state_pose  = my_vector(1,N_DOFS);

    // Feedback gains:
    KP_pose     = my_vector(1,3);
    KD_pose     = my_vector(1,3);
    KI_pose     = my_vector(1,3);
    // double KP_val  = 1000.0;
    // double KI_val  = 1000.0;
    double KP_val  = 2000.0;
    double KD_val  = 10.0;
    double KI_val  = 100.0;
    KP_pose[1]     = KP_val; KP_pose[2] = KP_val; KP_pose[3] = KP_val;
    KI_pose[1]     = KI_val; KI_pose[2] = KI_val; KI_pose[3] = KI_val;
    KD_pose[1]     = KD_val; KD_pose[2] = KD_val; KD_pose[3] = KD_val;

    // Number of cartesian DOFs:
    cstatus_pose = my_ivector(1,n_endeffs*6);

    // DOFs involved in the cartesian feedback control loop: [x,y,z] from right hand
    cstatus_pose[1] = 1; cstatus_pose[2] = 1; cstatus_pose[3] = 1;

  // We integrate over a window of time:
  int win_time          = 5; //Seconds
  win_length            = win_time*((int)task_servo_rate);
  integrator_window     = my_matrix(1,R_WAA,1,win_length);
  ind_win = 0;

  // Gain that solves the shifting issue:
  K_shift = 1;

  // Sign convention: when working on real-time, with the real robot, the sign of the angle acquired by
  // the vision system should be coherent with the position of the kinect: if the kinect is placed
  // in front of Apollo, then the sign must be (-). If the kinect is placed on its head, it must be (+).
  // Additionally, if working with VICON, we should pay attention to the axis convention. By now must be (+).
  sign_conv = +1;

  // Setting the time step:
  time_step = 1./(double)task_servo_rate;

  // Timers:
  start_time          = 0.0;
  start_time_exp      = 0.0;
  task_time           = 0.0;
  elapsed_time_run    = 0.0;

  // Counters initialization:
  vision_counter = 0;
  global_counter = 0;

  // Kinect sampling time steps (at SL loop rate):
  // if ( double_rate )
  //   vision_Ts = 16;
  // else
    // vision_Ts = 33;

  // Vicon sampling time [ms]:
  vision_Ts = 5;

  // Variables used in visualizePole() 
  rot_matrix   = my_matrix(1,3,1,3);
  tmp_vector   = my_vector(1,3);

  // Safety thresholds:
  box_width_thres  = 0.10;
  box_height_thres = 0.10;

  // Safety set for states and controller:

    // Initialization:
    x_safe = my_vector(1,5); vec_zero(x_safe); xdd_safe = 0;
    
    // Angle threshold:
    x_safe[1] = 2.5*M_PI/180; 
    
    // End-effector position threshold:
    x_safe[3] = box_length_thres - 0.05;

    // End-effector acceleration threshold:
    // xdd_safe  = u_sat;
    xdd_safe  = 5;

    sw2safe = 0;
    reset_  = 0;
    hold_   = 0;

  // Flag set to true when a new experiment is running:
  exp_running = FALSE; 

  // Unstability timer:
  unstability_timer_ms = 0;

  // Handling unstability:
  recovering_from_unstability = FALSE;

  // Angle of the plane of movement:
  // plane_angle = -HR_offset;
  plane_angle = -HR_offset_user*M_PI/180;

  // End-effector measured position, defined from the starting point, in the robot's frame:
  cart_x_base = my_vector(1,3);

  // End-effector measured velocity, defined from the starting point, in the robot's frame:
  cart_xd_base = my_vector(1,3);

  // End-effector measured acceleration, defined from the starting point, in the robot's frame:
  cart_xdd_base = my_vector(1,3);

  // End-effector position dned from the starting point, in the plane of movement frame's coordiantes:
  cart_x_plane = my_vector(1,3);

  // End-effector desired position defined from the starting point, in the plane of movement frame's coordiantes:
  cart_des_x_plane = my_vector(1,3);

  vec_zero(cart_x_base);
  vec_zero(cart_xd_base);
  vec_zero(cart_xdd_base);

  vec_zero(cart_x_plane);
  vec_zero(cart_des_x_plane);

  // Head angles:
  theta_pan  = 0.0;
  theta_tilt = 0.0;

  // We define the rotation matrix from the local frame to the "plane of movement" frame:
  rotation_angles     = my_vector(1,3);
  rotation_angles[1]  = 0;
  rotation_angles[2]  = 0;
  rotation_angles[3]  = plane_angle;
  rot_mat_            = my_matrix(1,3,1,3);
  eulerToRotMat(rotation_angles,rot_mat_);

  // Position and velocity of the end-effector in the plane of movement:
  cart_xd_plane  = 0.0;
  cart_xdd_plane = 0.0;
  cart_xdd_plane_filt = 0.0;

  // Initial angles:
  initial_angle_offset  = 0.0;

  // Current measured angle:
  current_angle_measured = 0.0;

  // System model variables and parameters:
    if ( long_pole )
    {
      // Pole physical parameters (without the middle big ball, long version):
      g_n = 9.81;
      pole.length_ = 1.067;
      pole.dist_bottom_ball_ = 0.525;
      pole.handle_height_ = 0.065;
      pole.mass_ = 0.2903;
      pole.center_of_mass_ = 0.636666256657899;
      pole.inertia_ =  0.117671340663077;
      pole.damping_ = 0.011603721783265;
    }
    else
    {
      // Pole physical parameters (without the middle big ball, short version):
      g_n = 9.81;
      pole.length_ = 1.067;
      pole.dist_bottom_ball_ = 0.525;
      pole.handle_height_ = 0.065;
      pole.mass_ = 0.265100000000000;
      pole.center_of_mass_ = 0.273212004785564;
      pole.inertia_ =  0.019788336363077;
      pole.damping_ = 0.011603721783265;
    }

    // Initialize the pole states always, although they are only used in simulation:
    pole.theta   = initial_angle*M_PI/180;
    pole.thetad  = 0.0;

    // Initialize the pole angle, for visualization:
    angle_visualization = initial_angle*M_PI/180;

    // // Discrete-time linearized system:
    // a1 = pole.mass_*g_n*pole.center_of_mass_/pole.inertia_;
    // a2 = pole.mass_*pole.center_of_mass_/pole.inertia_;
    // a3 = pole.damping_/pole.inertia_;

    // Ar  = my_matrix(1,2,1,2);   Br  = my_vector(1,2);   Cr  = my_vector(1,2);

    // Ar[1][1] = 1; Ar[1][2] = time_step;  Ar[2][1] = a1*time_step; Ar[2][2] = 1-a3*time_step;
    // Br[1] = 0; Br[2] = -a2*time_step;
    // Cr[1] = 1; Cr[2] = 0;

  // Integral action:
  int_state = 0.0;

  // Constant measurement offset:
  meas_offset = 0.6*M_PI/180;

  // Groundtruth:
  angle_groundtruth = 0.0;

  // Measurement noise std:
  // if ( double_rate )
  //   sigma_n = 0.0219;
  // else
    // sigma_n = 0.0078; // Holger's noise (kinect)
    // sigma_n = 0.013111; // Maximum noise with the kinect placed far away 
    // sigma_n = 0.0095; // 30Hz, Distance Kinect-pole: 1m
    sigma_n = 0.000051985; // Vicon


  // Prediction states:
  // x_pred               = my_vector(1,2);

  // Butterworth filtering variables:
  uptonow_measured_angle = 0;
  uptonow_velocity       = 0;
  vis_states_filtered    = my_vector(1,2);
  filtered_angle         = 0;
  previous_angle         = 0;
  ind_vel                = 0;

  // Control queue:
  u_queue = my_vector(1,PREDICTION_HORIZON);
  vec_zero(u_queue);

  // Control inputs initialization:
  // u_del   = 0;
  u_to_be_sent  = 0;
  u_send  = 0;

  // Control gain initialization:
  if ( !control_gain_initialization() )
    return FALSE;

  // Entropy Search iteration counter:
  ES_iter_counter = -1;

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

  // We select which function for getting new measurements we should use and initialize some variables:
  if (sim_flag)
  {
    // New measurements are read by simulating the latency by selecting this function:
    new_measurement_available = new_meas_simulation;

    // Latency simulation parameters:
    len = vision_Ts + MEAS_LATENCY;
    store_meas = my_vector(1,len);
    vec_zero(store_meas);
    ind_del = vision_Ts;
    ind = 0;
  }
  else
  {
    // New measurements are read from the vision program by selecting this function:
    new_measurement_available = new_meas_realtime;

    // // Training the initial offset:
    // get_int("Do you want to train the offset of the initial angle by using the vision system?: ", train_init_angle, &train_init_angle); 
    // if ( train_init_angle )
    //   if (!compute_angle_offset())
    //     return FALSE;
  }

  // Integrator part of the ufb:
  joint_error = my_vector(1,R_WAA);
  u_i         = my_vector(1,R_WAA);
  u_pd        = my_vector(1,R_WAA);

  // Controller gains:
  controller_gain_th  = my_vector(1,n_dofs);
  controller_gain_thd = my_vector(1,n_dofs);
  controller_gain_int = my_vector(1,n_dofs);

  /* read the control gains and max controls  */
  if (!read_gains("Gains.cf",controller_gain_th, controller_gain_thd, controller_gain_int))
    return FALSE;

  // We capture the initial desired position of the cart (plotting purposes):
  cart_des_x_offset = cart_des_state[1].x[_X_];
  cart_des_y_offset = cart_des_state[1].x[_Y_];
  cart_des_z_offset = cart_des_state[1].x[_Z_];

  // Variables to be recorded:
  addVarToCollect((char *)&(vis_states_filtered[1]),"angle_filt","rad",DOUBLE,TRUE);
  addVarToCollect((char *)&(vis_states_filtered[2]),"veloc_filt","rad/s",DOUBLE,TRUE);
  addVarToCollect((char *)&(previous_angle),"previous_angle","rad",DOUBLE,TRUE);
  addVarToCollect((char *)&(filtered_angle),"filtered_angle","rad",DOUBLE,TRUE);

  // Position of the end-effector in the plane of movement:
  addVarToCollect((char *)&(cart_x_plane[1]),"cart_state_x","m",DOUBLE,TRUE);
  addVarToCollect((char *)&(cart_des_x_plane[1]),"cart_des_state_x","m",DOUBLE,TRUE);
  addVarToCollect((char *)&(cart_tar_x_plane),"cart_tar_state_x","m",DOUBLE,TRUE);

  // Velocity of the end-effector in the plane of movement: 
  addVarToCollect((char *)&(cart_xd_plane),"cart_state_xd","m/s",DOUBLE,TRUE);
  addVarToCollect((char *)&(cart_des_xd_plane),"cart_des_state_xd","m/s",DOUBLE,TRUE);
  addVarToCollect((char *)&(cart_tar_xd_plane),"cart_tar_state_xd","m",DOUBLE,TRUE);

  // Acceleration of the end-effector in the plane of movement: 
  addVarToCollect((char *)&(cart_xdd_plane),"cart_state_xdd","m/s2",DOUBLE,TRUE);
  addVarToCollect((char *)&(cart_des_xdd_plane),"cart_des_state_xdd","m/s2",DOUBLE,TRUE);
  addVarToCollect((char *)&(u_send),"u_send","m/s2",DOUBLE,TRUE); // Same as cart_tar_state_xdd
  addVarToCollect((char *)&(u_to_be_sent),"u_to_be_sent","m/s^2",DOUBLE,TRUE);

  // Safety:
  addVarToCollect((char *)&(cart_xdd_plane_filt),"cart_state_xdd_f","m/s2",DOUBLE,TRUE);
  
  addVarToCollect((char *)&(int_state),"int_state","m.s",DOUBLE,TRUE);

  addVarToCollect((char *)&(u_i[R_SFE]),"u_int_R_SFE","Nm",DOUBLE,TRUE);
  addVarToCollect((char *)&(u_i[R_SAA]),"u_int_R_SAA","Nm",DOUBLE,TRUE);
  addVarToCollect((char *)&(u_i[R_HR]),"u_int_R_HR","Nm",DOUBLE,TRUE);
  addVarToCollect((char *)&(u_i[R_EB]),"u_int_R_EB","Nm",DOUBLE,TRUE);
  addVarToCollect((char *)&(u_i[R_WR]),"u_int_R_WR","Nm",DOUBLE,TRUE);
  addVarToCollect((char *)&(u_i[R_WFE]),"u_int_R_WFE","Nm",DOUBLE,TRUE);
  addVarToCollect((char *)&(u_i[R_WAA]),"u_int_R_WAA","Nm",DOUBLE,TRUE);
  addVarToCollect((char *)&(u_pd[R_SFE]),"u_pd_R_SFE","Nm",DOUBLE,TRUE);
  addVarToCollect((char *)&(u_pd[R_SAA]),"u_pd_R_SAA","Nm",DOUBLE,TRUE);
  addVarToCollect((char *)&(u_pd[R_HR]),"u_pd_R_HR","Nm",DOUBLE,TRUE);
  addVarToCollect((char *)&(u_pd[R_EB]),"u_pd_R_EB","Nm",DOUBLE,TRUE);
  addVarToCollect((char *)&(u_pd[R_WR]),"u_pd_R_WR","Nm",DOUBLE,TRUE);
  addVarToCollect((char *)&(u_pd[R_WFE]),"u_pd_R_WFE","Nm",DOUBLE,TRUE);
  addVarToCollect((char *)&(u_pd[R_WAA]),"u_pd_R_WAA","Nm",DOUBLE,TRUE);

  addVarToCollect((char *)&(sw2safe),"sw2safe","binary",INT,TRUE);
  
  // Cartesian feedback control loop variables:
  addVarToCollect((char *)&(ufb_pose_KP[R_SFE]),"uff_pose_KP_R_SFE","Nm",DOUBLE,TRUE);
  addVarToCollect((char *)&(ufb_pose_KP[R_SAA]),"uff_pose_KP_R_SAA","Nm",DOUBLE,TRUE);
  addVarToCollect((char *)&(ufb_pose_KP[R_HR]),"uff_pose_KP_R_HR","Nm",DOUBLE,TRUE);
  addVarToCollect((char *)&(ufb_pose_KP[R_EB]),"uff_pose_KP_R_EB","Nm",DOUBLE,TRUE);
  addVarToCollect((char *)&(ufb_pose_KP[R_WR]),"uff_pose_KP_R_WR","Nm",DOUBLE,TRUE);
  addVarToCollect((char *)&(ufb_pose_KP[R_WFE]),"uff_pose_KP_R_WFE","Nm",DOUBLE,TRUE);
  addVarToCollect((char *)&(ufb_pose_KP[R_WAA]),"uff_pose_KP_R_WAA","Nm",DOUBLE,TRUE);
  addVarToCollect((char *)&(ufb_pose_KD[R_SFE]),"uff_pose_KD_R_SFE","Nm",DOUBLE,TRUE);
  addVarToCollect((char *)&(ufb_pose_KD[R_SAA]),"uff_pose_KD_R_SAA","Nm",DOUBLE,TRUE);
  addVarToCollect((char *)&(ufb_pose_KD[R_HR]),"uff_pose_KD_R_HR","Nm",DOUBLE,TRUE);
  addVarToCollect((char *)&(ufb_pose_KD[R_EB]),"uff_pose_KD_R_EB","Nm",DOUBLE,TRUE);
  addVarToCollect((char *)&(ufb_pose_KD[R_WR]),"uff_pose_KD_R_WR","Nm",DOUBLE,TRUE);
  addVarToCollect((char *)&(ufb_pose_KD[R_WFE]),"uff_pose_KD_R_WFE","Nm",DOUBLE,TRUE);
  addVarToCollect((char *)&(ufb_pose_KD[R_WAA]),"uff_pose_KD_R_WAA","Nm",DOUBLE,TRUE);
  addVarToCollect((char *)&(ufb_pose_KI[R_SFE]),"uff_pose_KI_R_SFE","Nm",DOUBLE,TRUE);
  addVarToCollect((char *)&(ufb_pose_KI[R_SAA]),"uff_pose_KI_R_SAA","Nm",DOUBLE,TRUE);
  addVarToCollect((char *)&(ufb_pose_KI[R_HR]),"uff_pose_KI_R_HR","Nm",DOUBLE,TRUE);
  addVarToCollect((char *)&(ufb_pose_KI[R_EB]),"uff_pose_KI_R_EB","Nm",DOUBLE,TRUE);
  addVarToCollect((char *)&(ufb_pose_KI[R_WR]),"uff_pose_KI_R_WR","Nm",DOUBLE,TRUE);
  addVarToCollect((char *)&(ufb_pose_KI[R_WFE]),"uff_pose_KI_R_WFE","Nm",DOUBLE,TRUE);
  addVarToCollect((char *)&(ufb_pose_KI[R_WAA]),"uff_pose_KI_R_WAA","Nm",DOUBLE,TRUE);

  if ( sim_flag ){
    addVarToCollect((char *)&(current_angle_measured),"theta_real","rad",DOUBLE,TRUE);
    addVarToCollect((char *)&(pole.theta),"theta_artificial","rad",DOUBLE,TRUE);
    addVarToCollect((char *)&(pole.thetad),"thetad_real","rad/s",DOUBLE,TRUE);
  }
  else{
    addVarToCollect((char *)&(current_angle_measured),"theta_real","rad",DOUBLE,TRUE);
    addVarToCollect((char *)&(blobs[1].blob.x[_Y_]),"thetad_real","rad/s",DOUBLE,TRUE);
  }

  // Task time:
  addVarToCollect((char *)&(task_time),"task_time","s",DOUBLE,TRUE);
  addVarToCollect((char *)&(elapsed_time_run),"elapsed_time_run","s",DOUBLE,TRUE);

  // Right hand fingers:
  addVarToCollect((char *)&(joint_des_state[R_LF].th),"fin_R_LF_des","rad",DOUBLE,TRUE);
  addVarToCollect((char *)&(joint_des_state[R_MF].th),"fin_R_MF_des","rad",DOUBLE,TRUE);
  addVarToCollect((char *)&(joint_des_state[R_RF].th),"fin_R_RF_des","rad",DOUBLE,TRUE);
  addVarToCollect((char *)&(joint_state[R_LF].th),"fin_R_LF_cur","rad",DOUBLE,TRUE);
  addVarToCollect((char *)&(joint_state[R_MF].th),"fin_R_MF_cur","rad",DOUBLE,TRUE);
  addVarToCollect((char *)&(joint_state[R_RF].th),"fin_R_RF_cur","rad",DOUBLE,TRUE);

  return TRUE;

}

static int
copy_gains_from_file(char * fname, Vector F_read)
{

  // Initialization of F_search with LQR_gains_search.cf
  if (!read_controller_gains(fname,F_read))
  {
    freeze();
    #ifdef __XENO__
      rt_printf("ERROR: Could not read properly the file [%s]\n",fname);
      rt_printf("\n\n    Task is finisehd!\n\n");
    #else
      printf("ERROR: Could not read properly the file [%s]\n",fname);
      printf("\n\n    Task is finisehd!\n\n");
    #endif

    return FALSE;
  }
  else
  {
    #ifdef __XENO__
      rt_printf("Gains read sucessfully from [%s]\n",fname);
    #else
      printf("Gains read sucessfully from [%s]\n",fname);
    #endif
  }

  return TRUE;
} 

static int
control_gain_initialization(void)
{

  int Cont_dim  = 5;
  F_safe        = my_vector(1,Cont_dim);
  F_search      = my_vector(1,Cont_dim);
  F             = my_vector(1,Cont_dim);

  // Initialization of the safe set of gains:
  if( !copy_gains_from_file(fname_safe,F_safe) )
    return FALSE;

  // Initialization of the set of gains for exploration:
  if( !copy_gains_from_file(fname_search,F_search) )
    return FALSE;

  // Initialization of the current control gain with safe one:
  vec_equal(F_safe,F);

  return TRUE;
}

// // Apply 2nd-order Butterworth filter:
// static double
// apply_filter_pos(double raw_angle)
// {
//   angle_filt.x[0] = raw_angle;

//   // Filtering:
//   angle_filt.y[0] = angle_filt.b[0]*angle_filt.x[0] + 
//                     angle_filt.b[1]*angle_filt.x[1] + 
//                     angle_filt.b[2]*angle_filt.x[2] - 
//                     angle_filt.a[1]*angle_filt.y[1] - 
//                     angle_filt.a[2]*angle_filt.y[2];

//   // Assignments for next step:
//   angle_filt.x[2] = angle_filt.x[1];
//   angle_filt.x[1] = angle_filt.x[0];

//   angle_filt.y[2] = angle_filt.y[1];
//   angle_filt.y[1] = angle_filt.y[0];

//   return angle_filt.y[0];
// }

// // Apply 2nd-order Butterworth filter:
// static double
// apply_filter_vel(double raw_vel)
// {
//   vel_filt.x[0] = raw_vel;

//   // Filtering:
//   vel_filt.y[0] = vel_filt.b[0]*vel_filt.x[0] + 
//                     vel_filt.b[1]*vel_filt.x[1] + 
//                     vel_filt.b[2]*vel_filt.x[2] - 
//                     vel_filt.a[1]*vel_filt.y[1] - 
//                     vel_filt.a[2]*vel_filt.y[2];

//   // Assignments for next step:
//   vel_filt.x[2] = vel_filt.x[1];
//   vel_filt.x[1] = vel_filt.x[0];

//   vel_filt.y[2] = vel_filt.y[1];
//   vel_filt.y[1] = vel_filt.y[0];

//   return vel_filt.y[0];
// }

// // Apply 2nd-order Butterworth filter:
// static double
// apply_filter_EF_acc(double raw_angle)
// {
//   acc_filt.x[0] = raw_angle;

//   // Filtering:
//   acc_filt.y[0] = acc_filt.b[0]*acc_filt.x[0] + 
//                     acc_filt.b[1]*acc_filt.x[1] + 
//                     acc_filt.b[2]*acc_filt.x[2] - 
//                     acc_filt.a[1]*acc_filt.y[1] - 
//                     acc_filt.a[2]*acc_filt.y[2];

//   // Assignments for next step:
//   acc_filt.x[2] = acc_filt.x[1];
//   acc_filt.x[1] = acc_filt.x[0];

//   acc_filt.y[2] = acc_filt.y[1];
//   acc_filt.y[1] = acc_filt.y[0];

//   return acc_filt.y[0];
// }

static void
update_measurements(void)
{
  // printf("[DBG]: @update_measurements() 1\n");
  // Update when new measurements are available:
  if( new_measurement_available() )
    uptonow_measured_angle = current_angle_measured;

  // printf("[DBG]: @update_measurements() 2\n");

  // Update velocity when necessary:
  ind_vel = ind_vel % 5 + 1;
  if ( ind_vel == 1 )
  {
    uptonow_velocity = ( uptonow_measured_angle - previous_angle )/0.005;
    previous_angle = uptonow_measured_angle;
  }
}

// Adding the pole velocity to the pole states:
static void
compute_states(void)
{
  // Pole's states:
  vis_states_filtered[1] = bfilt_apos->apply_filter(uptonow_measured_angle);
  vis_states_filtered[2] = bfilt_avel->apply_filter(uptonow_velocity);

  // End-effector states:
  measure_endeffector_states();

  // Target and desired positions are computed in cartesian_target_computation()

}

static void
measure_endeffector_states(void)
{
  // End-effector position in the robot frame's coordiantes:
  cart_x_base[_X_] = sensor_cart_x_base_X->add_delay(cart_state[1].x[_X_]);
  cart_x_base[_Y_] = sensor_cart_x_base_Y->add_delay(cart_state[1].x[_Y_]);
  cart_x_base[_Z_] = sensor_cart_x_base_Z->add_delay(cart_state[1].x[_Z_]);

  // End-effector velocity in the robot frame's coordiantes:
  cart_xd_base[_X_] = sensor_cart_xd_base_X->add_delay(cart_state[1].xd[_X_]);
  cart_xd_base[_Y_] = sensor_cart_xd_base_Y->add_delay(cart_state[1].xd[_Y_]);
  cart_xd_base[_Z_] = sensor_cart_xd_base_Z->add_delay(cart_state[1].xd[_Z_]);

  // End-effector acceleration in the robot frame's coordiantes:
  cart_xdd_base[_X_] = sensor_cart_xdd_base_X->add_delay(cart_state[1].xdd[_X_]);
  cart_xdd_base[_Y_] = sensor_cart_xdd_base_Y->add_delay(cart_state[1].xdd[_Y_]);
  cart_xdd_base[_Z_] = sensor_cart_xdd_base_Z->add_delay(cart_state[1].xdd[_Z_]);

  // End-effector position defined from the starting point, in the local frame's coordiantes:
  Vector cart_x_local = my_vector(1,3);
  cart_x_local[_X_] = sensor_cart_x_local_X->add_delay(cart_state[1].x[_X_] - cart_x_offset);
  cart_x_local[_Y_] = sensor_cart_x_local_Y->add_delay(cart_state[1].x[_Y_] - cart_y_offset);
  cart_x_local[_Z_] = sensor_cart_x_local_Z->add_delay(cart_state[1].x[_Z_] - cart_z_offset);

  // End-effector position defined from the stang point, in the plane of movement frame's coordiantes:
  mat_vec_mult(rot_mat_,cart_x_local,cart_x_plane);

  // Cartesian velocities and accelerations in the plane of movement:
  cart_xd_plane     = sign(cart_xd_base[_X_])*Norm(cart_xd_base[_X_],cart_xd_base[_Y_]);
  cart_xdd_plane    = sign(cart_xdd_base[_X_])*Norm(cart_xdd_base[_X_],cart_xdd_base[_Y_]);

  // Desired variables:
  cart_des_xd_plane = sign(cart_des_state[1].xd[_X_])*Norm(cart_des_state[1].xd[_X_],cart_des_state[1].xd[_Y_]);
  cart_des_xdd_plane = sign(cart_des_state[1].xdd[_X_])*Norm(cart_des_state[1].xdd[_X_],cart_des_state[1].xdd[_Y_]);

}

/*!*****************************************************************************
 *******************************************************************************
\note  read_controller_gains
\date  Apr 2015
\remarks 

parses the gain configuration file into global variables

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     fname         : the name of the link parameter file
 \param[out]    control_gain  : F_pred gains

 ******************************************************************************/
static int
read_controller_gains(char *fname, double *control_gain)
{
  int     j,i,rc;
  char    file_path[100], control_gain_name[10];
  FILE    *in;
  double  dum;

  // Defining the file path:
  sprintf(file_path,"%s%s",CONFIG,fname);

  // Opening the file:
  in = fopen_strip(file_path);
  if (in == NULL) 
  {
    #ifdef __XENO__
      rt_printf("ERROR: Cannot open file >%s<!\n",file_path);
    #else
      printf("ERROR: Cannot open file >%s<!\n",file_path);
    #endif
    return FALSE;
  }

  // Reading the control gains from a file:
  for (i=1; i<= control_gain[0]; ++i)
  {
    // We initialize the control gain name:
    sprintf(control_gain_name,"F_%i",i);

    // We look for the control gain name within the file.
    // If it is found, then the reading cursor is placed right after the name:
    if (!find_keyword(in, &(control_gain_name[0])))
    {
      // #ifdef __XENO__
      //   rt_printf("** ERROR: Cannot find gains for >%s<!\n",control_gain_name);
      // #else
      //   printf("** ERROR: Cannot find gains for >%s<!\n",control_gain_name);
      // #endif
      fclose(in);
      return FALSE;
    }
    else
    {
      if ( control_gain != NULL )
        rc=fscanf(in,"%lf",&control_gain[i]);
      else
        rc=fscanf(in,"%lf",&dum);
    }
  }
  
  // Closing the file:
  fclose(in);

  return TRUE;
}

static void
integrator_update(){
  int_state += time_step*cart_x_plane[1];
}

static double
control_input_computation(Vector vis_state_BG)
{
  /* vis_state_BG := Best Guess for the states of the vision system' */

  double u_control;

  if (integral_action)
    u_control = - (F[1]*vis_state_BG[1] + F[2]*vis_state_BG[2] + F[3]*cart_x_plane[1] + F[4]*cart_xd_plane + F[5]*int_state);
  else
    u_control = - (F[1]*vis_state_BG[1] + F[2]*vis_state_BG[2] + F[3]*cart_x_plane[1] + F[4]*cart_xd_plane);

  // Control saturation:
  saturate_signal(&u_control,u_sat);

  return u_control; 
}

/*****************************************************************************
******************************************************************************
  Function Name : visualizePole
  Date          : Oct. 2014

  Remarks:

  calculates the state of a simulated pole and copies it into the vision
  states

******************************************************************************
  Paramters:  (i/o = input/output)

  none

 *****************************************************************************/
static int 
visualizePole(double angle_measured_)
{
    double current_angle;

    // TODO: when the pole has to point downwards it actually points to the direction 
    // of the x axis of the end-effector local frame.
    // quatToEulerInv(&cart_orient[RIGHT_HAND],trial_vector);
    // current_angle = pole.theta + theta_offset + trial_vector[2];
    
    current_angle = angle_measured_ - M_PI;

    // Generate rotation matrix to get from endeffector frame into base frame
    quatToRotMatInv(&cart_orient[RIGHT_HAND], rot_matrix);

    // Bottom part of the pole expressed in end-effector frame
    tmp_vector[1] = - pole.handle_height_;
    tmp_vector[2] = 0.0;
    tmp_vector[3] = 0.0;

    // Transform handle offset into base frame
    mat_vec_mult(rot_matrix, tmp_vector, tmp_vector);

    // Position of the bottom of the pole expressed from a base frame:
    pole.pole_bottom_.x[_X_] = tmp_vector[1] + cart_state[RIGHT_HAND].x[_X_];
    pole.pole_bottom_.x[_Y_] = tmp_vector[2] + cart_state[RIGHT_HAND].x[_Y_];
    pole.pole_bottom_.x[_Z_] = tmp_vector[3] + cart_state[RIGHT_HAND].x[_Z_];

    // Compute pole tip in local frame from current theta_2
    pole.pole_tip_ = computePositionOnPole(current_angle, pole.length_);

    tmp_vector[1] = pole.pole_tip_.x[_X_];
    tmp_vector[2] = pole.pole_tip_.x[_Y_];
    tmp_vector[3] = pole.pole_tip_.x[_Z_];

    // Transform into base frame
    mat_vec_mult(rot_matrix, tmp_vector, tmp_vector);
    
    // Add position of right hand
    pole.pole_tip_.x[_X_] = tmp_vector[1] + cart_state[RIGHT_HAND].x[_X_];
    pole.pole_tip_.x[_Y_] = tmp_vector[2] + cart_state[RIGHT_HAND].x[_Y_];
    pole.pole_tip_.x[_Z_] = tmp_vector[3] + cart_state[RIGHT_HAND].x[_Z_];

    // Compute bottom ball in local frame from current theta_2
    pole.ball_bottom_ = computePositionOnPole(current_angle, pole.dist_bottom_ball_);

    tmp_vector[1] = pole.ball_bottom_.x[_X_];
    tmp_vector[2] = pole.ball_bottom_.x[_Y_];
    tmp_vector[3] = pole.ball_bottom_.x[_Z_];

    // Transform into base frame
    mat_vec_mult(rot_matrix, tmp_vector, tmp_vector);

    // Add position of right hand
    pole.ball_bottom_.x[_X_] = tmp_vector[1] + cart_state[RIGHT_HAND].x[_X_];
    pole.ball_bottom_.x[_Y_] = tmp_vector[2] + cart_state[RIGHT_HAND].x[_Y_];
    pole.ball_bottom_.x[_Z_] = tmp_vector[3] + cart_state[RIGHT_HAND].x[_Z_];

    sendUserGraphics("pole",&pole, sizeof(pole));

    return TRUE;
}



static SL_Cstate
computePositionOnPole(double theta_2, double dist_to_pole_end)
{
    
      // Theta > 0 : Pole falls in negative y-axis direction
      // (To the left looking at Apollo balancing with his right arm)
     

    // This is done cuz the measured angle is referenced with the pole pointing downwards:
    double theta_rad = theta_2 - M_PI; 

    double y_diff = dist_to_pole_end * sin(theta_rad);
    double x_diff = dist_to_pole_end * cos(theta_rad);

    SL_Cstate position;
    vec_zero(position.x);
    vec_zero(position.xd);
    vec_zero(position.xdd);
    position.x[_X_] = - pole.handle_height_ - x_diff;
    position.x[_Y_] = - y_diff;
    position.x[_Z_] = 0.0;

    return position;
}

static void
simulatePoleDynamics()
{
    double new_theta_2, new_theta_2_dot;
    double theta_2_rad = pole.theta;

    // Forward Euler
    new_theta_2 = pole.theta + time_step * pole.thetad;

    new_theta_2_dot = pole.thetad;
    new_theta_2_dot += time_step * (
                pole.mass_ * g_n * pole.center_of_mass_ / pole.inertia_ * sin(theta_2_rad)
                - pole.mass_ * pole.center_of_mass_ * u_send / pole.inertia_ * cos(theta_2_rad)
                - pole.damping_ * pole.thetad / pole.inertia_);

    // Write back new values in pole-struct
    pole.theta = new_theta_2;
    pole.thetad = new_theta_2_dot;
}

static void
saturate_signal(double *sig,double max_value)
{
  if ( fabs(*sig) > max_value )
    *sig = max_value*sign(*sig);
}

static int
send_control(void)
{
  int i;
  double aux;

  update_PID_gains();

  cartesian_target_computation();

    // Inverse Kinematics (according to implementation in SL_go_cart_task.c)
    for (i=1; i<=R_WAA; ++i)
        target[i].th = joint_des_state[i].th;

    if (!inverseKinematics(target,endeff,joint_opt_state,
               cart,cstatus,time_step)) {
      freeze();
      return FALSE;
    }

    // Preparing inverse dynamics: (according to implementation in SL_go_cart_task, 
    // except for turning off fitering below)
    for (i=1; i<=R_WAA; ++i) {
        aux = (target[i].thd-joint_des_state[i].thd)*(double)task_servo_rate;
        // target[i].thdd = aux;    // no filtering -> faster response
        target[i].thdd  = filt(aux,&(fthdd[i]));    // Original implementation

        joint_des_state[i].thdd = target[i].thdd;
        joint_des_state[i].thd  = target[i].thd;
        joint_des_state[i].th   = target[i].th;

        if (joint_des_state[i].th > joint_range[i][MAX_THETA]) {
            joint_des_state[i].th = joint_range[i][MAX_THETA];
            joint_des_state[i].thd = 0.0;
            joint_des_state[i].thdd = 0.0;
        }
        if (joint_des_state[i].th < joint_range[i][MIN_THETA]) {
            joint_des_state[i].th = joint_range[i][MIN_THETA];
            joint_des_state[i].thd = 0.0;
            joint_des_state[i].thdd = 0.0;
        }
    }

    // We want to keep frozen all the joints that are not involved in the movement
    freeze_inactive_joints(joint_des_state);

    // Adds torque to joint_des_state[i].uff
    SL_InvDyn(joint_state,joint_des_state,endeff,&base_state,&base_orient);

    // Cartesian control contribution:
    if ( !cartesian_feedback_control_loop() )
      return FALSE;

  return TRUE;
}

static void
cartesian_target_computation(void)
{
  if ( only_recording )
    u_send = 0.0;
  else
    u_send = gaussian(u_to_be_sent,0.1440);

  // If we are in the pause on the middle or at the beginning, we catch the initial values for the trapezoidal integration:
  if ( firsttime_  )
  {
    cart_target_state[1].xdd[_X_] =  u_send*cos(-plane_angle);
    cart_target_state[1].xdd[_Y_] = -u_send*sin(-plane_angle);
    cart_target_state[1].xd[_X_]  = 0.0;
    cart_target_state[1].xd[_Y_]  = 0.0;
    cart_target_state[1].x[_X_]   = cart_x_offset;
    cart_target_state[1].x[_Y_]   = cart_y_offset;
    cart_target_state[1].x[_Z_]   = cart_z_offset;

    firsttime_ = FALSE;
  }
  else
  {
    // Target acceleration for the end-effector:
    cart_target_state[1].xdd[_X_] =  u_send*cos(-plane_angle);
    cart_target_state[1].xdd[_Y_] = -u_send*sin(-plane_angle);

    // Compute desired endeff velocity (from acceleration, using the trapezoidal rule)
    cart_target_state[1].xd[_X_] = vel_prev_x_SL + ( acc_prev_x_SL + cart_target_state[1].xdd[_X_] )*time_step/2;
    cart_target_state[1].xd[_Y_] = vel_prev_y_SL + ( acc_prev_y_SL + cart_target_state[1].xdd[_Y_] )*time_step/2;

    // Compute desired endeff position (from velocity, using the trapezoidal rule)
    cart_target_state[1].x[_X_] = pos_prev_x_SL + ( vel_prev_x_SL + cart_target_state[1].xd[_X_] )*time_step/2;
    cart_target_state[1].x[_Y_] = pos_prev_y_SL + ( vel_prev_y_SL + cart_target_state[1].xd[_Y_] )*time_step/2;
  }

  // Update step:
  pos_prev_x_SL = cart_target_state[1].x[_X_];
  pos_prev_y_SL = cart_target_state[1].x[_Y_];
  vel_prev_x_SL = cart_target_state[1].xd[_X_];
  vel_prev_y_SL = cart_target_state[1].xd[_Y_];
  acc_prev_x_SL = cart_target_state[1].xdd[_X_];
  acc_prev_y_SL = cart_target_state[1].xdd[_Y_];

  // Preparing cartesian target for IK: [xd, yd, zd, ad, bd, gd]
  cart[1] = cart_target_state[1].xd[_X_];
  cart[2] = cart_target_state[1].xd[_Y_];
  cart[3] = 0.;

    // // Shifting correction:
    // cart[1] += K_shift*( cart_target_state[1].x[_X_] - cart_des_state[1].x[_X_] );
    // cart[2] += K_shift*( cart_target_state[1].x[_Y_] - cart_des_state[1].x[_Y_] );
    // cart[3] += K_shift*( cart_target_state[1].x[_Z_] - cart_des_state[1].x[_Z_] );

  cart[4] = 0.;
  cart[5] = 0.;
  cart[6] = 0.;

  // Update target cartesian states:
  double aux_x = cart_target_state[1].x[_X_] - cart_x_offset;
  double aux_y = cart_target_state[1].x[_Y_] - cart_y_offset;

  cart_tar_x_plane = sign(aux_x)*Norm(aux_x,aux_y);
  cart_tar_xd_plane = sign(cart_target_state[1].xd[_X_])*Norm(cart_target_state[1].xd[_X_],cart_target_state[1].xd[_Y_]);
  cart_tar_xdd_plane = sign(cart_target_state[1].xdd[_X_])*Norm(cart_target_state[1].xdd[_X_],cart_target_state[1].xdd[_Y_]);

}

static int
cartesian_feedback_control_loop(void)
{
  int            i,j,c;
  int            count; 
  MY_MATRIX(Jac,1,6*N_ENDEFFS,1,N_DOFS);
  MY_MATRIX(Jreal,1,6*N_ENDEFFS,1,N_DOFS);
  MY_MATRIX(Jtrans,1,N_DOFS,1,6*N_ENDEFFS);
  MY_MATRIX(J_pinv,1,N_DOFS,1,6*N_ENDEFFS);
  MY_IVECTOR(ind,1,6*N_ENDEFFS);
  MY_MATRIX(local_link_pos_des,0,N_LINKS,1,3);
  MY_MATRIX(local_joint_cog_mpos_des,0,N_DOFS,1,3);
  MY_MATRIX(local_joint_origin_pos_des,0,N_DOFS,1,3);
  MY_MATRIX(local_joint_axis_pos_des,0,N_DOFS,1,3);
  MY_MATRIX_ARRAY(local_Alink_des,1,4,1,4,N_LINKS+1);
  MY_MATRIX_ARRAY(local_Adof_des,1,4,1,4,N_DOFS+1);
  double         condnr;
  double         condnr_cutoff = 70.0;  // this corresponds to condnr_cutoff^2 in invere space
  double         aux;

  // Compute the Jacobian 
  linkInformationDes(target,&base_state,&base_orient,endeff,
         local_joint_cog_mpos_des,
         local_joint_axis_pos_des,
         local_joint_origin_pos_des,
         local_link_pos_des,
         local_Alink_des,
         local_Adof_des);

  jacobian(local_link_pos_des,local_joint_origin_pos_des,local_joint_axis_pos_des,Jac);

  count = 0;
  for (i=1; i<=6*N_ENDEFFS; ++i) {
    if (cstatus_pose[i]) {
      ++count;
      ind[count] = i;
    }
  }

  // Double check:
  if (!count)
    return FALSE;

  // The Jacobian that is actually needed, i.e., only the constraint rows
  mat_zero(Jreal);
  for (i=1; i<=count; ++i)
    for (j=1; j<=N_DOFS; ++j)
      Jreal[i][j] = Jac[ind[i]][j];

  // Transpose:
  mat_trans(Jreal,Jtrans);

  // Computing the desired torques:
  for (i=1; i<=R_WAA; ++i)
  {
    ufb_pose_KP[i] = 0.0;
    ufb_pose_KD[i] = 0.0;

    if (!use_forg_factor)
      ufb_pose_KI[i] = 0.0;

    int_state_pose[i] = 0.0;
    for (j=_X_; j<=_Z_; ++j){

      ufb_pose_KP[i] += Jtrans[i][j]*KP_pose[j]*( cart_target_state[1].x[j] - cart_x_base[j] );

      ufb_pose_KD[i] += Jtrans[i][j]*KD_pose[j]*( cart_target_state[1].xd[j] - cart_xd_base[j] );

      int_state_pose[i] += Jtrans[i][j]*KI_pose[j]*( cart_target_state[1].x[j] - cart_x_base[j] )*time_step;

    }

    // // Standard time-window:
    // for(c=1;c<win_length;++c)
    // {
    //   integrator_window[i][c] = integrator_window[i][c+1];
    //   ufb_pose_KI[i] += integrator_window[i][c];
    // }
    // integrator_window[i][win_length] = int_state_pose[i];
    // ufb_pose_KI[i] += integrator_window[i][win_length];

    if (use_forg_factor)
    {
      // Forgetting factor time-window:
      ufb_pose_KI[i] -= ufb_pose_KI[i]/win_length;
      ufb_pose_KI[i] += int_state_pose[i];
    }
    else
    {
      // Standard time-window (similar):
      ind_win = ind_win % win_length + 1;
      integrator_window[i][ind_win] = int_state_pose[i];
      for(c=1;c<=win_length;++c)
        ufb_pose_KI[i] += integrator_window[i][c];
    }

    // The uff is read in the motor servo trhough the shared memory, thus we add the torque to this variable:
    joint_des_state[i].uff += ufb_pose_KP[i] + ufb_pose_KD[i] + ufb_pose_KI[i];
  }

  return TRUE;
}

static void 
update_PID_gains(void)
{
  int i;

  for (i=R_SFE; i<=R_WAA; ++i) {
    joint_error[i] = joint_des_state[i].th - joint_state[i].th;
    u_i[i]  += joint_error[i] * controller_gain_int[i];
    u_pd[i]  = joint_error[i] * controller_gain_th[i] + 
               (joint_des_state[i].thd - joint_state[i].thd) * controller_gain_thd[i];
  }
}

static void
set_zero_dynamics(void)
{
  int i,j;
  for (i=1; i<=N_DOFS; i++){
    joint_des_state[i].thd = 0;
    joint_des_state[i].thdd = 0;
//    joint_des_state[i].uff = 0;
  }

  for(i=1; i<=N_ENDEFFS;++i){
    for(j=1; j<=N_CART;++j){
      cart_target_state[i].xd[j] = 0.0;
      cart_target_state[i].xdd[j] = 0.0;
      cart_des_state[i].xd[j] = 0.0;
      cart_des_state[i].xdd[j] = 0.0;
    }

    for(j=1; j<=N_QUAT;++j){
      cart_target_orient[i].qd[j] = 0.0; /*!< Velocity */
      cart_target_orient[i].qdd[j] = 0.0; /*!< Acceleration */
      cart_des_orient[i].qd[j] = 0.0; /*!< Velocity */
      cart_des_orient[i].qdd[j] = 0.0; /*!< Acceleration */
    }

    for(j=1; j<=N_CART;++j){
      cart_target_orient[i].ad[j] = 0.0; /*!< Angular Velocity [alpha,beta,gamma] */
      cart_target_orient[i].add[j] = 0.0; /*!< Angular Acceleration */
      cart_des_orient[i].ad[j] = 0.0; /*!< Angular Velocity [alpha,beta,gamma] */
      cart_des_orient[i].add[j] = 0.0; /*!< Angular Acceleration */
    }
  }
}

static void
wait_until_transient_gone(void)
{
  // Hold the data saving for some time, until the transient is gone:
  ++unstability_timer_ms;

  if ( unstability_timer_ms > transient_time_ms )
  {
    unstability_timer_ms = 0;
    recovering_from_unstability = FALSE;
  }

}

// This function returns false only when the gains file could no be read.
// We always access it from run() 
static int
within_safety_set(void)
{

  sw2safe = 0;

  // printf("[DBG]: @within_safety_set() 1\n");

  // Filter this variables, only used for unstability detection:
  cart_xdd_plane_filt  = bfilt_xacc->apply_filter(cart_xdd_plane);

  // Safe set check-out:
  if (fabs(cart_xdd_plane_filt) > xdd_safe  ||
      fabs(cart_x_plane[1]) > x_safe[3]      || 
      fabs(vis_states_filtered[1]) >  x_safe[1]  ){

    // printf("[DBG]: @within_safety_set() not_safe\n");

    // Debugging (remove)
    // #ifdef __XENO__
    //   rt_printf("** Coming back to the safe set of gains...\n");
    // #else
    //   printf("** Coming back to the safe set of gains...\n");
    // #endif

    // Load the safe set of gains:
    vec_equal(F_safe,F);

    reset_ = TRUE;

    // Update the switch-to-safety flag:
    sw2safe = TRUE;

  }
  else{
    // printf("[DBG]: @within_safety_set() safe\n");
    reset_ = FALSE;
    hold_ = FALSE;
  }

  // Do this when safety is triggered, but do it only once
  if( reset_ && !hold_ )
  {
    // Reset the integrator:
    // int_state = 0.0;

    // printf("[DBG]: @within_safety_set() print_only_once\n");

    recovering_from_unstability = TRUE;

    // Stop collecting data and save it, but only if an experiment was running:
    if(record_data && exp_running){ // Redundant: exp_running can only be TRUE if record_data is TRUE. Left record_data for clarity
      exp_running     = FALSE;
      save_in_next    = TRUE;
    }

    // printf("[DBG]: @within_safety_set() print_this 1\n");

    /***** Print on screen: *****/
    #ifdef __XENO__
      rt_printf(RED "\nUNSTABILITY DETECTED !!!\n");
      if( fabs(cart_xdd_plane_filt) > xdd_safe )
        rt_printf("** Switch triggered because End-effector acceleration is out\n");

      if ( fabs(cart_x_plane[1]) > x_safe[3] )
        rt_printf("** Switch triggered because End-effector position is out\n");

      if ( fabs(vis_states_filtered[1]) >  x_safe[1] )
        rt_printf("** Switch triggered because Pole angle is out\n");

        rt_printf("** Going back to the safe set of gains...\n");
        rt_printf(BLUE "** Safety check disabled until the transient of %i seconds is gone\n",transient_time_ms/1000);
        rt_printf("---\n" BLACK);
    #else
      printf(RED "\nUNSTABILITY DETECTED !!!\n");
      if( fabs(cart_xdd_plane_filt) > xdd_safe )
        printf(RED "** Switch triggered because End-effector acceleration is out\n");

      if ( fabs(cart_x_plane[1]) > x_safe[3] )
        printf(RED "** Switch triggered because End-effector position is out\n");

      if ( fabs(vis_states_filtered[1]) >  x_safe[1] )
        printf(RED "** Switch triggered because Pole angle is out\n");

        printf("** Going back to the safe set of gains...\n");
        printf(BLUE "** Safety check disabled until the transient of %i seconds is gone\n",transient_time_ms/1000);
        printf("---\n" BLACK);
    #endif

    printPrompt_color();

    // We hold this variable TRUE until the states are back into the safe region
    hold_ = TRUE;
  }

  return TRUE;
}

static int
within_joint_limits(void)
{
  int i=0;

  // Set all target joints to zero. Exceptions are set seperately afterwards
  for (i=1; i<=7; ++i)
  {
    // Joint limits have been set after offset subtraction
    if( joint_state[i].th > joint_range[i][MAX_THETA] || joint_state[i].th < joint_range[i][MIN_THETA]){
      freeze();
      printf("Joint %i is out of the limits\n",i);
      if(record_data){
        stopcd();
        sendCommandLineCmd("saveData");
      }
      return FALSE;
    }
  }

return TRUE;
}

static int
within_task_space_limits(void)
{

  // We check if the robot's end-effector gets out of the "box"
  if ( !inside_the_box() )
  {
    freeze();
    printf("The robot's end-effector space limits have been reached\n");
    // printf("cart_x_plane[1] = %f\n",cart_x_plane[1]);
    // printf("cart_x_plane[2] = %f\n",cart_x_plane[2]);
    // printcart_x_in_plane[3] = %f\n",cart_x_plane[3]);
    if(record_data){
      stopcd();
      sendCommandLineCmd("saveData");
    }
    return FALSE;
  }
  else
    return TRUE;
}

static int
inside_the_box(void)
{
  // We check if the robot's end-effector gets out of the "box":
  if (  fabs(cart_x_plane[1]) > box_length_thres || 
        fabs(cart_x_plane[2]) > box_width_thres  ||
        fabs(cart_x_plane[3]) > box_height_thres  )
  {
    return FALSE;
  }
  else
    return TRUE;

}


// static int
// new_meas_simulation_old(void)
// {
//   ++global_counter;

//   ++vision_counter;

//   angle_visualization = pole.theta;

//   // Obtaining a new measurement:
//   if ( vision_counter == vision_Ts ){

//     // Getting a new measurement:
//     if (integral_action)
//       angle_groundtruth = get_new_measurement() + meas_offset;
//     else
//       angle_groundtruth = get_new_measurement() + meas_offset;
//       // angle_groundtruth = get_new_measurement();

//     // We restart the counter for the kinect's loop:
//     vision_counter = 0;
//   }

//   ind = ind % len + 1;
//   store_meas[ind] = angle_groundtruth;

//   if ( global_counter >= vision_Ts + MEAS_LATENCY )
//     ind_del = ind_del % len + 1;

//   if ( current_angle_measured != store_meas[ind_del] ){
//     current_angle_measured = store_meas[ind_del];
//     return TRUE;
//   }
//   else
//     return FALSE;
// }

static int
new_meas_simulation(void)
{

  ++vision_counter;

  angle_visualization = pole.theta;

  // Obtaining a new measurement:
  if ( vision_counter == vision_Ts ){

    // Getting a new measurement:
    angle_groundtruth = get_new_measurement();// + meas_offset;

    // We restart the counter for the kinect's loop:
    vision_counter = 0;
  }

  current_angle_measured = sensor->add_delay(angle_groundtruth);
}


static int
new_meas_realtime(void)
{
  // printf("[DBG]: @new_meas_realtime() 1\n");
  double angle_from_ROS_vicon = pole_angle_getter->get();
  // printf("[DBG]: @new_meas_realtime() 2\n");
  if( current_angle_measured != sign_conv*(angle_from_ROS_vicon - initial_angle_offset) ){
    
    // Sign convention: initialized in parameters_initialization()
    current_angle_measured    = sign_conv*( angle_from_ROS_vicon - initial_angle_offset );
    angle_visualization       = current_angle_measured;
    return TRUE;
  }
  else
    return FALSE;
}

static void 
joint_offsets_initialization(void)
{
  // Joint Offsets (look at /apolloUser/config/SensorOffset.cf)
  // Out-of-the-joint-limits posture (estimated by hand):
  shoulder_offset = -0.2;
  HR_offset       = HR_offset_user*M_PI/180;
  elbow_offset    = M_PI_2 - (HR_offset/(M_PI_2)*(-shoulder_offset+0.1)) + shoulder_offset + 0.05;
  WR_offset       = -shoulder_offset - 0.05;
  wrist_offset    = -0.785;
  hand_offset     = -0.1;
  fingers_offset  = 2.43;
  // Hardcoded: 0, -0.3, 0.523599, 1.320796, 0.15, 0, 0
}

static int
stop_criteria(void)
{
  task_time = task_servo_time - start_time;

  // STOP CRITERIA: Finish the task when the total time is consumed:
  if ( task_time > total_tt ){
    freeze();
    printf("\n\n    Task is finisehd!\n\n");
    if(record_data){
      stopcd();
      sendCommandLineCmd("saveData");
    }
    return FALSE;
  }
  else
    return TRUE;
}

static int
check_elapsed_time_current_experiment(void)
{
  double experiment_time = task_servo_time - start_time_exp;

  // Stop collecting data when the total time is consumed:
  if ( experiment_time > total_et )
  {

    exp_running = FALSE;
    
    // We reload the safe set of gains:
    if ( back_to_safety_after_experiments )
      vec_equal(F_safe,F);

    // Verbosity:
    #ifdef __XENO__
      rt_printf(BLUE "\n");
      rt_printf("** Experiment finished after %f s. Saving data... \n",experiment_time);
      if ( back_to_safety_after_experiments )
        rt_printf("** Going back to the safe set of gains...\n" BLACK);
      else
        rt_printf("** Keeping the same set of exploration gains...\n" BLACK);
    #else
      printf(BLUE "\n");
      printf("** Experiment finished after %f s. Saving data... \n",experiment_time);
      if ( back_to_safety_after_experiments )
        printf("** Going back to the safe set of gains...\n" BLACK);
      else
        printf("** Keeping the same set of exploration gains...\n" BLACK);
    #endif

    if (record_data){
      stopcd();
      sendCommandLineCmd("saveData");
    }
  }

  return TRUE;
}


// static void
// get_filter_parameters(int which_filter)
// {

//   switch ( which_filter ) {

//     // Pre-processing filter parameters:
//     case 0:
//       bw_filter.a[0] = 0.0; bw_filter.a[1] = 0.0; bw_filter.a[2] = 0.0;
//       bw_filter.b[0] = 0.0; bw_filter.b[1] = 0.0; bw_filter.b[2] = 0.0;
//       break;
//     case 1: // 2nd order Butterworth filter, with fc = 1Hz:
//       bw_filter.a[0] =  1.000000000000000; bw_filter.a[1] = -1.991114292201654; bw_filter.a[2] =  0.991153595868935;
//       bw_filter.b[0] =  9.825916820471736e-06; bw_filter.b[1] =  1.965183364094347e-05; bw_filter.b[2] =  9.825916820471736e-06;
//       break;
//     case 2: // 2nd order Butterworth filter, with fc = 5Hz:
//       bw_filter.a[0] =  1.000000000000000; bw_filter.a[1] = -1.955578240315035; bw_filter.a[2] =  0.956543676511203;
//       bw_filter.b[0] =  2.413590490419615e-04; bw_filter.b[1] =  4.827180980839230e-04; bw_filter.b[2] =  2.413590490419615e-04;
//       break;
//     case 3: // 2nd order Butterworth filter, with fc = 10Hz:
//       bw_filter.a[0] =  1.000000000000000; bw_filter.a[1] = -1.911197067426073; bw_filter.a[2] =  0.914975834801434;
//       bw_filter.b[0] =  9.446918438401619e-04; bw_filter.b[1] =  0.001889383687680; bw_filter.b[2] =  9.446918438401619e-04;
//       break;
//     case 4: // 2nd order Butterworth filter, with fc = 15Hz:
//       bw_filter.a[0] =  1.000000000000000; bw_filter.a[1] = -1.866892279711715; bw_filter.a[2] =  0.875214548253684;
//       bw_filter.b[0] =  0.002080567135492; bw_filter.b[1] =  0.004161134270985; bw_filter.b[2] =  0.002080567135492;
//       break;
//     case 5: // 2nd order Butterworth filter, with fc = 20Hz:
//       bw_filter.a[0] =  1.000000000000000; bw_filter.a[1] = -1.822694925196308; bw_filter.a[2] =  0.837181651256023;
//       bw_filter.b[0] =  0.003621681514929; bw_filter.b[1] =  0.007243363029857; bw_filter.b[2] =  0.003621681514929;
//       break;
//   }
  
// };

static void
printPrompt_color(void)
{

  #ifdef __XENO__
    rt_printf(BLUE "");
  #else
    printf(BLUE "");
  #endif

  printPrompt();

  #ifdef __XENO__
    rt_printf(BLACK "");
  #else
    printf(BLACK "");
  #endif
}

extern "C" void
add_pole_balancing_apollo(void)
{
  static int firsttime = TRUE;
  int i;
  
  if (firsttime) {
    /* things that only need to be done once */
    firsttime = FALSE;
    
    cart    = my_vector(1,n_endeffs*6);
    ctarget = (SL_Cstate *) my_calloc(n_endeffs+1,sizeof(SL_Cstate),MY_STOP);
    cstatus = my_ivector(1,n_endeffs*6);
    target  = (SL_DJstate *)my_calloc(n_dofs+1,sizeof(SL_DJstate),MY_STOP);
    fthdd   = (Filter *)my_calloc(n_dofs+1,sizeof(Filter),MY_STOP);

    // Initialize the filters:
    init_filters();
    for (i=1; i<=n_dofs; ++i) 
      fthdd[i].cutoff = 5;

    addTask("Pole Balancing Task", init_pole_balancing_apollo, 
      run_pole_balancing_apollo, change_pole_balancing_apollo);
  }
}    



