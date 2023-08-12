/*!=============================================================================
  ==============================================================================

  \file    pole_balancing_task.h

  \author  Alonso Marco
  \date    Apr. 2015

  ==============================================================================
  \remarks
  
  header file for pole_balancing_task.c
  
  ============================================================================*/

#ifndef _pole_balancing_task_
#define _pole_balancing_task_

// for shared ptr
#include <memory>

// SL general includes of system headers
#include "SL_system_headers.h"

/* private includes */
#include "SL.h"
#include "utility.h"
#include "SL_tasks.h"
#include "SL_task_servo.h"
#include "SL_collect_data.h"
#include "SL_kinematics.h"
#include "SL_dynamics.h"
#include "SL_unix_common.h"
#include "SL_user.h"
#include "SL_userGraphics.h"
#include "SL_filters.h"
#include "SL_common.h"
#include "utility_macros.h"
#include <stdlib.h>
#include "SL_filters.h"

// Defintion of pole data structure
#include "pole.h"

#ifdef __XENO__
#include <native/task.h>
#endif

/* local variables */
#define Norm(x,y)     ( sqrt( sqr(x) + sqr(y) ) )
#define STYLE_BOLD    "\033[1m"
#define STYLE_NO_BOLD "\033[22m"
#define RED           "\033[31m"
#define BLACK         "\033[30m"
#define WARN          STYLE_BOLD RED "[WARNING]: " BLACK STYLE_NO_BOLD
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */

// Name of the ROS node:
char NODE[50];

// Name of the ROS topics:
char SAFE_GAINS_TOPIC[50];
char SEARCH_GAINS_TOPIC[50];

// Name of the files:
char fname_safe[100];
char fname_search[100];

// Gravity constant:
static  double  g_n;

// SL time step:
static  double  time_step;

// Task flags:
static  int     record_data      = TRUE;

#ifdef __XENO__
  static  int     prepare          = TRUE;
#else
  static  int     prepare          = FALSE;
#endif

static  int     simulate_noise   = TRUE;
static  int     only_recording   = FALSE;
static  int 		train_init_angle = FALSE;
static  int     change_kal_par   = FALSE;
static  int     integral_action  = TRUE;
static  int     change_safety    = FALSE;
static  int     change_safe_set  = FALSE;

#ifdef __XENO__
static  int     using_head       = FALSE;
#else
static  int     using_head       = FALSE;
#endif
// static  int     double_rate      = FALSE;
static  int     control_change   = FALSE;
static  int     firsttime_;

// Changing the cartesian feedback control loop gains:
static int change_cart_gains = FALSE;

static  int     use_forg_factor  = FALSE;

// Which filter:
static  int     which_filter_pole = 3;
static  int     which_filter_xdd  = 5;
static  int     which_filter_xd   = 1;
static  int     which_filter_x    = 2;
static  int     ind_vel;

// Which pole:
static  int     long_pole    = TRUE;

// Save in the next iteration:
static  int     save_in_next = FALSE;

// Unstability counter:
static  int  unstability_timer_ms;

// Maximum allowed transient time:
static  int  transient_time_ms = 2000;

static  int   recovering_from_unstability;

static  int   back_to_safety_after_experiments = FALSE;

// // Filter structure:
// typedef struct{
//   double x[3];
//   double y[3];
//   double a[3];
//   double b[3];
// } Butter_filt;

// Butter_filt angle_filt, vel_filt, acc_filt;

// struct{
// 	double a[3];
// 	double b[3];
// }bw_filter;

// Initialize pole structure object:
Pole_physics pole;

// Sign convention:
static double   sign_conv;

// Timers:
static  double  start_time;
static  double  start_time_exp;
static  double  task_time;
static  double  elapsed_time_run;

// Total task time:
static double total_tt = 36000;

#ifdef __XENO__
  static double total_et = 20;
#else
  static double total_et = 20;
#endif

static int    exp_running = FALSE;

// Variables for handling the latency simulation:
static  int     len;
static  Vector  store_meas;
static  int     ind;
static  int     ind_del;
static  double  angle_groundtruth;
static  int     vision_Ts;
static  int     vision_counter;
static  int     global_counter;

// Number of time steps needed for reading a new angle. This number simulates the delay of the image processing. 
// It is implemented at the SL loop rate. Minimum is MEAS_LATENCY = 1. Do not set MEAS_LATENCY = 0
static int      MEAS_LATENCY = 30;
static int      CART_LATENCY = 0;

// We predict the current state x(k|k-mu) up to an horizon defined by mu = PREDICTION_HORIZON
static int      PREDICTION_HORIZON = 1;

// Joint Offsets (look at /apolloUser/config/SensorOffset.cf)
static double   wrist_offset;
static double   hand_offset;
static double   shoulder_offset;
static double   elbow_offset;
static double   HR_offset;
static double   HR_offset_user = 30;
static double   fingers_offset;
static double   WR_offset;

// Head states:
static double   theta_pan;
static double   theta_tilt;

// Control inputs:
// static double   u_del;
static double   u_to_be_sent;
static double   u_send;

// Integrator states:
static Vector   joint_error;
static Vector   controller_gain_th;
static Vector   controller_gain_thd;
static Vector   controller_gain_int;
static Vector   u_i;
static Vector   u_pd;

// Trapezoidal integration variables:
static double   pos_prev_x_SL;
static double   pos_prev_y_SL;
static double   vel_prev_x_SL;
static double   vel_prev_y_SL;
static double   acc_prev_x_SL;
static double   acc_prev_y_SL;

// Solving the shifting issue:
static double   K_shift;

// Cartesian feedback control variables:
static Vector   ufb_pose_KP;
static Vector   ufb_pose_KD;
static Vector   ufb_pose_KI;
static Vector 	KP_pose;
static Vector 	KI_pose;
static Vector   KD_pose;
static Vector   int_state_pose;
static Matrix   integrator_window;
static int      ind_win;
static int 		  win_length;

// Control saturation:
static double	u_sat = 3;

// Cart offset:
static double   cart_x_offset;
static double   cart_y_offset;
static double   cart_z_offset;

static double   cart_des_x_offset;
static double   cart_des_y_offset;
static double   cart_des_z_offset;

// End-effector measured position, defined from the starting point, in the robot's frame:
static Vector   cart_x_base;

// End-effector measured velocity, defined from the starting point, in the robot's frame:
static Vector   cart_xd_base;

// End-effector measured acceleration, defined from the starting point, in the robot's frame:
static Vector   cart_xdd_base;

  // End-effector position defined from the starting point, in the plane of movement frame's coordiantes:
static Vector   cart_x_plane;

// End-effector desired position defined from the starting point, in the plane of movement frame's coordiantes:
static Vector   cart_des_x_plane;

// Position and velocity of the end-effector in the plane of movement:
static double   cart_xd_plane;
static double   cart_xdd_plane;
static double   cart_xdd_plane_filt;

static double   cart_tar_x_plane;
static double   cart_tar_xd_plane;
static double   cart_tar_xdd_plane;

static double   cart_des_xd_plane;
static double   cart_des_xdd_plane;

// Pole angle offset introduced by the user in simulation:
static double   initial_angle = 0.0;

// Initial offset averaged from a previous training phase to a real-time experiment:
static double   initial_angle_offset;

// Measured angle:
static double   current_angle_measured;

// Angle for visualizing the pole in SL (both, real time and simulation):
static double   angle_visualization;

// Vertical plane of movement defined by its angle with respect to the XZ plane of the world frame:
static double   plane_angle;

// Auxiliar variables for the coordinates transformation at inside_the_box():
static Vector 	rotation_angles;
static Matrix 	rot_mat_;

// Safety thresholds:
static double  	box_width_thres;
static double  	box_length_thres = 0.30;
static double   box_height_thres;

// Safety set for states and controller:
static Vector   x_safe;
static double   xdd_safe;
static int      sw2safe;
static int      reset_;
static int      hold_;

// Used to transform from endeffector-frame to base-frame (simulation only)
static Matrix   rot_matrix;
static Vector   tmp_vector;
static Vector   trial_vector;

// Control gain:
static Vector   F;
static Vector   F_search;
static Vector   F_safe;
static Vector   F_pred;

// Entropy Search iteration counter:
static int      ES_iter_counter = 0;

// Kalman filter gain:
static Vector   K;

// Kalman filter auxiliar variables:
static Vector   Bru;
static Vector   x_pred;
static Vector   Arx_pred;
static Matrix   aux_mat;
static Vector   aux_vec;
static double   aux;
static Matrix   Id;

// // System variables:
// static double   a1, a2, a3;
// static Matrix   Ar;
// static Vector   Br;
// static Vector   Cr;

// Integrator:
static double   int_state;

// Measurement offset (to be considered only when the user decides to include an integral action):
static double   meas_offset;

// Kalman filter tuning parameters::
static double   q, r;
static Matrix   Q;
static double   R;

// System states:
static Vector   vis_state_p;    //Posterior  state x(k+1-mu|k-mu)
static Vector   vis_state_m;    //Prior      state x(k+1-mu|k+1-mu)
static Vector   vis_states; // Predicted state x(k+1|k-mu)
static Vector   u_queue;

static double  uptonow_measured_angle;
static double  uptonow_velocity;
static Vector  vis_states_filtered;
static double  filtered_angle;
static double  previous_angle;

// Variance matrices:
static Matrix   P_m; //Posterior  variance matrix P_m(k+1-mu|k-mu)
static Matrix   P_p; //Prior      variance matrix P_p(k+1-mu|k+1-mu)

// Measurement noise:
static double   sigma_n;

// SL variables:
static double     *cart;
static int        *cstatus;
static int        *cstatus_pose;
static SL_Cstate  *ctarget;
static SL_DJstate *target;
static Filter     *fthdd;

//Initial position (N_DOFS is defined in SL_user.h)
static SL_DJstate  initial_position[N_DOFS+1]; 

// Automatic switch when working with the real robot:
#ifndef __XENO__
static int        sim_flag = TRUE;
#else
static int        sim_flag = FALSE;
#endif

/* local functions */
// Pointer that switchs between the two functions that search for new measurements:
static int  (*new_measurement_available)(void);

// Functions that check out if there are new measurements available:
static int  new_meas_simulation(void);
static int  new_meas_realtime(void);
// static int  new_meas_simulation_old(void);

// // Running the kalman filter:
// static void running_kalman_filter(void);
 
 // Task functions:
static int  init_pole_balancing_apollo(void);
static int  run_pole_balancing_apollo(void);
static int  change_pole_balancing_apollo(void);
extern "C" void
add_pole_balancing_apollo(void);

// Functions for the pole simulation:
static int        visualizePole(double angle_measured_);
static void       simulatePoleDynamics(void);
static SL_Cstate  computePositionOnPole(double theta_2, double dist_to_pole_end);

// Obtain a new measurement from the vision system:
static double get_new_measurement(void);

// KF functions:
static int  control_gain_initialization(void);

// Butterworh filter:
// static double apply_filter(double raw_angle);

// Update the filter parameters:
// static void   get_filter_parameters(int which_filter);

// Update measurements:
static void  update_measurements(void);

// Computing the pole's angular velocity:
static void  compute_states(void);

static void measure_endeffector_states(void);

// Reading controller gains:
static int  read_controller_gains(char *fname, double *control_gain);

// Integrator update:
static void integrator_update(void);

// Control computation:
static double control_input_computation(Vector vis_state_BG);
static void   saturate_signal(double *sig,double max_value);
static int    send_control(void);
static void cartesian_target_computation(void);

// Cartesian feedback control contribution:
static int  cartesian_feedback_control_loop(void);

// Updating the PID gains:
static void update_PID_gains(void);

// Initialization of the joint offsets:
static void joint_offsets_initialization(void);

// Freeze the robot if it any of its joints reach any of its limits
static int  within_joint_limits(void);

// Freeze the robot the end-effector task space limits are reached:
static int  within_task_space_limits(void);

// If we get out of the safety set, defined in the state space, we update with the safe the LQR gains:
static int within_safety_set(void);

// Wait some time until the transient is gone.
static void wait_until_transient_gone();

// Check the elapsed experiment time, and save data when total_tt reached:
static int  check_elapsed_time_current_experiment(void);

// Logic for updating the LQR gains only when the file LQR_gains_search.cf externally changes:
static int detect_new_LQR(void);

// We start a new experiment:
static int start_new_experiment(void);

// End-effector task space limits are defined as the boundaries of a 3D box centered on the initial position of
// the end-effector, dependent on its plane of movement:
static int  inside_the_box(void);

// All the joints that do not contribute to the task are set still to the initial position
static int  freeze_inactive_joints(SL_DJstate[]);

// Other functions:
static void set_zero_dynamics(void);
static int  move_to_initial_position(void);
static int  parameters_initialization(void);
static int  stop_criteria(void);

// Print promt with color:
static void printPrompt_color(void);

// Head functions:
static int  move_head_to_initial_position(void);
static int  move_head(void);
static void head_target_computation(void);
static int  head_inverse_kinematics(void);

/* global variables */
/* global functions */

#endif // _pole_balancing_task_
