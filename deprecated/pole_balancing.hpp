// Copyright (c) 2015 Max Planck Society

/*!@file
 * @author  Alonso Marco <alonso.marco@tuebingen.mpg.de>
 *
 * @date    2015-11-20
 *
 * @details 
 * Pole Balancing interface/library, to be called from
 * an external SL task.
 *
 */

#ifndef INCLUDE_POLE_BALANCING_H
#define INCLUDE_POLE_BALANCING_H

#include <cstdio>
#include "SL.h"
#include "SL_task_servo.h"
#include "SL_collect_data.h"
#include "SL_unix_common.h"
#include <cmath>

// Structure of user-defined options:
#ifndef USER_OPTS
#define USER_OPTS
#include "opts.hpp"
#endif

#define Norm(x,y)     ( sqrt( sqr(x) + sqr(y) ) )

#define N_CART 3

class CartesianState 
{
  public:
  double   x[N_CART];    /*!< Position [x,y,z] */
  double   xd[N_CART];   /*!< Velocity */
  double   xdd[N_CART];  /*!< Acceleration */
};



class PoleBalancing {

public:

  PoleBalancing(Options * opts);

  ~PoleBalancing();

  // int get_endeff_pos_in_plane(SL_Cstate * cart_state, 
  //                             Vector endeff_pos_plane);

  // int get_endeff_vel_in_plane(SL_Cstate * cart_state,
  //                             double * endeff_vel_plane);

  // int get_endeff_acc_in_plane(SL_Cstate * cart_state,
  //                             double * endeff_acc_plane);



  // PoleBalancing(Options * opts, std::shared_pointer<Robot> robot){
  //  this->robot = robot;
  // }

  // bool check_stuff(){
  //  this->robot.within_joint_limits();
  //  this->robot

  // }

  // bool within_joint_limits(){
  //  return this->robot.within_joint_limits();
  // }


  // Sends TRUE if it any of its joints reaches any of its limits
  int within_joint_space_limits(SL_Jstate * joint_state,
                                double joint_range[][4]);

  int within_time(double task_servo_time,
                  double start_time,
                  double * task_time);  

	// Freeze the robot the end-effector task space limits are reached. End-effector task 
	// space limits are defined as the boundaries of a 3D box centered on the initial position of 
	// the end-effector, dependent on its plane of movement:
	int within_task_space_limits(Vector endeff_pos_plane);

  // READ SENSORS:
    // int update_pole_readings(double & pole_angle); //get_new_measurement() update_measurements() new_measurement_available() new_meas_simulation() new_meas_realtime()
    // int update_endeff_readings(CartesianState endeffector_state_robot_frame, CartesianState & endeffector_state_local_frame);
    // int get_sensor_readings(double & pole_angle, CartesianState & endeffector_state_local_frame); //update_pole_readings() update_endeff_readings()

  // GET SYSTEM STATE:
  // Transform from robot's frame to local frame:
  void get_endeff_in_plane( CartesianState cart_base, 
                            CartesianState & cart_plane);

  // Integrator update:
  void get_integrator_state(double endeff_pos_plane_x, 
                            double int_state);

  // int observe_pole_states(double pole_angular_pos, 
                          // double pole_states[]); // compute_states() simulatePoleDynamics()

  // int get_system_states(CartesianState cart_plane, 
                        // double pole_states[], 
                        // double (&x)[]); // get_endeff_in_plane() observe_pole_states() get_integrator_state()

  // COMPUTE CONTROL:
  int get_LQR_control(Vector x, Vector & u); // detect_new_LQR() control_input_computation() saturate_signal()
  int get_robot_control(Vector u, CartesianState & cart, CartesianState & cart_target_state); // Maybe in Robot Control class ??? cartesian_target_computation()

protected:

private:
	Options * opts;

};

#endif /* INCLUDE_POLE_BALANCING_H */
