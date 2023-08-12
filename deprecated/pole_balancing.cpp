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

#include "pole_balancing_apollo/pole_balancing.hpp"


PoleBalancing::PoleBalancing(Options * opts)
{
  // Allocate memory for the pointer to the user-defined options structure:
  this->opts = new Options;

  // Initialize all pointers inside the structure:
  // this->opts->endeff_offset = my_vector(1,N_CART);
  // vec_zero(this->opts->endeff_offset);
  this->opts->endeff_offset = new double[n_cart];

  this->opts->rot_mat_ = my_matrix(1,n_cart,1,n_cart);
  mat_zero(this->opts->rot_mat_);

  // Fill the structure, to be used in this scope
    // Non-pointer variables:
    this->opts->record_data       = opts->record_data;
    this->opts->total_tt          = opts->total_tt;
    this->opts->box_width_thres   = opts->box_width_thres;
    this->opts->box_height_thres  = opts->box_height_thres;
    this->opts->box_length_thres  = opts->box_length_thres;
    this->opts->time_step         = opts->time_step;

    // We copy the memory address from one pointer to the other's
    this->opts->endeff_offset     = opts->endeff_offset;
    this->opts->rot_mat_          = opts->rot_mat_;

}


PoleBalancing::~PoleBalancing()
{
  
}

int PoleBalancing::within_joint_space_limits(SL_Jstate * joint_state,
                                             double joint_range[][4]){

  for (int i=1; i<=7; ++i)
  {
    if( joint_state[i].th > joint_range[i][MAX_THETA] || joint_state[i].th < joint_range[i][MIN_THETA]){
      #ifdef __XENO__
        rt_printf("Joint %i is out of the limits\n",i);
        rt_printf("\n\n    Task is finisehd!\n\n");
      #else
        printf("Joint %i is out of the limits\n",i);
        printf("\n\n    Task is finisehd!\n\n");
      #endif

      freeze();
      if(opts->record_data){
        stopcd();
        sendCommandLineCmd("saveData");
      }

      return FALSE;
    }
  }

  return TRUE;

}


int PoleBalancing::within_time(double task_servo_time,
                               double start_time,
                               double * task_time){

  *task_time = task_servo_time - start_time;

  // STOP CRITERIA: Finish the task when the total time is consumed:
  if ( *task_time > opts->total_tt ){
    freeze();
    #ifdef __XENO__
      rt_printf("\n\n    Task is finisehd!\n\n");
    #else
      printf("\n\n    Task is finisehd!\n\n");
    #endif
    if(opts->record_data){
      stopcd();
      sendCommandLineCmd("saveData");
    }
    return FALSE;
  }
  else
    return TRUE;
}

// int PoleBalancing::get_endeff_pos_in_plane( SL_Cstate * cart_state,
//                                             Vector endeff_pos_plane) {

//   // End-effector position in the robot frame's coordiantes: cart_state[1].x[...]

//   // End-effector position defined from the starting point, in the local frame's coordiantes:
//   Vector cart_x_local = my_vector(1,N_CART);
//   vec_zero(cart_x_local); 

//   cart_x_local[1] = cart_state[1].x[_X_] - opts->endeff_offset[0];
//   cart_x_local[2] = cart_state[1].x[_Y_] - opts->endeff_offset[1];
//   cart_x_local[3] = cart_state[1].x[_Z_] - opts->endeff_offset[2];

//   // End-effector position defined from the stang point, in the plane of movement frame's coordiantes:
//   mat_vec_mult(opts->rot_mat_,cart_x_local,endeff_pos_plane);

//   return TRUE;
// }

// int PoleBalancing::get_endeff_vel_in_plane( SL_Cstate * cart_state,
//                                             double * endeff_vel_plane) {

//   // Offsets for the cart position and the pole angle are removed (both due to task needs and for recording)
//   *endeff_vel_plane = sign(cart_state[1].xd[_X_])*Norm(cart_state[1].xd[_X_],cart_state[1].xd[_Y_]);

//   return TRUE;
// }

// int PoleBalancing::get_endeff_acc_in_plane( SL_Cstate * cart_state,
//                                             double * endeff_acc_plane) {

//   // Offsets for the cart position and the pole angle are removed (both due to task needs and for recording)
//   *endeff_acc_plane = sign(cart_state[1].xdd[_X_])*Norm(cart_state[1].xdd[_X_],cart_state[1].xdd[_Y_]);

//   return TRUE;
// }

int PoleBalancing::within_task_space_limits(Vector endeff_pos_plane)
{
  bool in_the_box;

  // We check if the robot's end-effector gets out of the "box":
  if (  fabs(endeff_pos_plane[_X_]) > opts->box_length_thres || 
        fabs(endeff_pos_plane[_Y_]) > opts->box_width_thres  || 
        fabs(endeff_pos_plane[_Z_]) > opts->box_height_thres  ){
    in_the_box = false;
  }
  else
    in_the_box = true;

  // We check if the robot's end-effector gets out of the "box"
  if ( !in_the_box )
  {
    freeze();
    printf("The robot's end-effector space limits have been reached\n");
    if(opts->record_data){
      stopcd();
      sendCommandLineCmd("saveData");
    }
    return FALSE;
  }
  else
    return TRUE;
}

void PoleBalancing::get_endeff_in_plane( CartesianState cart_base,
                                         CartesianState & cart_plane ){
  /* Position */
    // End-effector position in the robot frame's coordiantes: cart_base[1].x[...]
    // End-effector position defined from the starting point, in the local frame's coordiantes:
    Vector cart_x_local = my_vector(1,N_CART); vec_zero(cart_x_local); 
    Vector cart_x_plane = my_vector(1,N_CART); vec_zero(cart_x_plane); 
    cart_x_local[_X_] = cart_base.x[_X_] - opts->endeff_offset[0];
    cart_x_local[_Y_] = cart_base.x[_Y_] - opts->endeff_offset[1];
    cart_x_local[_Z_] = cart_base.x[_Z_] - opts->endeff_offset[2];
    // End-effector position defined from the stang point, in the plane of movement frame's coordiantes:
    mat_vec_mult(opts->rot_mat_,cart_x_local,cart_x_plane);
    cart_plane.x[_X_] = cart_x_plane[_X_];
    cart_plane.x[_Y_] = cart_x_plane[_Y_];
    cart_plane.x[_Z_] = cart_x_plane[_Z_];

  /* Velocity */
    // Offsets for the cart position and the pole angle are removed (both due to task needs and for recording)
    cart_plane.xd[_X_] = sign(cart_base.xd[_X_])*Norm(cart_base.xd[_X_],cart_base.xd[_Y_]);
    cart_plane.xd[_Y_] = 0;
    cart_plane.xd[_Z_] = 0;

  /* Acceleration */
    // Offsets for the cart position and the pole angle are removed (both due to task needs and for recording)
    cart_plane.xdd[_X_] = sign(cart_base.xdd[_X_])*Norm(cart_base.xdd[_X_],cart_base.xdd[_Y_]);
    cart_plane.xdd[_Y_] = 0;
    cart_plane.xdd[_Z_] = 0;
}

void PoleBalancing::get_integrator_state(double endeff_pos_plane_x, double int_state){
  int_state += opts->time_step*endeff_pos_plane_x;
}