// Copyright (c) 2015 Max Planck Society

/*!@file
 * @author  Alonso Marco <alonso.marco@tuebingen.mpg.de>
 *
 * @date    2017-01-12
 *
 * @details 
 * Pole Balancing interface/library, to be called from
 * an external SL task.
 *
 */

#include "pole_balancing_apollo/estimate_states.hpp"

EstimateStatesPole::EstimateStatesPole( double plane_angle, double time_step,
                                        int which_filter_pole, int which_filter_xdd,
                                        int integral_action, std::vector<double> endeff_offset,
                                        std::vector<double> state_space_box_limits,
                                        double endeff_acc_limits) : endeff_offset(endeff_offset) {
  int i;

  // Cartesian state of the end-effector in the local coordinates:
  Tools::vec_zero(this->cart_plane.x);
  Tools::vec_zero(this->cart_plane.xd);
  Tools::vec_zero(this->cart_plane.xdd);

  // Generate rotation matrix:
  // We define the rotation matrix from the local frame to the "plane of movement" frame:
  double * rotation_angles = new double[N_CART];
  rotation_angles[_x_]  = 0;
  rotation_angles[_y_]  = 0;
  rotation_angles[_z_]  = plane_angle;
  Tools::eulerToRotMat(rotation_angles,this->rot_matrix);
  delete rotation_angles;

  // Initialize:
  this->int_state = 0;
  this->pos_old = 0;
  this->time_step = time_step;
  this->integral_action = integral_action;
  this->states.resize(N_STATES,0.0);

  // Initialize filters:
  bfilter_angle_pos = std::make_shared<ButterFilt>(which_filter_pole);
  bfilter_angle_vel = std::make_shared<ButterFilt>(which_filter_pole);
  bfilter_xacc      = std::make_shared<ButterFilt>(which_filter_xdd);

  // End-effector filtered acceleration:
  this->endeff_acc_filtered = 0.0;

  // Safety:
  this->state_space_box_limits = state_space_box_limits;
  this->endeff_acc_limits = endeff_acc_limits;

  // Differentiation:
  this->signal_treatment = std::make_shared<Tools::SignalTreatment>(this->time_step);

  // Verbosity:
  this->verbosity.initialize("Message has to be replaced",10);

}

EstimateStatesPole::~EstimateStatesPole() {}

/* Public functions */
void
EstimateStatesPole::update_states(CartesianState endeffector_state_measured, double measured_pole_angle)
{

  // Extract measurements:
  KineticState pole_state;
  pole_state.th   = measured_pole_angle;
  pole_state.thd  = this->signal_treatment->differentiate(pole_state.th);
  CartesianState cart_base;
  for(int i=0;i<3;++i){
    cart_base.x[i] = endeffector_state_measured.x[i];
    cart_base.xd[i] = endeffector_state_measured.xd[i];
    cart_base.xdd[i] = endeffector_state_measured.xdd[i];
  }
  
  // Update states:
  this->states[PHI]   = this->bfilter_angle_pos->apply_filter(pole_state.th);
  this->states[PHID]  = this->bfilter_angle_vel->apply_filter(pole_state.thd);
  this->cart_plane    = transform_world2plane(cart_base);
  this->states[S]     = this->cart_plane.x[_x_];
  this->states[SD]    = this->cart_plane.xd[_x_];
  if(this->integral_action)
    this->states[INT_]   = this->update_integrator_state(this->states[S]);
  else
    this->states[INT_] = 0.0;

  // Update also the filtered acceleration of the end-effector in the plane coordinates:
  this->endeff_acc_filtered = this->bfilter_xacc->apply_filter(this->cart_plane.xdd[_x_]);

}

void
EstimateStatesPole::get_states(std::vector<double> & states){

  states = this->states;
}

void 
EstimateStatesPole::get_endeff_pos_in_plane(std::vector<double> & endeff_pos){

  for(int i=0;i<3;++i)
    endeff_pos[i] = this->cart_plane.x[i];

}

double EstimateStatesPole::differentiate(double pos)
{
  // Compute velocity:
  double vel = ( pos - this->pos_old ) / this->time_step;

  // Update old position:
  this->pos_old = pos;

  return vel;
}

CartesianState EstimateStatesPole::transform_world2plane(CartesianState cart_base){

  int i,j;

  // Initialize cart_plane
  CartesianState cart_plane;
  Tools::vec_zero(cart_plane.x);
  Tools::vec_zero(cart_plane.xd);
  Tools::vec_zero(cart_plane.xdd);

  /* Position */
    // End-effector position defined from the stang point, in the plane of movement frame's coordiantes:
    // First, reset cart_plane:
    for(i=0; i<N_CART ; ++i)
     for(j=0; j<N_CART ; ++j)
       cart_plane.x[i] += this->rot_matrix[i][j] * ( cart_base.x[j] - this->endeff_offset[j] );

  /* Velocity */
    // Offsets for the cart position and the pole angle are removed (both due to task needs and for recording)
    cart_plane.xd[_x_] = Tools::sign(cart_base.xd[_x_])*Norm(cart_base.xd[_x_],cart_base.xd[_y_]);

  /* Acceleration */
    // Offsets for the cart position and the pole angle are removed (both due to task needs and for recording)
    cart_plane.xdd[_x_] = Tools::sign(cart_base.xdd[_x_])*Norm(cart_base.xdd[_x_],cart_base.xdd[_y_]);

    return cart_plane;

}

double EstimateStatesPole::update_integrator_state(double endeff_pos_plane_x){
  return this->int_state += this->time_step * endeff_pos_plane_x;
}

bool EstimateStatesPole::within_state_space_limits(){

  // Check state-space positions:
  for(int i=0;i<5;++i){
    if(this->state_space_box_limits[i] != 0.0){
      if( fabs(this->states[i]) > this->state_space_box_limits[i] ){
        sprintf(this->buff,"State x[%i] is out of limits\n",i);
        this->verbosity.print(this->buff);
        return false;
      }
    }
  }

  // Check end-effector acceleration:
  if(this->endeff_acc_filtered > this->endeff_acc_limits){
    printf("Filtered end-effector acceleration out of limits\n");
    return false;
  }

  return true;
}