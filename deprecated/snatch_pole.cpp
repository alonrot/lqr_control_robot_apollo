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

#include "snatch_pole.hpp"

SnatchPoleVicon::SnatchPoleVicon(std::shared_ptr<ParametersInitialization> par)
{

  int i;

  // Allocate memory for the pointer to the user-defined options structure:
  this->par = par;

  // Object for getting the pole angle:
  this->pole_angle_getter.reset(new RosRT_pole_angle());

  // Object for artificially adding delay, noise or different acquisition rate:
  this->angle_sensor.reset(new FakeSensor(this->par->sigma_n,this->par->vision_Ts,this->par->VISION_LATENCY));

  // Simulated pole kinetic states:
  simu_pole.th = 0.0;
  simu_pole.thd = 0.0;
  simu_pole_new.th = 0.0;
  simu_pole_new.thd = 0.0;

  // Relevant variables:
  this->measured_pole_angle       = 0.0;
  this->visualization_pole_angle  = 0.0;
  for(i=0;i<N_CART;++i){
    this->endeffector_state_measured.x[i] = 0.0;
    this->endeffector_state_measured.xd[i] = 0.0;
    this->endeffector_state_measured.xdd[i] = 0.0;
    this->endeffector_state_sensor.x[i].reset(new FakeSensor(0.0,0,this->par->ENDEFF_LATENCY));
    this->endeffector_state_sensor.xd[i].reset(new FakeSensor(0.0,0,this->par->ENDEFF_LATENCY));
    this->endeffector_state_sensor.xdd[i].reset(new FakeSensor(0.0,0,this->par->ENDEFF_LATENCY));
  }

  // Gaussian noise:
  this->rnd_generator = std::mt19937(this->rd());
  this->gaussian_noise = std::normal_distribution<double>(0.0, this->par->sigma_n);



}

SnatchPoleVicon::~SnatchPoleVicon() {}

/* Public functions */
void
SnatchPoleVicon::update(const CartesianState endeffector_state_raw,
                        const double u_send){

  update_endeff_readings(endeffector_state_raw);
  update_pole_readings(u_send);
}

double SnatchPoleVicon::get_measured_pole_angle(void){
  return this->measured_pole_angle;
}

double SnatchPoleVicon::get_visualization_pole_angle(void){
  return this->visualization_pole_angle;
}

CartesianState SnatchPoleVicon::get_measured_endeffector(void){
  return this->endeffector_state_measured;
}

/* Private functions */
void
SnatchPoleVicon::update_endeff_readings(const CartesianState endeffector_state_raw){

  int i;

  // Copy over:
  for(i=0; i<N_CART ; ++i){
    this->endeffector_state_measured.x[i]   = this->endeffector_state_sensor.x[i]->measure(endeffector_state_raw.x[i]);
    this->endeffector_state_measured.xd[i]  = this->endeffector_state_sensor.xd[i]->measure(endeffector_state_raw.xd[i]);
    this->endeffector_state_measured.xdd[i] = this->endeffector_state_sensor.xdd[i]->measure(endeffector_state_raw.xdd[i]);
  }

}

void
SnatchPoleVicon::update_pole_readings(const double u_send)
{
  // Select the real pole or the simulated pole, depending on the user choice:
  if (this->par->sim_flag)
    from_simulated_pole(u_send);
  else
    from_real_pole();
}

void
SnatchPoleVicon::from_simulated_pole(const double u_send)
{

  /* Simulate pole dynamics */
    // Forward Euler integration:
    this->simu_pole_new.th = this->simu_pole.th + this->par->time_step * this->simu_pole.thd;
    this->simu_pole_new.thd = this->simu_pole.thd + this->par->time_step * (
                                        this->par->pole_para.mass * this->par->g_n * this->par->pole_para.center_of_mass / this->par->pole_para.inertia * sin(this->simu_pole.th)
                                        - this->par->pole_para.mass * this->par->pole_para.center_of_mass * u_send / this->par->pole_para.inertia * cos(this->simu_pole.th)
                                        - this->par->pole_para.damping * this->simu_pole.thd / this->par->pole_para.inertia);

    // Write back new values in simu_pole:
    this->simu_pole.th  = this->simu_pole_new.th;
    this->simu_pole.thd = this->simu_pole_new.thd;

  /* Fake noise, delay and acquisition rate */ 
    this->measured_pole_angle = this->angle_sensor->measure(this->simu_pole.th);
    this->visualization_pole_angle = this->simu_pole.th;

    printf("measured_pole_angle = %f\n",this->measured_pole_angle);
    printf("visualization_pole_angle = %f\n",this->visualization_pole_angle);
    printf("\n");
}

void
SnatchPoleVicon::from_real_pole(void)
{

  double angle_from_vision_system = this->pole_angle_getter->get();

  if( this->measured_pole_angle != this->par->sign_conv*(angle_from_vision_system - this->par->initial_angle_offset) )
  {  
    // Sign convention: initialized in parameters_initialization()
    this->measured_pole_angle       = this->par->sign_conv*( angle_from_vision_system - this->par->initial_angle_offset );
    this->visualization_pole_angle  = this->measured_pole_angle;
  }

}