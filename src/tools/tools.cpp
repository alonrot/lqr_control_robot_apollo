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

#include "tools/tools.hpp"

CartesianState::CartesianState(){
  this->x.resize(3);
  this->xd.resize(3);
  this->xdd.resize(3);
}

SensorCartesianState::SensorCartesianState(){
  this->x.resize(3);
  this->xd.resize(3);
  this->xdd.resize(3);
}

/*!*****************************************************************************
 *******************************************************************************
\note  vec_zero
\date  January 2006
\remarks 

  From SL utility.c

 ******************************************************************************/

namespace Tools {

  void     
  vec_zero(std::vector<double> v)
  {
    std::fill(v.begin(), v.end(), 0);
  }

  /*!*****************************************************************************
   *******************************************************************************
  \note  vec_zero
  \date  January 2006
  \remarks 

    From SL utility.c

   ******************************************************************************/

  void     
  vec_zero(std::vector<int> v)
  {
    std::fill(v.begin(), v.end(), 0);
  }


  /*!*****************************************************************************
   *******************************************************************************
  \note  sign
  \date  January 2006
  \remarks 

    From SL utility.c

   ******************************************************************************/

  double
  sign(double expr)
  {
    if (expr > 0)
      return (1.0);
    else
      if (expr < 0)
        return (-1.0);
      else
        return (0.0);
  }

  /*!*****************************************************************************
   *******************************************************************************
  \note  eulerToRotMat
  \date  January 2006
  \remarks 

    From SL SL_common.c

    Converts an a-b-g Euler angle notation to a rotation matrix. This is
    the forward transformation (from global to local coordinates), computed
    as 

    Rz*Ry*Rx  where

    Rx = {{1, 0, 0}, {0, Cos[a], Sin[a]}, {0, -Sin[a], Cos[a]}}
    Ry = {{Cos[b], 0, -Sin[b]}, {0, 1, 0}, {Sin[b], 0, Cos[b]}}
    Rz = {{Cos[g], Sin[g], 0}, {-Sin[g], Cos[g], 0}, {0, 0, 1}}

   ******************************************************************************/
  void
  eulerToRotMat(double a[N_CART], double R[N_CART][N_CART])
  {
    R[_x_][_x_] =  cos(a[_y_])*cos(a[_z_]);
    R[_x_][_y_] =  cos(a[_z_])*sin(a[_x_])*sin(a[_y_]) + cos(a[_x_])*sin(a[_z_]);
    R[_x_][_z_] = -(cos(a[_x_])*cos(a[_z_])*sin(a[_y_])) +sin(a[_x_])*sin(a[_z_]);
    R[_y_][_x_] = -(cos(a[_y_])*sin(a[_z_]));
    R[_y_][_y_] =  cos(a[_x_])*cos(a[_z_]) - sin(a[_x_])*sin(a[_y_])*sin(a[_z_]);
    R[_y_][_z_] =  cos(a[_z_])*sin(a[_x_]) + cos(a[_x_])*sin(a[_y_])*sin(a[_z_]);
    R[_z_][_x_] =  sin(a[_y_]);
    R[_z_][_y_] = -(cos(a[_y_])*sin(a[_x_]));
    R[_z_][_z_] =  cos(a[_x_])*cos(a[_y_]);

  }

  ////////////////////////////////////////////////////////////////////////////////////////

  TaskCounter::TaskCounter(void){

    #ifdef __XENO__
      this->time_start = rt_timer_ticks2ns(rt_timer_read());
    #else
      this->time_start = std::clock();
    #endif
  }

  void
  TaskCounter::restart(){

    #ifdef __XENO__
      this->time_start = rt_timer_ticks2ns(rt_timer_read());
    #else
      this->time_start = std::clock();
    #endif

  }

  double
  TaskCounter::get_elapsed_time_in_us(){

    double elapsed_time;

    #ifdef __XENO__
      SRTIME current_time = rt_timer_ticks2ns(rt_timer_read());
      elapsed_time = rt_timer_ticks2ns( current_time - this->time_start)*0.001;
    #else
      clock_t elapsed_time_aux = std::clock() - this->time_start;
      elapsed_time = ((float)elapsed_time_aux)/CLOCKS_PER_SEC*1000000;
    #endif

    return elapsed_time;
  }

  double
  TaskCounter::get_elapsed_time_in_s(){

    double elapsed_time;

    #ifdef __XENO__
      SRTIME current_time = rt_timer_ticks2ns(rt_timer_read());
      elapsed_time = rt_timer_ticks2ns( current_time - this->time_start)*0.000000001;
    #else
      clock_t elapsed_time_aux = std::clock() - this->time_start;
      elapsed_time = ((float)elapsed_time_aux)/CLOCKS_PER_SEC;
    #endif

    return elapsed_time;
  }


  //////////////////////////////////////////////////////////////////////////////////////////

  SignalTreatment::SignalTreatment(double time_step){

    this->pos_old = 0;
    this->vel_old = 0;
    this->count_steps = 1.0;
    this->time_step = time_step;
    this->first_time = true;
  }

  double SignalTreatment::differentiate(double pos)
  {
    double vel;

    if (this->first_time){
      vel = 0.0;
      this->pos_old = pos;
      this->first_time = false;
    } 
    else{

      if (pos == pos_old){ // Keep
        vel = vel_old; 
        this->count_steps += 1.0;  
      }
      else{ // Update
        vel = (pos - this->pos_old)/(this->count_steps*this->time_step);
        this->count_steps = 1.0;

        // Update old position:
        this->pos_old = pos;
        this->vel_old = vel;
      }

      // Security check:
      // TODO: do not hard-code this number
      if (this->count_steps >= 100){
          vel = 0;
          this->verbosity.print("@SignalTreatment::differentiate: this->count_steps >= 100\n");
      }

    }

    return vel;
  }



} // namespace Tools




// PrintThis::PrintThis(std::string my_msg, int max_budget, double time_limit){
    
//     // Input arguments:
//     this->my_msg.assign(my_msg);
//     this->max_budget = max_budget;
//     this->time_limit = time_limit;
//     #ifdef __XENO__
//       this->time_start = rt_timer_ticks2ns(rt_timer_read());
//     #else
//       this->time_start = std::time(0);
//     #endif

//     // Rest:
//     this->counter = 0;
//     this->restart_timer = true;
//     task_counter = std::make_shared<TaskCounter>();
// }

// bool
// PrintThis::budget_for_printing(void){

//   bool we_can_print = true;

//   ++this->counter;
//   if(this->counter > this->max_budget){
//     we_can_print = false;
//   }

//   return we_can_print;

// }

// /* 
//   This function returns always false, except when the elapsed time is bigger than
//   this->time_limit, or when this->time_limit = 0
// */
// bool
// PrintThis::pause_for_printing(void){

//   bool we_can_print;
//   double time_elapsed;

//   // Return if no time limit is passed:
//   if(this->time_limit == 0.0){
//     we_can_print = true;
//     return we_can_print;
//   }
//   else
//     we_can_print = false;

//   // Restart timer:
//   if(this->restart_timer){
//     task_counter->restart();
//     this->restart_timer = false;
//   }

//   // Compute current time:
//   time_elapsed = task_counter->get_elapsed_time_in_s();

//   // Compute elapsed time:
//   if (time_elapsed > this->time_limit){
//     we_can_print = true;
//     this->restart_timer = true;
//   }
//   else
//     we_can_print = false;

//   return we_can_print;

// }

// bool
// PrintThis::can_we_print(void){

//   bool thereis_budget;
//   bool printing = false;

//   // Introduce zero if no printing should be made:
//   if(this->max_budget == 0)
//     return printing;

//   // Wait some time before printing:
//   bool occasion_to_print = this->pause_for_printing();

//   // Print only if there is enough budget:
//   if(occasion_to_print){
//     thereis_budget = this->budget_for_printing();
//   }
//   else
//     thereis_budget = false; 

//   // Decide whether it is allowed to print, or not:
//   if(thereis_budget){
//     printing = true;
//   }

//   return printing;
// }

// void
// PrintThis::print(void){

//   bool printing_allowed = this->can_we_print();

//   if(printing_allowed){
//     #ifdef __XENO__
//       rt_printf("%s",this->my_msg.c_str());
//     #else
//       printf("%s",this->my_msg.c_str());
//     #endif
//   }

// }

// void
// PrintThis::print(std::string my_msg){

//   bool printing_allowed = this->can_we_print();

//   if(printing_allowed){
//     #ifdef __XENO__
//       rt_printf("%s",my_msg.c_str());
//     #else
//       printf("%s",my_msg.c_str());
//     #endif
//   }

// }