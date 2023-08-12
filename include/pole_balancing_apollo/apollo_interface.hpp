#pragma once

#include "pole_balancing_apollo/parameters.hpp"
#include "tools/fake_sensor.hpp"
#include "tools/tools.hpp"

#include <memory>
#include <stdlib.h>
#include <map>
#include <vector>

#include "SL_system_headers.h"
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
#include "SL_filters.h"

#ifdef __XENO__
  #include <native/task.h>
#else
  #include <ctime>
#endif

#define Norm(x,y)     ( sqrt( sqr(x) + sqr(y) ) )


namespace apollo_interface {

  /* ------------------------------- State class ------------------------------- */

  class State {
  public:
    State();
    std::vector<double> x;
    std::vector<double> xd;
    std::vector<double> xdd;
  };

  /* ------------------------------- Home position class (robot movement functions that block SL task) ------------------------------- */ 

  class Go_home{
  public:
    virtual ~Go_home(){}
    virtual bool move_to_initial_position(void)=0;
  protected:
    void read_default_position(SL_DJstate desired_position[N_DOFS+1]);
    bool move_to(SL_DJstate desired_position[N_DOFS+1], int time2finish_in_ns, std::string where2move);
  };

  class Go_home_pole_balancing : public Go_home{
  public:
    Go_home_pole_balancing(bool use_head);
    bool move_to_initial_position(void);
  private:
    void add_position_home(SL_DJstate desired_position[N_DOFS+1]);
    void add_position_hand_closed(SL_DJstate desired_position[N_DOFS+1]);
    void add_position_head(SL_DJstate desired_position[N_DOFS+1]);
    bool use_head;

  };

  /* ------------------------------- Target_state_in_robot_frame class ------------------------------- */  

  class Target_state_in_robot_frame {
  public:
    Target_state_in_robot_frame(const std::vector<double> cartesian_offset, double time_step, double plane_angle);
    State get_from_endeff_acceleration(double endeffector_acceleration);
  private:
    double previous_position[3];
    double previous_velocity[3];
    double previous_acceleration[3];
    double time_step;
    double plane_angle;
    bool   initial_condition_euler_integration;
  };

  /* ------------------------------- Inverse_kinematics_and_dynamics class ------------------------------- */  

  class Inverse_kinematics_and_dynamics{
  public:
    Inverse_kinematics_and_dynamics(int filter_order, int filter_cutoff, double time_step);
    ~Inverse_kinematics_and_dynamics();
    bool update_desired_state_arm(State endeffector_target);
    void update_desired_state_head(void);
    void update_uff(void);
    SL_DJstate * joints_target;
  private:
    Filter *fthdd;
    double *cart;
    int *cstatus;
    std::vector<int> joints_to_freeze;
    double time_step;
  };

  /* ------------------------------- function that returns current jacobian transpose ------------------------------- */  
  static void _get_jacobian_transpose(SL_DJstate *target, Matrix jacobian_transpose);  


  /* ------------------------------- final control ------------------------------- */  

  class Cartesian_feedback_control_loop {
  public:
    Cartesian_feedback_control_loop(bool use_forg_factor, int win_time, double kp, double kd, double ki,double time_step);
    ~Cartesian_feedback_control_loop();
    void update_arm_uff(State target_in_robot_frame, const Matrix jacobian_transpose, bool dbg_integrator);
  private:
    std::vector<int> ind_win_per_joint;
    bool use_forg_factor;
    int win_time;
    int win_length;
    Matrix integrator_window;
    SL_DJstate initial_position[N_DOFS+1]; 
    Vector ufb_pose_KP;
    Vector ufb_pose_KD;
    Vector ufb_pose_KI;
    double KP_pose[4];
    double KD_pose[4];
    double KI_pose[4];
    double time_step;
  };

  /* ------------------------------- Measure endeffector state ------------------------------- */  

  class Measure_endeffector_cartesian_state{
  public:
    Measure_endeffector_cartesian_state(double noise_std, int sampling_steps, int delay_time_steps);
    CartesianState update_and_get();
  private:
    void update();
    CartesianState endeffector_state_measured;
    SensorCartesianState sensor_endeff;
  };

  /* ------------------------------- standalone functions ------------------------------- */
  void read_cartesian_offset(std::vector<double> & cartesian_offset);

  /* ------------------------------------- control interface for end user ------------------------------------- */


  // Construct needed classes:
  static std::shared_ptr<apollo_interface::Cartesian_feedback_control_loop> apollo_endeff_feedback;
  static std::shared_ptr<apollo_interface::Target_state_in_robot_frame> target_in_robot_frame;
  static std::shared_ptr<apollo_interface::Inverse_kinematics_and_dynamics> apollo_IKD;

  /*****************************************************************************
  ******************************************************************************
    \note   init_control
    \date   Jan. 2017
    \author Vincent Berenz
  
    \remarks
    
    Read end-effector cartesian state, to be kept as a constant offset.
    Initialization of the objects:
      apollo_control
      target_in_robot_frame
      inverse_kinematics_and_dynamics
  
  ******************************************************************************
    Parameters:  (i/o = input/output)
  
  ******************************************************************************/



  bool init_control(bool use_forg_factor, int window_time, double kp, double kd, double ki, 
 									int filter_order, int filter_cutoff, double time_step, double plane_angle);

  bool apply_control(double control, bool use_head, bool dbg_integrator);

  /* ------------------------------- Stop Robot class ------------------------------- */

  class StopRobot{

  public:
    virtual ~StopRobot(){}
    virtual bool within_time(double task_servo_time)=0;
    virtual bool within_task_space_limits(std::vector<double> endeff_pos_in_plane)=0;
    virtual bool within_joint_space_limits(SL_Jstate * joint_state, double joint_range[][4])=0;

  };

  class StopApollo : public StopRobot {

  public:
    StopApollo(bool record_data, int duration_task, std::vector<double> endeff_box_limits);
    bool within_time(double task_servo_time);
    bool within_task_space_limits(std::vector<double> endeff_pos_in_plane);
    bool within_joint_space_limits(SL_Jstate * joint_state, double joint_range[][4]);

  private:
    bool   record_data;
    int    duration_task;
    double start_time;
    double current_task_time;
    std::vector<double> endeff_box_limits;

  };

  /* -------------------------------- stopping options interface for end user -------------------------------- */

  static std::shared_ptr<apollo_interface::StopRobot> stop_robot = NULL;

  /*****************************************************************************
  ******************************************************************************
    \note   stop_robot
    \date   Feb. 2017
    \author Alonso Marco
  
    \remarks
  
  ******************************************************************************
    Parameters:  (i/o = input/output)
  
  ******************************************************************************/
  void init_stop_criterions(bool record_data, int duration_task, std::vector<double> endeff_box_limits);

  bool check_stopping_criterions(std::vector<double> endeff_pos_in_plane);

  /* -------------------------------- Classes for testing purposes -------------------------------- */


} // namespace apollo_interface


/* -------------------------------- Record data -------------------------------- */

namespace record_SL_data{

  class Rec{
  public:
    virtual ~Rec(){}
    virtual void update_readings()=0;
  };

  class Rec_LQRPoleBalancing : public Rec {
  public:
    Rec_LQRPoleBalancing( std::shared_ptr<parameters::YAML_data_share> estimate_states_pars,
                          std::shared_ptr<parameters::YAML_data_share> measurements_pars,
                          std::shared_ptr<parameters::YAML_data_share> control_pars);
    void update_readings();
  private:
    const std::shared_ptr<parameters::YAML_data_share> estimate_states_pars; // Read-only
    const std::shared_ptr<parameters::YAML_data_share> measurements_pars; // Read-only
    const std::shared_ptr<parameters::YAML_data_share> control_pars; // Read-only

    // Tracked variables:
    std::vector<double> states;
    double measured_pole_angle;
    double visualization_pole_angle;
    double u;

  };

  class Rec_PIDPoleBalancing : public Rec {
  public:
    Rec_PIDPoleBalancing(std::shared_ptr<parameters::YAML_data_share> estimate_states_pars);
    void update_readings();
  private:
    const std::shared_ptr<parameters::YAML_data_share> estimate_states_pars; // Read-only

    // Tracked variables:
    double states_S;
  };

} // namespace record_SL_data
