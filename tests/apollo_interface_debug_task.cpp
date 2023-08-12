#include "pole_balancing_apollo/apollo_interface.hpp"

static std::shared_ptr<apollo_interface::Measure_endeffector_cartesian_state> measurements_endeff = NULL;
static int init_apollo_interface_debug_task(void) {

  
  std::vector<double> endeff_box_limits;
  endeff_box_limits.push_back(0.3);
  endeff_box_limits.push_back(0.1);
  endeff_box_limits.push_back(0.1);
  
  apollo_interface::init_stop_criterions(false,36000,endeff_box_limits);
  std::shared_ptr<apollo_interface::Go_home> go_home_movement = std::make_shared<apollo_interface::Go_home_pole_balancing>(false);
  measurements_endeff = std::make_shared<apollo_interface::Measure_endeffector_cartesian_state>(0.0,0,0);
  apollo_interface::init_control(false,5,2000.0,10.0,100.0,2,5,1.0/(double)task_servo_rate,-30*3.1451/180);

  return TRUE;
  
}


static int run_apollo_interface_debug_task(void) {

  // test only : the robot must not move
  bool apply_to_robot = false;

  CartesianState endeffector_state_measured;
  endeffector_state_measured = measurements_endeff->update_and_get();

  if( !apollo_interface::apply_control(0.0,false,apply_to_robot) ){
    return FALSE;
  }
  return TRUE;
}

static int change_apollo_interface_debug_task(void){
  return TRUE;
}


extern "C" void add_apollo_interface_debug_task(void) {
    addTask("Pole Balancing Apollo Interface", init_apollo_interface_debug_task, 
      run_apollo_interface_debug_task, change_apollo_interface_debug_task);
}
