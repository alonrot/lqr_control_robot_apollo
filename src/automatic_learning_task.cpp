#include "pole_balancing_apollo/measurements.hpp"
#include "pole_balancing_apollo/estimate_states.hpp"
#include "pole_balancing_apollo/feedback_control.hpp"
#include "pole_balancing_apollo/apollo_interface.hpp"
#include "pole_balancing_apollo/parameters.hpp"
#include "pole_balancing_apollo/pole_physical_parameters.h"
#include "pole_balancing_apollo/StateMachineLearning.hpp"
#include "lqr_gains/ComputeLQRcost.hpp"
#include "std_msgs/Bool.h"

// Sleep task:
#include <chrono>
#include <thread>

static std::shared_ptr<Measurements> measurements_pole;
static std::shared_ptr<apollo_interface::Measure_endeffector_cartesian_state> measurements_endeff;
static std::shared_ptr<EstimateStates> state_estimation;
static std::shared_ptr<feedback_control::FeedbackController> LQRcontroller;
static std::shared_ptr<state_machine_learning::StateMachineLearning> sml;

// Compute cost:
static std::shared_ptr<ComputeLQRcost> cost;

// User-defined global variables:
bool use_head;
static double angle_current;
static std::vector<double> states;
static double u;
std::vector<double> endeff_pos_in_plane;
static int flag_unstability;
static bool record_data_for_learning;
static bool record_data_now;
static bool stop_robot;
static int  counter_;
static std::shared_ptr<Tools::TaskCounter> task_counter;

static std::shared_ptr<ros::NodeHandle> nh;
static rosrt::Publisher<std_msgs::Bool> pub_I_am_alive;
static std_msgs::BoolPtr msg_I_am_alive;

//DBG:
bool dbg_integrator;
static  double  time_of_iteration;


static int init_automatic_learning_task(void) {

  printf("\n");
  printf("[INFO] ** @init_automatic_learning_task()\n");

  int ans, i, j, k;

  /* DEBUGGING */
  time_of_iteration = 0.0;
  addVarToCollect((char *)&(time_of_iteration),"DBG_elapsed_timee","ns",DOUBLE,TRUE);

  // Pointers local to init():
  std::shared_ptr<parameters::YAML_pole_balancing_config> pars = NULL;
  std::shared_ptr<apollo_interface::Go_home> go_home_movement = NULL;

  // Parameters:
  pars = std::make_shared<parameters::YAML_pole_balancing_config>(YAML_CONFIG_FILE);

  /* Variables shared by the constructors of many classes */
  double plane_angle_deg  = pars->get<double>("plane_angle_deg");
  double plane_angle      = plane_angle_deg*M_PI/180;
  double time_step        = 1.0/(double)task_servo_rate;

  /* Initialize stopping criterions */
  bool record_data_standard               = pars->get<bool>("record_data_standard");
  int duration_task                       = pars->get<int>("duration_task");
  std::vector<double> endeff_box_limits   = pars->get< std::vector<double> >("endeff_box_limits");
  apollo_interface::init_stop_criterions(record_data_standard,duration_task,endeff_box_limits);

  /* Execute blocking functions that move the arm (and head) to the home position */
    // Read variables:
    use_head = pars->get<bool>("use_head");
    // Go home:
    go_home_movement = std::make_shared<apollo_interface::Go_home_pole_balancing>(use_head);

    if ( !go_home_movement->move_to_initial_position() )
      return FALSE;

  /* Initialize measurements pole class */
    // Read parameters:
      bool sim_flag = pars->get<bool>("sim_flag");
      std::string topic_name_pole_angle_visualization = pars->get<std::string>("topic_name_pole_angle_visualization");

    // Simulation or real experiments?
    if (sim_flag){
      double sigma_n            = pars->get<double>("sigma_n");
      int vision_Ts             = pars->get<int>("vision_Ts");
      int VISION_LATENCY        = pars->get<int>("VISION_LATENCY");
      double g_n                = pars->get<double>("g_n");
      double initial_angle_sim  = pars->get<double>("initial_angle_sim");
      // Pole physical parameters:
      PoleParameters pole_para;
      pole_para.length =          pars->get<double>("pole_length");
      pole_para.mass =            pars->get<double>("pole_mass");
      pole_para.center_of_mass =  pars->get<double>("pole_center_of_mass");
      pole_para.inertia =         pars->get<double>("pole_inertia");
      pole_para.damping =         pars->get<double>("pole_damping");

      // Initialize class:
      measurements_pole = std::make_shared<SnatchPoleSimulation>(sigma_n,vision_Ts,VISION_LATENCY,initial_angle_sim,
                                                  time_step,g_n,pole_para,topic_name_pole_angle_visualization);
    }
    else{
      int sign_conv = pars->get<int>("sign_conv");
      double initial_angle_vicon = pars->get<double>("initial_angle_vicon");
      std::string topic_name_read_pole_angle_vicon = pars->get<std::string>("topic_name_read_pole_angle_vicon");
      measurements_pole = std::make_shared<SnatchPoleVicon>(sign_conv,initial_angle_vicon,topic_name_pole_angle_visualization,topic_name_read_pole_angle_vicon);
    }

  /* End-effector measurements class */
    int ENDEFF_LATENCY = pars->get<int>("ENDEFF_LATENCY");
    measurements_endeff = std::make_shared<apollo_interface::Measure_endeffector_cartesian_state>(0.0,0,ENDEFF_LATENCY);

  /* Initialize some global variables to be used in run(): */
  int Nx = pars->get<int>("Nx");
  int Nu = pars->get<int>("Nu");
  int _NCART_ = pars->get<int>("_NCART_");
  for(int i=0;i<Nx;++i)
    states.push_back(0);
  for(int i=0;i<_NCART_;++i)
    endeff_pos_in_plane.push_back(0);
  task_counter = std::make_shared<Tools::TaskCounter>();

  /* Initialize Control class */
    int which_control = pars->get<int>("which_control");
    double u_sat = pars->get<double>("u_sat");
    switch(which_control){
      case 1 :
      {
        // Read parameters:
        std::string topic_name_LQRsafe = pars->get<std::string>("topic_name_LQRsafe");
        std::string topic_id_LQRsafe = pars->get<std::string>("topic_id_LQRsafe");
        int id_LQRsafe = pars->get<int>("id_LQRsafe");

        std::string topic_name_LQRsearch = pars->get<std::string>("topic_name_LQRsearch");
        std::string topic_id_LQRsearch = pars->get<std::string>("topic_id_LQRsearch");
        int id_LQRsearch = pars->get<int>("id_LQRsearch");

        // Initialize controller:
        LQRcontroller = std::make_shared<feedback_control::PoleBalancingLQRController>(topic_name_LQRsafe,
                                                                                      topic_id_LQRsafe,
                                                                                      id_LQRsafe,
                                                                                      topic_name_LQRsearch,
                                                                                      topic_id_LQRsearch,
                                                                                      id_LQRsearch,
                                                                                      Nx,Nu);
        LQRcontroller->set_saturation(u_sat);
        break;
      }
      case 2 :
      {
        // Read parameters:
        double f      = pars->get<double>("f");
        double w      = 2*M_PI*f;
        double amp_a  = pars->get<double>("amp_a");
        // Initialize controller:
        LQRcontroller = std::make_shared<feedback_control::SinusoidalInput>(w,amp_a,time_step);
        LQRcontroller->set_saturation(u_sat);
        break;
      }
    }
    
  /* Initialize State estimation class */
    // Read parameters:
    int which_filter_pole = pars->get<int>("which_filter_pole");
    int which_filter_xdd  = pars->get<int>("which_filter_xdd");
    bool integral_action  = pars->get<bool>("integral_action");
    std::vector<double> state_space_box_limits  = pars->get< std::vector<double> >("state_space_box_limits");
    double endeff_acc_limits = pars->get<double>("endeff_acc_limits");
    std::vector<double> endeff_offset;
    endeff_offset.resize(3);
    apollo_interface::read_cartesian_offset(endeff_offset);

    printf("Reading end-effector position as an offset...\n");
    for(i=0;i<3;++i)
      printf("endeff_offset[%i] = %f\n",i,endeff_offset[i]);

    // Initialize:
    state_estimation = std::make_shared<EstimateStatesPole>(plane_angle,time_step,which_filter_pole,which_filter_xdd,
                                                  integral_action,endeff_offset,
                                                  state_space_box_limits,endeff_acc_limits);

  /* States Machine Learning class */
    // Read parameters:
    int duration_experiments = pars->get<int>("duration_experiments");
    int K_exp = (int)(duration_experiments * task_servo_rate);

    // Initialize:
    sml = std::make_shared<state_machine_learning::StateMachineLearning>(K_exp);


  /* Initialize general apollo robot control */
    // Read variables from config file:
    bool use_forg_factor  = pars->get<bool>("use_forg_factor");
    int window_time       = pars->get<int>("window_time");
    double kp             = pars->get<double>("kp");
    double kd             = pars->get<double>("kd");
    double ki             = pars->get<double>("ki");
    dbg_integrator   = pars->get<bool>("dbg_integrator");
    int filter_order      = pars->get<int>("filter_order");
    int filter_cutoff     = pars->get<int>("filter_cutoff");

    // Initialize control (reading cartesian end-effector state):
    apollo_interface::init_control(use_forg_factor,window_time,kp,kd,ki,
                                  filter_order,filter_cutoff,time_step,plane_angle);

  /* Initialize cost */
    std::string path2LQRweights_empirical = pars->get<std::string>("path2LQRweights_empirical");
    YAML::Node node_LQRweights;
    try{
      node_LQRweights = YAML::LoadFile(path2LQRweights_empirical);
    }
    catch(...){
      printf("Failed to load the file %s\nSpecified in %s\n",path2LQRweights_empirical.c_str(),YAML_CONFIG_FILE);
      printf("Exit the task. Bye!\n");
      return FALSE;
    }

    Eigen::VectorXd Q_empirical  = node_LQRweights["Q_empirical"].as<Eigen::VectorXd>();
    Eigen::VectorXd R_empirical  = node_LQRweights["R_empirical"].as<Eigen::VectorXd>();
    double J_heuristic          = pars->get<double>("heuristic_cost");

    cost = std::make_shared<ComputeLQRcost>(Q_empirical,R_empirical,J_heuristic);


  /* Other variables */
  record_data_for_learning  = pars->get<bool>("record_data_for_learning");
  bool activate_temporizer  = pars->get<bool>("activate_temporizer");
  int  temporizer_time      = pars->get<int>("temporizer_time");

  if(activate_temporizer)
    printf("Temporizer of %i seconds will be used!\n",temporizer_time);
  else
    printf("No temporizer will be used.\n");

  // Variables to be recorded:
  angle_current = 0.0;
  u             = 0.0;
  flag_unstability = FALSE;
  stop_robot = false;
  counter_ = 0;
  record_data_now = false;

  /* Control problem variables */
  if(record_data_standard){
    addVarToCollect((char *)&(states[0]),"states[PHI]","[rad]",DOUBLE,TRUE);
    addVarToCollect((char *)&(states[1]),"states[PHID]","[rad/s]",DOUBLE,TRUE);
    addVarToCollect((char *)&(states[2]),"states[S]","[m]",DOUBLE,TRUE);
    addVarToCollect((char *)&(states[3]),"states[SD]","[m/s]",DOUBLE,TRUE);
    addVarToCollect((char *)&(states[4]),"states[INT_]","[m*s]]",DOUBLE,TRUE);
    addVarToCollect((char *)&(angle_current),"y[PHI]","[rad]",DOUBLE,TRUE);
    addVarToCollect((char *)&(u),"u","[m/s2]",DOUBLE,TRUE);
    addVarToCollect((char *)&(flag_unstability),"flag_unstability","bool",INT,TRUE);
  }
  
  /* Start the task */
  ans = 999;
  while (ans == 999) {
    if (!get_int("Enter 1 to start to balance the pole or anthing else to abort ...",ans,&ans))
      return FALSE;
  }
  if (ans != 1) 
    return FALSE;

  // Robot status:
    // Arguments for node:
    std::string node_robot_alive  = pars->get<std::string>("node_robot_alive");
    std::string topic_robot_alive = pars->get<std::string>("topic_robot_alive");
    int argc = 1; char* argv[1];
    argv[0] = new char[node_robot_alive.size() + 1];
    std::copy(node_robot_alive.begin(), node_robot_alive.end(), argv[0]);
    argv[0][node_robot_alive.size()] = '\0';

    // Initialize node:
    ros::init(argc,argv,node_robot_alive);
    struct rosrt::InitOptions options;
    options.pubmanager_thread_name = node_robot_alive;
    rosrt::init(options);

    // Publisher:
    nh = std::make_shared<ros::NodeHandle>();
    pub_I_am_alive.initialize(nh->advertise<std_msgs::Bool>(topic_robot_alive, 0), 100, std_msgs::Bool());

    // Allocate memory for the message:
    msg_I_am_alive = pub_I_am_alive.allocate();
    msg_I_am_alive->data = true;

  // Use temporizer:
  if(activate_temporizer){
    printf("Using temporizer for %i seconds\n",temporizer_time);
    std::this_thread::sleep_for(std::chrono::nanoseconds((long)(temporizer_time*1.e9))); // Tested, works!
  }

  // Recording data:
  if (record_data_standard){
    scd();
    // sendMessageMotorServo("scdMotor",NULL,0);
  }

  printf("Starting task!\n");
  printf("\n");

  return TRUE;
}


static int run_automatic_learning_task(void) {

  task_counter->restart();

  // Stop collecting data:
  if(record_data_now){
    if(record_data_for_learning){
      stopcd();
      sendCommandLineCmd((char*)"saveData");
    }
    record_data_now = false;
    flag_unstability = FALSE;
  }

  /* Update measurements_pole */
  measurements_pole->update_measurements();
  measurements_pole->publish_pole_angle_for_visualization();

  /* Update measurements_endeff */
  CartesianState endeffector_state_measured;
  endeffector_state_measured = measurements_endeff->update_and_get();

  /* Update states */
  double angle_current = measurements_pole->measured_pole_angle;
  state_estimation->update_states(endeffector_state_measured,angle_current);
  state_estimation->get_states(states);

  /* Update state machine */
    // Detect controller change, AND update the current controller, if it is different:
    bool controller_mismatch = LQRcontroller->detect_mismatch();

    // Detect unstability:
    bool is_stable = state_estimation->within_state_space_limits();

    // States machine learning update:
    sml->update(controller_mismatch,is_stable);

  /* Start collecting data if needed */
  if(sml->start_collecting_data){ // We get here only once

    if(record_data_for_learning)
      scd();

    // Update cost:
    cost->reset_cost();
    // std::cout << "@start colecting data: Reset cost\n";

    sml->start_collecting_data = false;
  }

  /* Update controller */
  if(sml->reset_to_safe_controller){
    LQRcontroller->reset();
    sml->reset_to_safe_controller = false;
    record_data_now = true;

    // Publish cost value:
    if(sml->unstability_flag){
      // Publish heuristic
      cost->publish_heuristic_cost();
      // std::cout << "@reset controller: Publish heuristic\n";
    }
    else{
      // Publish current cost 
      cost->publish_current_cost();
      // std::cout << "@reset controller: Publish current\n";
    }

    // Reset anyways:
    cost->reset_cost();
    // std::cout << "@reset controller: Reset cost\n";

  }

  /* Detect unstability */ // Needed for data postprocessing in matlab
  if(sml->unstability_flag){
    flag_unstability = TRUE;
    sml->unstability_flag = false;
  }

  /* Update control */
  u = LQRcontroller->get_control(states);
  measurements_pole->read_endeff_acceleration(u);

  // Update cost if possible:
  if(sml->get_machine_state() == 2)
    cost->update_cost(states,u);

  /* Send controller to the robot */
  if( !apollo_interface::apply_control(u,use_head,dbg_integrator) )
    return FALSE;

  /* Stopping if needed */
  state_estimation->get_endeff_pos_in_plane(endeff_pos_in_plane);
  if( !apollo_interface::check_stopping_criterions(endeff_pos_in_plane) )
    return FALSE;

  time_of_iteration = task_counter->get_elapsed_time_in_us();

  // Send robot status:
  pub_I_am_alive.publish(msg_I_am_alive);

  return TRUE;
}

static int change_automatic_learning_task(void){

  #ifdef __XENO__
    rt_printf("[INFO] ** @change_automatic_learning_task()\n");
  #else
    printf("[INFO] ** @change_automatic_learning_task()\n");
  #endif

  stopcd();
  sendCommandLineCmd((char*)"saveData");
  return TRUE;
}


extern "C" void add_automatic_learning_task(void) {
    addTask("Automatic Learning with Pole Balancing", init_automatic_learning_task, 
      run_automatic_learning_task, change_automatic_learning_task);
}
