#include "pole_balancing_apollo/feedback_control.hpp"


namespace feedback_control {

  /* FeedbackController ---------------------------------------------------------------------------------------------*/

  FeedbackController::FeedbackController(std::vector<double> default_F){
    this->default_F = default_F;
    this->current_F = default_F;
    this->saturation = 0; // 0 means no saturation
  }

  FeedbackController::FeedbackController(){
    this->saturation  = 0.0; // 0 means no saturation
  }

  void FeedbackController::set_saturation(double saturation){
    this->saturation = saturation;
  }

  int FeedbackController::get_dimension(){
    return this->default_F.size();
  }

  void FeedbackController::switch_to_default_controller(){
    this->current_F = this->default_F;
  }

  void FeedbackController::update_controller(std::vector<double> F){
    this->current_F = F;
  }

  void FeedbackController::update_default_controller(std::vector<double> F){
    this->default_F = F;
  }

  double FeedbackController::get_control(std::vector<double> states){
    // Compute control:
    return this->compute_control_signal(states);
  }

  double FeedbackController::apply_saturation(double u){

    double u_sat = u;

    // Apply saturation:
    if (this->saturation==0.0) 
      u_sat = u;

    if (u < -this->saturation) 
      u_sat = -this->saturation;

    if (u > this->saturation) 
      u_sat = this->saturation;

    return u_sat;
  }

  /* LQRController ---------------------------------------------------------------------------------------------*/

  LQRController::LQRController(std::vector<double> default_F) : FeedbackController(default_F) {}
  LQRController::LQRController() : FeedbackController() {}

  void LQRController::reset(){}

  double LQRController::compute_control_signal(std::vector<double> states){
    
    double sum = 0;
    for(int i=0;i<this->current_F.size();i++) 
      sum+=this->current_F[i]*states[i];
    
    return sum;
  }


  /* PoleBalancingLQRController ------------------------------------------------------------------------------------*/

  PoleBalancingLQRController::PoleBalancingLQRController(std::vector<double> default_F) : LQRController(default_F)
  {
    if(default_F.size()!=5)
      throw std::runtime_error("pole balancing default controller accepts only gain vector of size 5");
  }

  PoleBalancingLQRController::PoleBalancingLQRController( std::string topic_lqr_safe, 
                                                          std::string topic_id_lqr_safe,
                                                          int id_lqr_safe,
                                                          std::string topic_lqr_search,
                                                          std::string topic_id_lqr_search,
                                                          int id_lqr_search,
                                                          int Nx, int Nu) : LQRController(){

    lqr_gains_safe_subs = std::make_shared<LQRGainsCollector>(topic_lqr_safe,topic_id_lqr_safe,id_lqr_safe,Nx,Nu);
    lqr_gains_search_subs = std::make_shared<LQRGainsCollector>(topic_lqr_search,topic_id_lqr_search,id_lqr_search,Nx,Nu);

    // Define some variables:
    this->Nel = Nx;
    this->read_F.resize(this->Nel); // This is called in the init() function, so it's not real-time sensitive

    // Loop until gains are successfully read:
    if(!ros::ok())
      this->verbosity.print("For some reason ros::ok() = false... \n");

    ros::Rate loop_rate(1);
    while ( ros::ok() )
    {
      if( this->lqr_gains_safe_subs->get(this->read_F) ){
        this->verbosity.print("LQR SAFE Gains successfully collected!...\n");
        this->verbosity.print("SAFE Gains will be used to control the robot by default.\n");
        break;
      }

      sprintf(this->buff,"Waiting for LQR gains to be lqr_gains_safe_subs from topic \"%s\"\n",topic_lqr_safe.c_str());
      this->verbosity.print(this->buff);
      loop_rate.sleep();
    }

    // Gain of the integrator:
    this->read_F[this->Nel-1] = -0.3;

    // Verbosity:
    this->verb_math.print("read_F",this->read_F);

    // Update current and default controllers:
    this->default_F = this->read_F;
    this->current_F = this->read_F;

    // while ( ros::ok() )
    // {
    //   if( this->lqr_gains_search_subs->get(F) ){
    //     printf(" ** LQR SEARCH Gains successfully lqr_gains_safe_subs!...\n");
    //     break;
    //   }

    //   printf(" ** Waiting for LQR gains to be lqr_gains_search_subs from topic \"%s\"\n",topic_lqr_search.c_str());
    //   loop_rate.sleep();
    // }

    // Safe state-space box:
    this->x_safe = x_safe;
    this->xdd_safe = xdd_safe;

  }

  bool PoleBalancingLQRController::detect_mismatch(void){

    bool mismatch_detected = false;

    // Analyze mismatch only if the gains were successfully read:
    bool collect_sucessfull = this->lqr_gains_search_subs->get(this->read_F);
    if(collect_sucessfull){

      // Add integrator gain:
      this->read_F[this->Nel-1] = -0.3;

      mismatch_detected = true;

      // Update current controller (TODO: this must be done somewhere else. ideally in the pole balancing task):
      this->current_F = this->read_F;

      // Verbosity:
      this->verbosity.print("New controller detected, and loaded on the robot\n");
      this->verb_math.print("current_F",this->current_F);
    }

    return mismatch_detected;
  }

  void PoleBalancingLQRController::update_controller(std::vector<double> F){
    this->lqr_gains_search_subs->get(this->current_F);
  }

  void PoleBalancingLQRController::update_default_controller(std::vector<double> F){
    this->lqr_gains_safe_subs->get(this->default_F);
  }
  
  void PoleBalancingLQRController::reset(){

    // Switch back to the default controller:
    this->current_F = this->default_F;

  }
  
  double PoleBalancingLQRController::compute_control_signal(std::vector<double> states)
  {

    // Sanity check:
    if(states.size()!=this->current_F.size())
      throw std::runtime_error("feedback controller error: gains and input of different size");

    // Expansion for clarity:
    double angle         = states[0];
    double velocity      = states[1];
    double cart_plane_x  = states[2];
    double cart_plane_xd = states[3];
    double integrator    = states[4];

    // Compute controller:
    double u_raw = - (this->current_F[0]*angle + this->current_F[1]*velocity + this->current_F[2]*cart_plane_x + this->current_F[3]*cart_plane_xd + this->current_F[4]*integrator);
    
    // Apply saturation:
    double u_sat = apply_saturation(u_raw);

    return u_sat;
  }


  SinusoidalInput::SinusoidalInput(double w, double amp_a, double time_step){
    this->w = w;
    this->amp_a = amp_a;
    this->time_step = time_step;
    this->time_counter = 0.0;
    this->saturation = 0.0;
  }

  double SinusoidalInput::compute_control_signal(std::vector<double> foo){

    this->time_counter += this->time_step;
    // double u_sinu = this->amp_a * sin(this->w * this->time_counter);
    double u_sinu = this->amp_a * cos(this->w * this->time_counter);

    return u_sinu;
  }

}
