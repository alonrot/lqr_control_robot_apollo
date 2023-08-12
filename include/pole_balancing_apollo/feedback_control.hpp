#ifndef __FEEDBACK_CONTROL__
#define __FEEDBACK_CONTROL__

#include <string>
#include <vector>
#include <iostream>
#include "lqr_gains/GainsCollector.hpp"
#include "tools/PrintMath.hpp"

namespace feedback_control {

  /* controllers --------------------------------------------------------------------------------------*/

  class FeedbackController {
  public:
    virtual ~FeedbackController(){}
    FeedbackController(std::vector<double> default_F);
    FeedbackController();
    virtual void update_controller(std::vector<double> F);
    virtual void update_default_controller(std::vector<double> F);
    virtual void reset()=0;
    virtual bool detect_mismatch(void){printf("detect_mismatch() not implemented\n");};
    void set_saturation(double saturation);
    int get_dimension();
    void switch_to_default_controller();
    double get_control(std::vector<double> states);
  protected:
    virtual double compute_control_signal(std::vector<double> states)=0;
    double apply_saturation(double u);
    std::vector<double> current_F;
    std::vector<double> default_F;
    double saturation;
  };


  class PIDcontroller : public FeedbackController {

  public:
    PIDcontroller();
    void reset();
  protected:
    double compute_control_signal(std::vector<double> states);

  };


  class LQRController : public FeedbackController {
  public:
    LQRController(std::vector<double> default_F);
    LQRController();
    void reset();
  protected:
    double compute_control_signal(std::vector<double> states);
  };

  
  // expected input: time_step, angle, velocity, cart_plane_x, cart_plan_xd
  // expected 5 gains : for angle, velocity, cart_plan_x, cart_plan_xd and integral term
  // adviced usage: use compute_default_control functions defined in this file
  class PoleBalancingLQRController : public LQRController {
  public:
    PoleBalancingLQRController(std::vector<double> default_F);
    PoleBalancingLQRController( std::string topic_lqr_safe, 
                                std::string topic_id_lqr_safe,
                                int id_lqr_safe,
                                std::string topic_lqr_search,
                                std::string topic_id_lqr_search,
                                int id_lqr_search,
                                int Nx, int Nu);
    bool detect_mismatch(void);
    void update_controller(std::vector<double> F); // Override the function in FeedbackController
    void update_default_controller(std::vector<double> F); // Override the function in FeedbackController
    void reset();
  private:
    double  compute_control_signal(std::vector<double> states); // override LQRController function
    std::shared_ptr<GainsCollector> lqr_gains_safe_subs = NULL;
    std::shared_ptr<GainsCollector> lqr_gains_search_subs = NULL;
    std::vector<double> x_safe;
    double xdd_safe;
    double u;
    PrintThis verbosity;
    char buff[200];
    PrintMath verb_math;
    std::vector<double> read_F;
    int Nel;
  };
  

  class SinusoidalInput : public FeedbackController {
  public:
    SinusoidalInput(double w, double amp_a, double time_step);
    double compute_control_signal(std::vector<double> foo);
    void reset() {}
  private:
    double w;
    double amp_a;
    double time_step;
    double time_counter;
    double saturation;
  };



}

#endif // __FEEDBACK_CONTROL__