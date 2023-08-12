#include "pole_balancing_apollo/feedback_control.h"
#include <gtest/gtest.h>



class feedback_control_tests : public ::testing::Test { 
protected:   
  void SetUp(){}
  void TearDown(){}
};


static std::vector<double> get_gains(){
  std::vector<double> gains;
  gains.push_back(1);
  gains.push_back(2);
  gains.push_back(3);
  return gains;
}

static std::vector<double> get_input(){
  std::vector<double> input;
  input.push_back(1);
  input.push_back(2);
  input.push_back(3);
  return input;
}


TEST_F(feedback_control_tests,test_basic_lqr_controller){

  std::vector<double> gains = get_gains();
  std::vector<double> input = get_input();

  feedback_control::LQRController controller(gains);
 
  double u = controller.get_control(input);
  ASSERT_EQ(u,14);

}

TEST_F(feedback_control_tests,test_saturation_lqr_controller){

  std::vector<double> gains = get_gains();
  std::vector<double> input = get_input();

  feedback_control::LQRController controller(gains);

  controller.set_saturation(10);
  double u = controller.get_control(input);
  ASSERT_EQ(u,10);

}

TEST_F(feedback_control_tests,test_gain_input_same_size_lqr_controller){

  std::vector<double> gains = get_gains();
  std::vector<double> input = get_input();

  gains.push_back(1);

  feedback_control::LQRController controller(gains);

  ASSERT_THROW(controller.get_control(input),std::runtime_error);

}

TEST_F(feedback_control_tests,test_reset_gains_lqr_controller){

  std::vector<double> gains = get_gains();
  std::vector<double> input = get_input();

  feedback_control::LQRController controller(gains);

  std::vector<double> new_gains;
  new_gains.push_back(0);
  new_gains.push_back(0);
  new_gains.push_back(0);

  controller.set_gains(new_gains);
  double u = controller.get_control(input);

  ASSERT_EQ(u,0);

  controller.reset_gains();
  u = controller.get_control(input);
  
  ASSERT_EQ(u,14);

}

TEST_F(feedback_control_tests,test_get_dimension_lqr_controller){

  std::vector<double> gains = get_gains();

  feedback_control::LQRController controller(gains);
 
  int dim = controller.get_dimension();

  ASSERT_EQ(dim,3);
  
}


TEST_F(feedback_control_tests,test_basic_pole_balancing){

  std::vector<double> gains;
  for(int i=0;i<5;i++) gains.push_back(1);

  std::shared_ptr<feedback_control::PoleBalancingDefaultController> controller(new feedback_control::PoleBalancingDefaultController(gains));

  double u = compute_default_control(controller,1,1,1,1,1);

}

TEST_F(feedback_control_tests,test_gain_size_pole_balancing){

  std::vector<double> gains;
  for(int i=0;i<5;i++) gains.push_back(1);
  gains.push_back(1);


  ASSERT_THROW(std::shared_ptr<feedback_control::PoleBalancingDefaultController> controller(new feedback_control::PoleBalancingDefaultController(gains)),
	       std::runtime_error);

}

TEST_F(feedback_control_tests,test_integral_pole_balancing){

  std::vector<double> gains;
  for(int i=0;i<5;i++) gains.push_back(1);

  std::shared_ptr<feedback_control::PoleBalancingDefaultController> controller(new feedback_control::PoleBalancingDefaultController(gains));

  double u1 = compute_default_control(controller,1,1,1,1,1);
  double u2 = compute_default_control(controller,1,1,1,1,1);
  
  // u2 smaller than u1 because of integral (controller returns negative value)
  //ASSERT_LT(u2,u1);

  // ki is set to zero, so should not ingrate
  gains[4]=0;
  controller->set_gains(gains);
  double u3 = feedback_control::compute_default_control(controller,1,1,1,1,1);

  ASSERT_EQ(u2,u3);

  // we reset the integral, so should be back to square 1
  gains[4]=1;
  controller->set_gains(gains);
  controller->reset();
  double u4 = feedback_control::compute_default_control(controller,1,1,1,1,1);
  
  ASSERT_EQ(u4,u1);

}


