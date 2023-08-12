#include "pole_balancing_apollo/parameters.hpp"
#include <gtest/gtest.h>



class parameters_tests : public ::testing::Test { 
protected:   
  void SetUp(){
    YAML::Node node;
    node["test_double"] = 0.1;
    node["test_float"] = 0.2;
    node["test_bool"] = true;
    node["test_str"] = "test";
    node["test_int"] = 1;
    node["gains"].push_back(1.0); 
    node["gains"].push_back(0.1); 
    node["gains"].push_back(0.01);
    std::ofstream fout(YAML_CONFIG_TEST_FILE);     
    fout<<node;
  }
  void TearDown(){
    std::remove(YAML_CONFIG_TEST_FILE);
  }
};




TEST_F(parameters_tests,test_read_file){
  parameters::YAML_pole_balancing_config config(YAML_CONFIG_TEST_FILE);
  std::string error;
  ASSERT_EQ(config.failed(error),false);
}

TEST_F(parameters_tests,fail_on_non_existing_file){
  parameters::YAML_pole_balancing_config config("imaginary path");
  std::string error;
  ASSERT_EQ(config.failed(error),true);
}

TEST_F(parameters_tests,raise_exception_on_non_existing_param){
  parameters::YAML_pole_balancing_config config(YAML_CONFIG_TEST_FILE);
  ASSERT_THROW(config.get<int>("imaginary_key"),std::runtime_error);
}

TEST_F(parameters_tests,read_simple_params){
  parameters::YAML_pole_balancing_config config(YAML_CONFIG_TEST_FILE);
  double test_double = config.get<double>("test_double");
  float test_float = config.get<float>("test_float");
  bool test_bool = config.get<bool>("test_bool");
  std::string test_str = config.get<std::string>("test_str");
  int test_int = config.get<int>("test_int");
  std::vector<double> gains = config.get< std::vector<double> >("gains");
  ASSERT_EQ(test_double,0.1);
  ASSERT_EQ(test_float,(float)0.2);
  ASSERT_EQ(test_bool,true);
  ASSERT_EQ(test_str,"test");
  ASSERT_EQ(test_int,1);
  ASSERT_EQ(gains[0],1.0);
  ASSERT_EQ(gains[1],0.1);
  ASSERT_EQ(gains[2],0.01);
}


