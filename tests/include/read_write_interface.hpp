#include "pole_balancing_apollo/parameters.hpp"

class DummyClass{
public:
  DummyClass();
  void update_members();
  void update_yaml_node();
  std::shared_ptr<parameters::YAML_data_share> pars;
private:
  double var1;
  double var2;
  int counter;
};

class DummyClassTwo{
public:
  DummyClassTwo();
  void update_members();
  void update_yaml_node();
  std::shared_ptr<parameters::YAML_data_share> pars;
private:
  double var3;
  double var4;
  int counter;
};


class Rec_ForDummy {
public:
Rec_ForDummy(	std::shared_ptr<parameters::YAML_data_share> pars_one,
							std::shared_ptr<parameters::YAML_data_share> pars_two);
void update_readings();
private:
const std::shared_ptr<parameters::YAML_data_share> pars_one; // Read-only
const std::shared_ptr<parameters::YAML_data_share> pars_two; // Read-only
double read_var1;
double read_var2;
double read_var3;
double read_var4;
};