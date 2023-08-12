#include "../include/read_write_interface.hpp"

Rec_ForDummy::Rec_ForDummy( std::shared_ptr<parameters::YAML_data_share> pars_one,
                            std::shared_ptr<parameters::YAML_data_share> pars_two) : 
                            pars_one(pars_one), pars_two(pars_two){

  this->read_var1 = 0.0;
  this->read_var2 = 0.0;

  this->read_var3 = 0.0;
  this->read_var4 = 0.0;

}

void
Rec_ForDummy::update_readings(){

  this->read_var1 = this->pars_one->get<double>("var1");
  this->read_var2 = this->pars_one->get<double>("var2");

  printf("this->read_var1 = %f\n",this->read_var1);
  printf("this->read_var2 = %f\n",this->read_var2);

  this->read_var3 = this->pars_two->get<double>("var3");
  this->read_var4 = this->pars_two->get<double>("var4");

  printf("this->read_var3 = %f\n",this->read_var3);
  printf("this->read_var4 = %f\n",this->read_var4);

}



DummyClass::DummyClass(){
  this->var1 = 0.0;
  this->var2 = 0.0;
  this->counter = 0;

  this->pars.reset(new parameters::YAML_data_share());
  this->pars->node["var1"] = this->var1;
  this->pars->node["var2"] = this->var2;
}

void
DummyClass::update_members(){

  ++(this->counter);
  this->var1 = sin((double)counter/10);
  this->var2 = 2*sin((double)counter/10);

}

void
DummyClass::update_yaml_node(){

  this->pars->node["var1"] = this->var1;
  this->pars->node["var2"] = this->var2;

}

DummyClassTwo::DummyClassTwo(){
  this->var3 = 0.0;
  this->var4 = 0.0;
  this->counter = 0;

  this->pars.reset(new parameters::YAML_data_share());
  this->pars->node["var3"] = this->var3;
  this->pars->node["var4"] = this->var4;
}

void
DummyClassTwo::update_members(){

  ++(this->counter);
  this->var3 = cos((double)counter/10);
  this->var4 = 2*cos((double)counter/10);

}

void
DummyClassTwo::update_yaml_node(){

  this->pars->node["var3"] = this->var3;
  this->pars->node["var4"] = this->var4;

}