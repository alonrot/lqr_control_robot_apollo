#include "tools/PrintThis.hpp"

/** default constructor and initializer */
PrintThis::PrintThis(){
  // By default, we print without limit and message has to be provided by the user:
  this->initialize("Message not defined",-1);
}

void
PrintThis::initialize(std::string msg_in,int print_limit){

  // Input arguments:
  this->my_msg = std::string(msg_in);
  this->print_limit = print_limit;

  // Rest:
  this->counter = 0;
  this->keep_printing = true;
  return;
}

PrintThis::PrintThis(std::string msg_in, int print_limit){
  this->initialize(msg_in,print_limit);
}

void
PrintThis::update_counter(void){

  ++this->counter;

  if(this->counter > this->print_limit){
      this->keep_printing = false;
      this->counter = 0;
  }
  return;
}

void
PrintThis::print(void){
  this->print(this->my_msg.c_str()); // c_str() is Real-time safe: it just takes the pointer
  return;
}

void
PrintThis::print(const char * msg_in){  // "std::string msg_in" has to be avoided. Also "const std::string & msg_in"
                                        // is a bad idea because we call this function with "my_message", 
                                        // which is not a std::string, but a char*, and it'll need to be 
                                        // converted to std::string, which involves memory allocation

  // Introduce zero if no printing should be made:
  if(this->print_limit == 0)
    return;

  if(print_limit != -1)
    this->update_counter();

  if(this->keep_printing){
    print_call(msg_in);
  }
  return;
}

void
PrintThis::print_call(const char * msg_in){
  
  #ifdef __XENO__
    rt_printf(msg_in);
  #else
    printf(msg_in);
  #endif

  return;
}