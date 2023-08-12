#ifndef READ_LQR_GAINS_H
#define READ_LQR_GAINS_H

#include "ros/ros.h"
#include "ros/param.h"
#include "rosrt/rosrt.h"
#include <std_msgs/Float64MultiArray.h>

#ifdef __XENO__
#include <native/task.h>
#include <sys/mman.h>
#endif

#include <iostream>
#include <fstream>
#include <string>

class ReadLQRGains{

public:

  ReadLQRGains(std::string fpath, std::string fname, std::string TOPIC, int n_gains);
  ~ReadLQRGains();
  bool get_gains(void);
  int     n_gains;
  double 	* set_of_gains;
  char   	* fname;
	char 		*	TOPIC;

private:
  bool read_from_file(void);
  char * cast_string_to_char(std::string my_string);
  char   	* fpath;

};

#endif /* READ_LQR_GAINS_H */