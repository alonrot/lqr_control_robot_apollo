#ifndef INCLUDE_FAKE_SENSOR_H
#define INCLUDE_FAKE_SENSOR_H

#ifdef __XENO__
#include <native/task.h>
#include <sys/mman.h>
#endif

#include <string>
#include <iostream>
#include <random>
#include <cstdio>

class FakeSensor {

public:
	FakeSensor(double noise_std, int sampling_steps, int delay_time_steps);
	~FakeSensor(){}
	double 	measure(double true_value);

private:
	double 	add_noise(double value);
	double 	add_delay(double value);
	void 		copy_vector(std::vector<double> orig_vector, std::vector<double> dest_vector);
	double  hold(double value);
	int 		N_del;
	bool 		add_noise_flag;
	double 	current_measurement;
	std::vector<double> stored_signal;
	double 	held_value;
	int    	steps_counter;
	int    	steps;
  std::random_device rd;
  std::mt19937 rnd_generator; 
  std::normal_distribution<double> gaussian_noise;


};

#endif /* INCLUDE_FAKE_SENSOR_H */