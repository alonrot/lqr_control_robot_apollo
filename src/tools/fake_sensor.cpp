#include "tools/fake_sensor.hpp"

FakeSensor::FakeSensor(double noise_std, int sampling_steps, int delay_time_steps)
{

	// If time_steps = 0, then no delay is added

	// Initialize delayed signal:
	this->N_del = delay_time_steps + 1;

	// Reserve memory for the array:
	stored_signal.resize(this->N_del,0);

	// The current meaasurement get initialized to 0:
	this->current_measurement = 0.0;

	// Held value:
	this->held_value = 0.0;
	this->steps_counter = sampling_steps-1;
	this->steps = sampling_steps;

	// Gaussian noise, only if required:
	if( noise_std > 0 ){
		this->add_noise_flag = true;
  	this->rnd_generator = std::mt19937(this->rd());
  	this->gaussian_noise = std::normal_distribution<double>(0.0, noise_std);
	}
	else
		this->add_noise_flag = false;

}

double FakeSensor::measure(double true_value)
{
	double noisy_value, held_value, delayed_value;

	noisy_value 	= add_noise(true_value);
	held_value 		= hold(noisy_value);
	delayed_value = add_delay(held_value);
	return delayed_value;
} 

double FakeSensor::add_noise(double value)
{
	double noisy_value;

	if( this->add_noise_flag )
		noisy_value = value + this->gaussian_noise(this->rnd_generator);
	else
		noisy_value = value;

	return noisy_value;
}

double FakeSensor::hold(double value)
{

	if(this->steps > 1){
		++this->steps_counter;
		if(this->steps_counter == this->steps){
			this->held_value = value;
			this->steps_counter = 0;
		}
	}
	else
		this->held_value = value;

	return this->held_value;
}

double FakeSensor::add_delay(double value)
{
	// Shift all the values one place (only worth if there is delay to introduce):
	if(this->N_del > 1)
	{
		std::vector<double> aux;
		aux.resize(this->N_del,0);

		// Copy the vector to an auxiliar vector:
		copy_vector(this->stored_signal,aux);

		for(int i = 0;i < this->N_del-1; ++i)
			this->stored_signal[i+1] = aux[i];
	}

	// Add the new measurement:
	this->stored_signal[0] = value;

	// Return the last value (delayed N_del):
  return this->stored_signal[this->N_del-1];

}

void FakeSensor::copy_vector(std::vector<double> orig_vector, std::vector<double> dest_vector)
{
	for(int i = 0;i < this->N_del; ++i)
		dest_vector[i] = orig_vector[i];
}
