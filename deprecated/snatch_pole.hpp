#ifndef INCLUDE_SNATCH_POLE_H
#define INCLUDE_SNATCH_POLE_H

#include "parameters_initialization.hpp"
#include "pole_balancing_apollo/pole_angle.hpp"
#include "pole_balancing_apollo/fake_sensor.hpp"
#include <iostream>
#include <random>

#define N_CART 3
// #define _x_ 0
// #define _y_ 1
// #define _z_ 2

typedef struct {
	std::shared_ptr<FakeSensor> x[N_CART];
	std::shared_ptr<FakeSensor> xd[N_CART];
	std::shared_ptr<FakeSensor> xdd[N_CART];
}SensorCartesianState;

// class Observation{
// 	public:
// 		virtual void update_observation()=0;
// 		virtual void get_observation()=0;

// };

class SnatchPole{// : public Observation{

	public:
		virtual void 		update(const CartesianState endeffector_state_raw, const double u_send)=0;
		virtual double	get_measured_pole_angle()=0;
		virtual double	get_visualization_pole_angle()=0;
		virtual CartesianState	get_measured_endeffector()=0;

	private:
		virtual void 		update_endeff_readings(const CartesianState endeffector_state_raw)=0;
		virtual void  	update_pole_readings(double u_send)=0;

};

class SnatchPoleVicon : public SnatchPole{

	public:
		SnatchPoleVicon(std::shared_ptr<ParametersInitialization> par);
		~SnatchPoleVicon();
		void 		update(	const CartesianState endeffector_state_raw, //update_pole_readings() update_endeff_readings()
										const double u_send);
		double	get_measured_pole_angle();
		double	get_visualization_pole_angle();
		CartesianState	get_measured_endeffector();

	private:
		void 		update_endeff_readings(const CartesianState endeffector_state_raw);
		void  	update_pole_readings(const double u_send); //get_new_measurement() update_measurements() new_measurement_available() new_meas_simulation() new_meas_realtime()
		void 		from_simulated_pole(const double u_send);
		void 		from_real_pole(void);
		double 	measured_pole_angle;
		double 	visualization_pole_angle;
    CartesianState endeffector_state_measured;
    SensorCartesianState endeffector_state_sensor;
		std::shared_ptr<ParametersInitialization> par;
		std::shared_ptr<Pole_angle> pole_angle_getter; 	// Object providing angle of pole during runtime
		std::shared_ptr<FakeSensor> angle_sensor;							// Object used for faking delay, noise or low sampling rate in the measurements
		KineticState simu_pole;
    KineticState simu_pole_new;
    std::random_device rd;
    std::mt19937 rnd_generator; 
    std::normal_distribution<double> gaussian_noise;

};

#endif /* INCLUDE_SNATCH_POLE_H */