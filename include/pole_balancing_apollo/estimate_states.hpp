#ifndef INCLUDE_ESTIMATE_STATES_H
#define INCLUDE_ESTIMATE_STATES_H

#include <iostream>
#include <random>
#include "ros/ros.h"
#include "rosrt/rosrt.h"
#include "pole_balancing_apollo/measurements.hpp"
#include "tools/butter_filt.hpp"
#include "tools/tools.hpp"
#include "tools/PrintThis.hpp"

class EstimateStates{

	public:
		virtual ~EstimateStates(){}
		virtual void 	update_states(CartesianState endeffector_state_measured, double measured_pole_angle)=0;
		virtual void  get_states(std::vector<double> & states)=0;

		// Other functions:
		virtual void get_endeff_pos_in_plane(std::vector<double> & endeff_pos){printf("No puede ser!!\n");};
		virtual bool within_state_space_limits(){printf("No puede ser!!!!!!\n");};
	    virtual void 	update_yaml_node(){printf("No puede ser!!\n");}
};

class EstimateStatesPole : public EstimateStates{

	public:
		EstimateStatesPole(	double plane_angle, double time_step,
												int which_filter_pole, int which_filter_xdd,
												int integral_action, std::vector<double> endeff_offset,
												std::vector<double> state_space_box_limits,
												double endeff_acc_limits);
		~EstimateStatesPole();
		void 	update_states(CartesianState endeffector_state_measured, double measured_pole_angle);
		void  get_states(std::vector<double> & states);
		void  get_endeff_pos_in_plane(std::vector<double> & endeff_pos); // Overrides the one from EstimateStatesPole
		bool 	within_state_space_limits();
	private:
		double 	differentiate(double pos);
		CartesianState transform_world2plane(CartesianState cart_base);
		double 	update_integrator_state(double endeff_pos_plane_x);
		
		CartesianState cart_plane;
		double rot_matrix[N_CART][N_CART];
		std::vector<double> states;
		const std::vector<double> endeff_offset;
		double time_step;
		int integral_action;
		double int_state;
		double pos_old;
		std::vector<double> state_space_box_limits;
		double endeff_acc_limits;
		double endeff_acc_filtered;
  	std::shared_ptr<ButterFilt> bfilter_angle_pos;
  	std::shared_ptr<ButterFilt> bfilter_angle_vel;
  	std::shared_ptr<ButterFilt> bfilter_xacc;
  	std::shared_ptr<Tools::SignalTreatment> signal_treatment;
  	bool verbo;
  	PrintThis verbosity;
  	char buff[200];

};

#endif /* INCLUDE_ESTIMATE_STATES_H */