#ifndef INCLUDE_MEASUREMENTS_H
#define INCLUDE_MEASUREMENTS_H

#include <iostream>
#include <random>
#include "pole_angle_measurement/subscriber_pole_angle.hpp"
#include "pole_balancing_apollo/parameters.hpp"
#include "tools/fake_sensor.hpp"
#include "tools/tools.hpp"
#include "pole_balancing_apollo/pole_physical_parameters.h"

#include "ros/ros.h" 
#include "ros/master.h"
#include "ros/param.h"
#include "rosrt/rosrt.h"
#include "std_msgs/Float32.h"

#ifdef __XENO__
#include <native/task.h>
#include <sys/mman.h>
#endif

class Measurements{
	public:
		Measurements();
		virtual ~Measurements(){}
		virtual void 	update_measurements()=0;
		virtual void  publish_pole_angle_for_visualization()=0;
		virtual void 	update_yaml_node(){
			printf("@Measurements::update_yaml_node() - No puede ser.\n");}
		virtual void  read_endeff_acceleration(double endeff_accel){};

		// Add here whatever variables should be public across the children of Measurements, and outside this class
		double 	measured_pole_angle;
		double 	visualization_pole_angle;
};

class SnatchPole : public virtual Measurements{

	public:
		// virtual ~SnatchPole(){}
		void 	update_measurements(); // Wrapper that calls update_pole_angle_measurements()
		void  publish_pole_angle_for_visualization();

	protected:
		SnatchPole(std::string topic_name_pole_angle);
		virtual void  update_pole_angle_measurements()=0;
	private:
		#ifdef __XENO__
			rosrt::Publisher<std_msgs::Float32> publisher_pole_angle_rt;
			std_msgs::Float32Ptr pole_angle_msg_rt;
		#else
			ros::Publisher 		publisher_pole_angle;
			std_msgs::Float32 	pole_angle_msg;
		#endif

};

class SnatchPoleSimulation : public SnatchPole{

	public:
		SnatchPoleSimulation(	double sigma_n, int vision_Ts, int VISION_LATENCY,
													double initial_angle_sim, double time_step, double g_n, PoleParameters pole_para,
													std::string topic_name_pole_angle_visualization);
		void read_endeff_acceleration(double endeff_accel);

	private:
		void  	update_pole_angle_measurements();
		double control_input;

		// Particular variables:
    double time_step;
    double g_n;
		std::shared_ptr<FakeSensor> angle_sensor; // Object used for faking delay, noise or low sampling rate in the measurements
		KineticState simu_pole;
    KineticState simu_pole_new;
    PoleParameters pole_para;
    bool verbo;
};

class SnatchPoleVicon : public SnatchPole{

	public:
		SnatchPoleVicon(int sign_conv, double initial_angle_vicon, std::string topic_pole_angle_visualization, std::string topic_read_pole_angle);
		~SnatchPoleVicon(){}

	private:
		void  	update_pole_angle_measurements();
		std::shared_ptr<Subscriber_pole_angle> pole_angle_subscriber; 	// Object providing angle of pole during runtime
		int sign_conv;
		double initial_angle_vicon;

};

class SnatchPoleKinect : public SnatchPole{
	
	public:
		SnatchPoleKinect();

	private:
		void  	update_pole_angle_measurements();

};


/* TESTING */
class MeasurementsTest : public Measurements{
	public:
		MeasurementsTest();
		void update_measurements(){}
};


#endif /* INCLUDE_MEASUREMENTS_H */