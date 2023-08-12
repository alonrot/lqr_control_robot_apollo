// Copyright (c) 2015 Max Planck Society

/*!@file
 * @author  Alonso Marco <alonso.marco@tuebingen.mpg.de>
 *
 * @date    2017-01-12
 *
 * @details 
 * Pole Balancing interface/library, to be called from
 * an external SL task.
 *
 */

#include "pole_balancing_apollo/measurements.hpp"

Measurements::Measurements(){

  // Recording data:
  this->measured_pole_angle = 0.0;
  this->visualization_pole_angle = 0.0;
}

SnatchPole::SnatchPole(std::string topic_pole_angle_visualization) : 
                        Measurements() {

  int argc = 1; char* argv[1]; 
  char name[] = "snatch_pole"; argv[0] = name;

  #ifdef __XENO__ // Real time publisher for Xenomai

    // We cannot use rt_task_shadow(NULL, name, 1, 0), as the following code
    // // int error_msg = rt_task_shadow(NULL, name, 1, 0); // Not really needed. It works without it
    // // if (error_msg)
    // //   ROS_ERROR("rosrt::thread - Couldn't spawn xenomai thread %s, error code = %d", name, error_msg);
    // returns the error_msg = -16, i.e., -EBUSY: the current Linux task is already mapped to a Xenomai context

    // Real-time node options:
    struct rosrt::InitOptions options_rt;
    options_rt.pubmanager_thread_name = "snatch_pole_RT";

    // Initialize non-real time ros node:
    ros::init(argc,argv,name);

    // Initialize non-real time ros node:
    rosrt::init(options_rt);

    // Node handle:
    ros::NodeHandle node;

    // Reset thread info:
    rosrt::resetThreadAllocInfo();

    // Initialize publisher. When using the default constructor (a we are, when declaring publisher_pole_angle_rt as a memeber of this class), it must be initialize with initialize().
    this->publisher_pole_angle_rt.initialize(node.advertise<std_msgs::Float32>(topic_pole_angle_visualization, 1), 10, std_msgs::Float32());

    // Allocate memory for the message. This is of special importance in real-time tasks:
    this->pole_angle_msg_rt = this->publisher_pole_angle_rt.allocate();

    // Publish for the first time:
    if(this->pole_angle_msg_rt){
      this->pole_angle_msg_rt->data = this->visualization_pole_angle;
      this->publisher_pole_angle_rt.publish(this->pole_angle_msg_rt);
    }

  #else // Non-real time publisher

    // Initialize non-real time ros node:
    ros::init(argc,argv,name);
    ros::NodeHandle node;
    this->publisher_pole_angle = node.advertise<std_msgs::Float32>(topic_pole_angle_visualization,5);

    // Show the pole for the first time:
    this->pole_angle_msg.data = this->visualization_pole_angle;
    this->publisher_pole_angle.publish(pole_angle_msg);
    
  #endif

}

void
SnatchPole::update_measurements(){

  this->update_pole_angle_measurements();

}

void
SnatchPole::publish_pole_angle_for_visualization(){

  // Copy into the std_msgs whatever is in visualization_pole_angle, 
  // which should be filled in by some of the children functions

  // Publish:
  #ifdef __XENO__
    this->pole_angle_msg_rt->data = this->visualization_pole_angle;
    this->publisher_pole_angle_rt.publish(this->pole_angle_msg_rt);
  #else
    this->pole_angle_msg.data = this->visualization_pole_angle;
    this->publisher_pole_angle.publish(this->pole_angle_msg);
  #endif

}

/* SnatchPoleSimulation class */
SnatchPoleSimulation::SnatchPoleSimulation( double sigma_n, int vision_Ts, int VISION_LATENCY,
                                            double initial_angle_sim, double time_step, double g_n, PoleParameters pole_para,
                                            std::string topic_pole_angle_visualization) : SnatchPole(topic_pole_angle_visualization) {
  int i;
  /* General variables*/

  /* Particular variables*/
    // Sensor of the pole angle:
    this->angle_sensor = std::make_shared<FakeSensor>(sigma_n,vision_Ts,VISION_LATENCY);

    // Simulated pole kinetic states:
    simu_pole.th = initial_angle_sim;
    simu_pole.thd = 0.0;
    simu_pole_new.th = 0.0;
    simu_pole_new.thd = 0.0;

    // Copy parameters:
    this->time_step = time_step;
    this->g_n = g_n;
    this->pole_para = pole_para; 

    // Measured angle:
    this->measured_pole_angle       = 0.0;
    this->visualization_pole_angle  = initial_angle_sim;

    // Current cart acceleration:
    this->control_input = 0.0;

    this->verbo = false;

}

void
SnatchPoleSimulation::update_pole_angle_measurements(){

  /* Simulate pole dynamics */

    // Forward Euler integration:
    this->simu_pole_new.th = this->simu_pole.th + time_step * this->simu_pole.thd;
    this->simu_pole_new.thd = this->simu_pole.thd + time_step * (
                              pole_para.mass * g_n * pole_para.center_of_mass / pole_para.inertia * sin(this->simu_pole.th)
                              - pole_para.mass * pole_para.center_of_mass * this->control_input / pole_para.inertia * cos(this->simu_pole.th)
                              - pole_para.damping * this->simu_pole.thd / pole_para.inertia);

    // Write back new values in simu_pole:
    this->simu_pole.th  = this->simu_pole_new.th;
    this->simu_pole.thd = this->simu_pole_new.thd;

  /* Fake noise, delay and acquisition rate */ 
    this->measured_pole_angle = this->angle_sensor->measure(this->simu_pole.th);
    this->visualization_pole_angle = this->simu_pole.th;

}

void
SnatchPoleSimulation::read_endeff_acceleration(double endeff_accel){
  this->control_input = endeff_accel;
}

/* SnatchPoleVicon class */
SnatchPoleVicon::SnatchPoleVicon( int sign_conv, double initial_angle_vicon, std::string topic_pole_angle_visualization, std::string topic_read_pole_angle)
                                  : SnatchPole(topic_pole_angle_visualization) {
  int i;

  /* Particular variables*/
    // Measured angle:
    this->measured_pole_angle       = 0.0;
    this->visualization_pole_angle  = 0.0;

    // Object for getting the pole angle:
    this->pole_angle_subscriber = std::make_shared<RosRT_Subscriber_pole_angle>(topic_read_pole_angle);

    // Copy parameters:
    this->sign_conv = sign_conv;
    this->initial_angle_vicon = initial_angle_vicon;

}


void
SnatchPoleVicon::update_pole_angle_measurements(){

  double angle_from_vision_system = this->pole_angle_subscriber->get();

  if( this->measured_pole_angle != (double)this->sign_conv*(angle_from_vision_system - this->initial_angle_vicon) )
  {  
    this->measured_pole_angle       = (double)this->sign_conv*( angle_from_vision_system - this->initial_angle_vicon );
    this->visualization_pole_angle  = this->measured_pole_angle;
  }


}

/* TESTING */
MeasurementsTest::MeasurementsTest(){

  int i;

  this->measured_pole_angle = 0;
  this->visualization_pole_angle = 0; 
}
