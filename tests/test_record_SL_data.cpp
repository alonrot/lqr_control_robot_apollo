#include "ros/ros.h" 
#include "include/read_write_interface.hpp"

#ifdef __XENO__
#include <native/task.h>
#include <sys/mman.h>
#endif

int run(int argc, char** argv)
{

	// Initialize a dummy class (representing estimate-states or feed-controller, etc.):
		// Inside the constructor: bind member variables to key-values of a yaml node
	std::shared_ptr<DummyClass> dummy_class_one = NULL;
	dummy_class_one.reset(new DummyClass());

	std::shared_ptr<DummyClassTwo> dummy_class_two = NULL;
	dummy_class_two.reset(new DummyClassTwo());

	// Initialize recording class:
		// Inside: bind member variables to addVarToCollect()
	std::shared_ptr<Rec_ForDummy> rec = NULL;
	rec.reset(new Rec_ForDummy(dummy_class_one->pars,dummy_class_two->pars));

  // Loop rate:
	ros::init(argc,argv,"dummy_node");
	ros::NodeHandle node;
  ros::Rate loop_rate(1000);

	// Write values inside the node's dummy class and
	// Update values reading them from the yaml node 

  while ( ros::ok() ){

  	// Do stuff:
		dummy_class_one->update_members(); 
		// Write on the yaml node:
		dummy_class_one->update_yaml_node();

		dummy_class_two->update_members();
		dummy_class_two->update_yaml_node(); 

		// Record from the Yaml node:
		rec->update_readings();

    // ros::spinOnce();
    loop_rate.sleep();
  }

}


int main(int argc, char** argv){
#ifdef __XENO__
  mlockall(MCL_CURRENT | MCL_FUTURE);
#endif
  run(argc,argv);
}