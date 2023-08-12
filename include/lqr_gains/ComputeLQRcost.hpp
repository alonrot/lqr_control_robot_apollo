#ifndef __COMPUTE_LQR_COST_H__
#define __COMPUTE_LQR_COST_H__

#include <Eigen/Dense>
#include "ros/ros.h"
#include "rosrt/rosrt.h"
#include <std_msgs/Float64.h>
#include <string>
#include <iostream>
#include "pole_balancing_apollo/CostValue.h"
#include "tools/PrintThis.hpp"

#ifdef __XENO__
#include <native/task.h>
#include <sys/mman.h>
#endif

class ComputeLQRcost {

public:
	ComputeLQRcost(	Eigen::VectorXd Q_diag,
									Eigen::VectorXd R_diag,
									double J_heuristic);
	virtual ~ComputeLQRcost(){}

	void update_cost(std::vector<double> state_in, double u);
	void reset_cost();
	double get_current_cost();
	double return_heuristic_cost();
	void publish_current_cost();
	void publish_heuristic_cost();

private:
	size_t step_k;
	double J_k;
	double J_heuristic;
	Eigen::VectorXd J_vec;
	Eigen::VectorXd J_vec_new;
	Eigen::VectorXd J_weights;
	Eigen::VectorXd state;
	size_t Nx, Nu;

	std::shared_ptr<ros::NodeHandle> nh;
	rosrt::Publisher<pole_balancing_apollo::CostValue> pub_cost;
	pole_balancing_apollo::CostValuePtr msg_cost;
	PrintThis verbosity;
	char buff[200];

};

#endif /* __COMPUTE_LQR_COST_H__ */
