#include "lqr_gains/ComputeLQRcost.hpp"

ComputeLQRcost::ComputeLQRcost(	Eigen::VectorXd Q_empirical,
																Eigen::VectorXd R_empirical,
																double J_heuristic){

	this->Nx = Q_empirical.size();
	this->Nu = R_empirical.size();

	// Error checking:
	if(this->Nx == 0){
		throw std::runtime_error("Size of Q_empirical is zero\n");
	}

	if(this->Nu == 0){
		throw std::runtime_error("Size of R_empirical is zero\n");
	}

	this->step_k = 0;
	this->J_k = 0.0;
	this->J_heuristic = J_heuristic;

	this->J_weights = Eigen::VectorXd::Zero(this->Nx + this->Nu);
	this->J_weights.head(this->Nx).array() = Q_empirical.array();
	this->J_weights.tail(this->Nu).array() = R_empirical.array();

	this->state = Eigen::VectorXd::Zero(this->Nx);
	this->J_vec = Eigen::VectorXd::Zero(this->Nx + this->Nu);
	this->J_vec_new = Eigen::VectorXd::Zero(this->Nx + this->Nu);

	// Start ROS node:
  // Get node name (has to be a std::string):
  std::string node_name("Compute_LQR_cost_node");

  // Arguments for node:
  int argc = 1; char* argv[1];
  argv[0] = new char[node_name.size() + 1];
  std::copy(node_name.begin(), node_name.end(), argv[0]);
  argv[0][node_name.size()] = '\0';

  // Initialize node:
  ros::init(argc,argv,node_name);

  struct rosrt::InitOptions options;
  options.pubmanager_thread_name = node_name;

  rosrt::init(options);

	this->nh = std::make_shared<ros::NodeHandle>();

  // Publisher:
	this->pub_cost.initialize(this->nh->advertise<pole_balancing_apollo::CostValue>("cost_value", 0), 100, pole_balancing_apollo::CostValue());

  // Allocate memory for the message:
  this->msg_cost = this->pub_cost.allocate();

}

void
ComputeLQRcost::update_cost(std::vector<double> state_in, double u){

	// Update step:
	++this->step_k;

	// Map into Eigen:
	for(size_t i = 0;i<this->Nx;++i)
		this->state(i) = state_in[i];

	// Compute next cost:
	this->J_vec_new.head(Nx).array() = this->state.array().square();
	this->J_vec_new.tail(Nu)(0) = std::pow(u,2); // This thinks Nu = 1

	// Recurse:
	this->J_vec.array() = (this->J_vec.array() * (this->step_k-1) + this->J_vec_new.array()) / this->step_k;

	// Compute cost value:
	this->J_k = this->J_vec.dot(this->J_weights);

	return;
}

void
ComputeLQRcost::reset_cost(){

	this->step_k = 0;
	this->J_k = 0.0;
	this->J_vec.setZero();
	this->J_vec_new.setZero();
	this->verbosity.print("Cost reseted!\n");

	return;
}

double
ComputeLQRcost::get_current_cost(){

	return this->J_k;
}

void
ComputeLQRcost::publish_current_cost(){

	this->msg_cost->cost_experiment = this->J_k;
	this->pub_cost.publish(this->msg_cost);
	sprintf(this->buff,"Published cost (current): %f\n",this->J_k);
	this->verbosity.print(this->buff);

	return;
}

void
ComputeLQRcost::publish_heuristic_cost(){

	this->msg_cost->cost_experiment = this->J_heuristic;
	this->pub_cost.publish(this->msg_cost);
	sprintf(this->buff,"Published cost (current): %f\n",this->J_k);
	this->verbosity.print(this->buff);

	return;
}