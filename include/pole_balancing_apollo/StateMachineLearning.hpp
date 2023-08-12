#pragma once

#include <iostream>
#include <stdlib.h>
#include <vector>
#include <sstream>
#include "tools/PrintThis.hpp"

namespace state_machine_learning {

	class StateMachineLearning{
	public:
		StateMachineLearning(int K_exp);
		virtual ~StateMachineLearning(){}
		bool update(bool F_new_search, bool flag_unstability);
		int  get_machine_state();

		// Communication flags:
		// bool update_gains;
		bool reset_to_safe_controller;
		bool start_collecting_data;
		bool unstability_flag;
	private:
		int state_balancing_safe(bool F_new_search);
		int state_balancing_search(bool flag_unstability);
		int K_exp;
		std::vector<double> F_search_current;
		int Mstate;
		int counter_k;
		int counter_Nexp;
		PrintThis print_state_one;
		PrintThis print_state_two;
		PrintThis print_Nexp;
		std::string string_Nexp;
		char buff[200];
	};

}