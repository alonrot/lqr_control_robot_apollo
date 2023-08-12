#include "tools/tools.hpp"
#include "pole_balancing_apollo/StateMachineLearning.hpp"

namespace state_machine_learning {

	StateMachineLearning::StateMachineLearning(int K_exp){

		// Maximum experiment time (in steps):
		this->K_exp = K_exp;

		// Counter of experiments:
		this->counter_Nexp = 0;
		// this->string_Nexp = std::string("Experiment nr.");

		// Initial state:
		this->Mstate = 0;
		this->counter_k = 0;

		// Communication flags:
		this->start_collecting_data = false;
		this->reset_to_safe_controller = false;
		this->unstability_flag = false;

		// Smart printer:
		this->print_state_one.initialize("[DBG]: one_switch_two\n",-1);
		this->print_state_two.initialize("[DBG]: two_switch_one\n",-1);

	}

	bool
	StateMachineLearning::update(bool F_new_search, bool is_stable){

    bool update_successful = true;

    switch(this->Mstate){
      case 0: // Wait for ES to kick start the task:
        this->Mstate = state_balancing_safe(F_new_search);
        break;
      case 1: // Wait for the experiment to finish
        this->Mstate = state_balancing_safe(F_new_search);
        break;
      case 2: // Wait for ES flag
        this->Mstate = state_balancing_search(is_stable);
        break;
      case 3:
        update_successful = false;
        break; 
    }

    return update_successful;
	}

	int  
	StateMachineLearning::get_machine_state(){
		return this->Mstate;
	}

	int
	StateMachineLearning::state_balancing_safe(bool F_new_search){

		int next_state;

		// Check whether the controller has changed externally:
		if(F_new_search){

			// Set communication flag:
			this->start_collecting_data = true;

			// Update state:
			next_state = 2;

			// Starting experiment:
			++this->counter_Nexp;

			// Verbosity:
			sprintf(this->buff," Experiment Nr. %i\n",this->counter_Nexp);
			this->print_Nexp.print(this->buff);
			this->print_state_one.print();
		}
		else{

			// Keep state:
			next_state = 1;
		}

		return next_state;

	}

	int
	StateMachineLearning::state_balancing_search(bool is_stable){

		int next_state;

		// Check whether the controller has changed externally:
		if( this->counter_k > this->K_exp || !is_stable ){

			// Reset variables:
			this->counter_k = 0;
			this->reset_to_safe_controller = true;

			// Unstability:
			if(!is_stable)
				this->unstability_flag = true;

			// Update state:
			next_state = 1;

			this->print_state_two.print();

		}
		else{

			// Update experiment counter:
			++this->counter_k;

			// Keep state:
			next_state = 2;
		}

	return next_state;
	}


} // namespace state_machine_learning