#include "pole_balancing_apollo/parameters.hpp"


namespace parameters {

  YAML_pole_balancing_config::YAML_pole_balancing_config(std::string yaml_file_path) {

    try {
      this->node = YAML::LoadFile(yaml_file_path);
      this->read_file_failed=false;
    } catch(const std::exception& e){
      this->read_file_failed=true;
      this->error_message = e.what();
    }

    this->file_path = yaml_file_path;

  }


  bool YAML_pole_balancing_config::failed(std::string &error_message) {

    if (!this->read_file_failed) return false;
    error_message = this->error_message;
    return true;

  }


  YAML_data_share::YAML_data_share(){
    
  }

}
