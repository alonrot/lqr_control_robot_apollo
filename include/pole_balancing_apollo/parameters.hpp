#pragma once

#include <string>
#include "yaml-cpp/yaml.h" 
#include <iostream> 
#include <fstream>
#include <Eigen/Dense>

namespace parameters {


  class YAML_pole_balancing_config {
  public:
    YAML_pole_balancing_config(std::string yaml_file_path);

    template<typename T> T get(std::string key) const {
      if (this->node[key]) return this->node[key].as<T>();
      std::string error = key+" parameter not found in "+this->file_path;
      throw std::runtime_error(error);
    }

    template<typename T> T get(const char *key) const {
      return this->get<T>(std::string(key));
    }
    
    bool failed(std::string &error_message);
  private:
    YAML::Node node;
    std::string error_message;
    bool read_file_failed;
    std::string file_path;
  };

  class YAML_data_share {

  public:
    YAML_data_share();
    // Define get() template in the header, to be read at compilation time:
    template<typename T> T get(std::string key) const {
      if (this->node[key]) 
        return this->node[key].as<T>();
      std::string error = key+" parameter not found";
      throw std::runtime_error(error);
    }
    YAML::Node node;
  private:


  };




}


// Template to be able to read Eigen::VectorXd and Eigen::MatrixXd
// Based on the Tutorial https://github.com/jbeder/yaml-cpp/wiki/Tutorial
namespace YAML {
template<> 
struct convert<Eigen::VectorXd> {

  static Node encode(const Eigen::VectorXd & vec) {
    Node node;

    for(int i=0;i<vec.size();++i)
      node.push_back(vec(i));

    return node;
  }

  static bool decode(const Node& node, Eigen::VectorXd & vec) 
  {
    if(!node.IsSequence()) 
      return false;

    // Initialize:
    vec = Eigen::VectorXd::Zero(node.size());

    // Fill:
    for(size_t i=0;i<node.size();++i)
      vec(i) = node[i].as<double>();

    return true;
  }

};

template<> 
struct convert<Eigen::VectorXi> {

  static Node encode(const Eigen::VectorXi & vec) {
    Node node;

    for(int i=0;i<vec.size();++i)
      node.push_back(vec(i));

    return node;
  }

  static bool decode(const Node& node, Eigen::VectorXi & vec) 
  {
    if(!node.IsSequence()) 
      return false;

    // Initialize:
    vec = Eigen::VectorXi::Zero(node.size());

    // Fill:
    for(size_t i=0;i<node.size();++i)
      vec(i) = node[i].as<double>();

    return true;
  }

};

template<> 
struct convert<Eigen::MatrixXd> {

  static Node encode(const Eigen::MatrixXd & mat) {
    Node node;

    for(int i=0;i<mat.rows();++i){
      for(int j=0;j<mat.cols();++j){
        node[i].push_back(mat(i,j));
      }
    }

    return node;
  }

  static bool decode(const Node& node, Eigen::MatrixXd & mat) 
  {
    if(!node.IsSequence()) 
      return false;

    size_t Nrows = node.size();
    size_t Ncols = node[0].size();

    // Initialize:
    mat = Eigen::MatrixXd::Zero(Nrows,Ncols);

    // Fill:
    for(size_t i=0;i<Nrows;++i){
      for(size_t j=0;j<Ncols;++j){
        mat(i,j) = node[i][j].as<double>();
      }
    }

    return true;
  }

};

} // namespace YAML
