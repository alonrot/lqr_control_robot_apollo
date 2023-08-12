#include "lqr_gains/gather_LQR_gains_to_publish.hpp"
#include "yaml-cpp/yaml.h" 

#include <iostream>

int main(int argc, char** argv)
{
  #ifdef __XENO__
    mlockall(MCL_CURRENT | MCL_FUTURE);
  #endif

    // Get path to LQR gains:
    // Note: variable PATH2FOLDER_CONFIG_NODE is set in the CMakeLists.txt with the function set_target_properties(), specifically for this node
    std::string path_LQRgains(PATH2FOLDER_CONFIG_NODE);
    std::string config_file_name;

    printf("[INFO] ** Starting NODE Publisher_LQRgains\n");
    printf("[INFO] ** Number of passed input arguments: %i\n",argc);
    int narg_expected = 1;

    // Parse arguments:
    if(argc < narg_expected+1){
      printf("[ERROR] ** No yaml file specified...\n");
      printf("[ERROR] ** Should be located in PATH2FOLDER_CONFIG_NODE\n");
      printf("[ERROR] ** Pass the file name as input argument to the node\n");
      printf("[ERROR] ** Exit the node...\n");
      return 0;
    }
    else{
      config_file_name = argv[1];
    }

    // Get path to the config file:
    std::string path2configfile = PATH2FOLDER_CONFIG_NODE + config_file_name;

    std::cout << "Path to config yaml file = " << path2configfile << std::endl;

  /* Extract variable from YAML file */
    // Define yaml node:
    YAML::Node configfile = YAML::LoadFile(path2configfile);

    // Declare parameters:
    std::string file_LQRsafe("foo.you");
    std::string file_LQRsearch("foo.you");
    std::string node_name("/node_name");
    std::string topic_name_safe("/topic_name");
    std::string topic_name_search("/topic_name");
    int n_gains_safe;
    int n_gains_search;

    // Name of the files containing the LQR controllers:
    if (configfile["file_LQRsafe"]){
      file_LQRsafe = configfile["file_LQRsafe"].as<std::string>();
      std::cout << " ** LQR safe gains: " << file_LQRsafe << std::endl;
    }
    else {
      printf("[ERROR]: No QR safe gains parameter could be found in yaml file!\n");
      return 0;
    }

    if (configfile["file_LQRsearch"]){
      file_LQRsearch = configfile["file_LQRsearch"].as<std::string>();
      std::cout << " ** LQR search gains: " << file_LQRsearch << std::endl;
    }
    else {
      printf("[ERROR]: No LQR search gains parameter could be found in yaml file!\n");
      return 0;
    }

    // ROS node:
    if (configfile["node_name"]){
      node_name = configfile["node_name"].as<std::string>();
      std::cout << " ** Node name: " << node_name << std::endl;
    }
    else {
      printf("[ERROR]: No Node name could be found in yaml file!\n");
      return 0;
    }

    // Name of the ROS topics:
    if (configfile["topic_name_safe"]){
      topic_name_safe = configfile["topic_name_safe"].as<std::string>();
      std::cout << " ** Topic for safe gains: " << topic_name_safe << std::endl;
    }
    else {
      printf("[ERROR]: No Topic for safe gains parameter could be found in yaml file!\n");
      return 0;
    }

    if (configfile["topic_name_search"]){
      topic_name_search = configfile["topic_name_search"].as<std::string>();
      std::cout << " ** Topic for search gains: " << topic_name_search << std::endl;
    }
    else {
      printf("[ERROR]: No Topic for search gains parameter could be found in yaml file!\n");
      return 0;
    }

    if (configfile["n_gains_safe"]){
      n_gains_safe = configfile["n_gains_safe"].as<int>();
      std::cout << " ** Number of safe gains: " << n_gains_safe << std::endl;
    }
    else {
      printf("[ERROR]: No Number of safe gains parameter could be found in yaml file!\n");
      return 0;
    }

    if (configfile["n_gains_search"]){
      n_gains_search = configfile["n_gains_search"].as<int>();
      std::cout << " ** Number of search gains: " << n_gains_search << std::endl;
    }
    else {
      printf("[ERROR]: No Number of search gains parameter could be found in yaml file!\n");
      return 0;
    }

  /* Read gains from file */
  ReadLQRGains * readlqr_safe   = new ReadLQRGains(path_LQRgains,file_LQRsafe,topic_name_safe,n_gains_safe);
  ReadLQRGains * readlqr_search = new ReadLQRGains(path_LQRgains,file_LQRsearch,topic_name_search,n_gains_search);

  // Broadcasting data:
  GatherLQRGainsToPublish publisher_lqr_gains(readlqr_safe,readlqr_search,node_name);
  publisher_lqr_gains.run();
}