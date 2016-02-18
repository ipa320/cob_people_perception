#include <ros/ros.h>

// Cob messages
#include <cob_perception_msgs/Detection.h>
#include <cob_perception_msgs/DetectionArray.h>

// Own
#include <people_fusion_node/fusion_node.h>
#include <people_fusion_node/detector_config.h>
#include <people_fusion_node/visualization/color_definitions.h>
#include <people_fusion_node/consts.h>

// System includes
#include <iostream>
#include <XmlRpcValue.h>
#include <yaml-cpp/yaml.h>



// Command line parameters
int g_argc;
char** g_argv;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "people_fusion_node");
  g_argc = argc;
  g_argv = argv;

  // Read the parameter file from args
  std::string paramFile;
  if(argc > 1){
    paramFile = argv[1];
    std::cout << "Parameter file: " << paramFile << std::endl;
  }else{
    std::cout << "Missing parameter file!" << std::endl;
    return -1;
  }
  YAML::Node config = YAML::LoadFile(paramFile);


  std::vector<detector_config> detectors;
  std::stringstream error_stream;

  if(config["detector_nodes"].Type() == YAML::NodeType::Sequence)
  {

    bool errorFound = false;

    for (size_t i = 0; i < config["detector_nodes"].size(); ++i)
    {

      detector_config detector_cfg;

      YAML::Node detector_yaml = config["detector_nodes"][i];


      // Parse the detector

      // Get the name first, abort if no name is provided
      if(detector_yaml["name"]) detector_cfg.name = detector_yaml["name"].as<std::string>();
      else{ error_stream <<  "Detector misses name!" << std::endl; return -1; }

      // Get the topic
      if(detector_yaml["topic"]) detector_cfg.topic = detector_yaml["topic"].as<std::string>();
      else{
        error_stream <<  "Detector \"" << detector_cfg.name << "\" misses topic parameter!" << std::endl;
        errorFound = true;
      }

      // Get the weight
      if(detector_yaml["weight"]) detector_cfg.weight = detector_yaml["weight"].as<double>();
      else{
        error_stream <<  "Detector \"" << detector_cfg.name << "\" misses weight parameter!" << std::endl;
        errorFound = true;
      }

      // Get the weight
      if(detector_yaml["min_detections"]) detector_cfg.min_detections = detector_yaml["min_detections"].as<int>();
      else{
        error_stream <<  "Detector \"" << detector_cfg.name << "\" misses min_detections parameter!" << std::endl;
        errorFound = true;
      }

      // Get the weight
      if(detector_yaml["min_frequency"]) detector_cfg.min_frequency = detector_yaml["min_frequency"].as<double>();
      else{
        error_stream << "Detector \"" << detector_cfg.name << "\" misses min_frequency parameter!" << std::endl;
        errorFound = true;
      }

      detectors.push_back(detector_cfg);


    }

    // Abort if a error has been found
    if(errorFound) {std::cout << BOLDRED << error_stream.str() << RESET; return -1;}

    // Print the detectors
    std::cout << "Defined detectors:" << std::endl;
    for (size_t i = 0; i < detectors.size(); ++i)
    {
      std::cout << detectors[i] << std::endl;
    }

  }
  else
  {
    std::cout << "Error parsing the yaml file!" << std::endl;
    return -1;
  }

  double timeHorizon;
  if(config["time_horizon_sec"].Type() == YAML::NodeType::Scalar)
  {
    timeHorizon = config["time_horizon_sec"].as<double>();
  }
  else
  {
    std::cout << "Error parsing the yaml file!" << std::endl;
    return -1;
  }


  // Create the node handle
  ros::NodeHandle nh("people_fusion_node");

  FusionNode fn(nh, detectors, timeHorizon);

  ROS_INFO("people_fusion_node started!");

  ros::spin();

  return 0;
}

