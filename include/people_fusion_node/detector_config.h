#ifndef PEOPLE_FUSION_NODE_INCLUDE_PEOPLE_FUSION_NODE_DETECTOR_CONFIG_H_
#define PEOPLE_FUSION_NODE_INCLUDE_PEOPLE_FUSION_NODE_DETECTOR_CONFIG_H_

//System includes
#include <string>
#include <iostream>

struct detector_config {
  std::string name;
  std::string topic;
  int min_detections;
  double min_frequency;
  double weight;
} ;

inline std::ostream& operator<<(std::ostream& os, const detector_config& detector_cfg) {
    return os << "Name: " << detector_cfg.name << std::endl
              << "Topic: " << detector_cfg.topic << std::endl
              << "Weight: " << detector_cfg.weight << std::endl
              << "Min_Frequency: " << detector_cfg.min_frequency << std::endl
              << "Min_Detections:" << detector_cfg.min_detections << std::endl;
}


#endif /* PEOPLE_FUSION_NODE_INCLUDE_PEOPLE_FUSION_NODE_DETECTOR_CONFIG_H_ */
