#include <people_fusion_node/detection/detection.h>

Detection::Detection(double x, double y, ros::Time time, unsigned int id, std::string type):
  id_(id),
  state_(x, y),
  detectionTime_(time),
  detection_type_(type)
{}

std::ostream& operator<<(std::ostream &strm, const Detection &detection) {
  return strm << "Detection[" << detection.getId() << "]";
}
