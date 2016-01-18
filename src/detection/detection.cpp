#include <people_fusion_node/detection/detection.h>

Detection::Detection(double x, double y, ros::Time time, unsigned int id):
  id_(id),
  state_(x, y),
  detectionTime_(time)
{}

std::ostream& operator<<(std::ostream &strm, const Detection &detection) {
  return strm << "Detection[" << detection.getId() << "]" << detection.getState();
}
