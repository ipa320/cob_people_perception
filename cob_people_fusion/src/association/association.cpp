#include <cob_people_fusion/association/association.h>

Association::Association(TrackerPtr tracker, DetectionPtr detection, double distance):
  tracker_(tracker),
  detection_(detection),
  distance_(distance)
{

}

std::ostream& operator<<(std::ostream &strm, const Association &association) {
  return strm << "Association: (Tracker[" << association.getTracker()->getId() << "] <--> Detection[" << association.getDetection()->getId() << "]" << " Distance: " << association.getDistance() << ")";
}

