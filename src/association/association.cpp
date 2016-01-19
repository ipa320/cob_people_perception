#include <people_fusion_node/association/association.h>

Association::Association(TrackerPtr tracker, DetectionPtr detection, double distance):
  tracker_(tracker),
  detection_(detection),
  distance_(distance)
{

}
