#ifndef PEOPLE_DUAL_PEOPLE_LEG_TRACKER_INCLUDE_DUAL_PEOPLE_LEG_TRACKER_ASSOCIATION_ASSOCIATORGNN_H_
#define PEOPLE_DUAL_PEOPLE_LEG_TRACKER_INCLUDE_DUAL_PEOPLE_LEG_TRACKER_ASSOCIATION_ASSOCIATORGNN_H_

// Own includes
#include <people_fusion_node/detection/detection.h>
#include <people_fusion_node/association/association.h>
#include <people_fusion_node/tracker.h>

#define DEBUG_ASSOCIATOR_GNN 1

class AssociatorGNN{

  private:
    double euclideanDistanceCost(TrackerPtr tracker, DetectionPtr detection);

  public:
    std::vector<AssociationPtr> associate(std::vector<DetectionPtr>& detections, std::vector<TrackerPtr>& trackers, std::vector<DetectionPtr>& notAssociatedDetections);

};



#endif /* PEOPLE_DUAL_PEOPLE_LEG_TRACKER_INCLUDE_DUAL_PEOPLE_LEG_TRACKER_ASSOCIATION_ASSOCIATORGNN_H_ */
