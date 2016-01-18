#include <people_fusion_node/association/association.h>
#include <people_fusion_node/association/associatorGNN.h>

std::vector<AssociationPtr>  AssociatorGNN::associate(std::vector<DetectionPtr>& detections, std::vector<TrackerPtr>& trackers, std::vector<DetectionPtr>& notAssociatedDetections){
  ROS_DEBUG_COND(DEBUG_ASSOCIATOR_GNN, "AssociatorGNN::%s - Number of detections: %i Number of trackers %i", __func__, (int) detections.size(), (int) trackers.size());

  std::vector<AssociationPtr> associations;

  // Calculate the costs
  for(std::vector<DetectionPtr>::iterator detectionIt = detections.begin();
      detectionIt < detections.end();
      detectionIt++)
  {
    bool associated = false;

    // Calculate the costs
    for(std::vector<TrackerPtr>::iterator trackerIt = trackers.begin();
        trackerIt < trackers.end();
        trackerIt++)
    {

      double distance = euclideanDistanceCost(*trackerIt, *detectionIt);
      std::cout << "Distance: " << distance << std::endl;

      if(distance < 0.3){
        associated = true;
        associations.push_back(AssociationPtr(new Association(*trackerIt, *detectionIt)));
      }
    }

    if(!associated){
      notAssociatedDetections.push_back(*detectionIt);
    }

  }
  return associations;
}

double AssociatorGNN::euclideanDistanceCost(TrackerPtr tracker, DetectionPtr detection){
  return (tracker->getCurrentState().getPos() - detection->getState().getPos()).length();
}

//ROS_DEBUG_COND(FUSION_NODE_DEBUG, "FusionNode::%s - Number of detections: %i", __func__, (int) detectionArray->detections.size());
