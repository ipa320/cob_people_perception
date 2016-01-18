/*
 * association.h
 *
 *  Created on: Jan 18, 2016
 *      Author: frm-ag
 */

#ifndef PEOPLE_FUSION_NODE_INCLUDE_PEOPLE_FUSION_NODE_ASSOCIATION_ASSOCIATION_H_
#define PEOPLE_FUSION_NODE_INCLUDE_PEOPLE_FUSION_NODE_ASSOCIATION_ASSOCIATION_H_

#include <people_fusion_node/tracker.h>
#include <people_fusion_node/detection/detection.h>

class Association{

  private:
    TrackerPtr tracker_;
    DetectionPtr detection_;

  public:
    Association(TrackerPtr tracker, DetectionPtr detection);

    TrackerPtr getTracker() const { return this->tracker_; };
    DetectionPtr getDetection() const { return this->detection_; };

};

// Define the Share pointer
typedef boost::shared_ptr<Association> AssociationPtr;



#endif /* PEOPLE_FUSION_NODE_INCLUDE_PEOPLE_FUSION_NODE_ASSOCIATION_ASSOCIATION_H_ */
