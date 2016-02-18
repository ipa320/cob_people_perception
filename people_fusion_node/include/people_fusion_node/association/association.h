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

/***
 * This class represents a association between a detection and a tracker
 */
class Association{

  private:
    TrackerPtr tracker_; /**< The associated tracker */
    DetectionPtr detection_; /**< The associated detection */
    double distance_; /**< Euclidean distance between tracker and detection */

    // For easy outstreaming
    friend std::ostream& operator<<(std::ostream&, const Association&);

  public:

    Association(TrackerPtr tracker, DetectionPtr detection, double distance_);

    /**
     * get the tracker
     * @return SharedPointer to the tracker
     */
    TrackerPtr getTracker() const { return this->tracker_; };

    /**
     * get the detection
     * @return SharedPointer to the detection
     */
    DetectionPtr getDetection() const { return this->detection_; };

    /**
     * get the distance between tracker and detection
     * @return distance
     */
    double getDistance() const { return this->distance_; };

};

// Define the Share pointer
typedef boost::shared_ptr<Association> AssociationPtr;



#endif /* PEOPLE_FUSION_NODE_INCLUDE_PEOPLE_FUSION_NODE_ASSOCIATION_ASSOCIATION_H_ */
