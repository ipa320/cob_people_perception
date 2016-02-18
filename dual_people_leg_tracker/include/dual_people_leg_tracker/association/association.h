/*
 * Association.h
 *
 *  Created on: Nov 19, 2015
 *      Author: frm-ag
 */

#ifndef PEOPLE_DUAL_PEOPLE_LEG_TRACKER_SRC_ASSOCIATION_ASSOCIATION_H_
#define PEOPLE_DUAL_PEOPLE_LEG_TRACKER_SRC_ASSOCIATION_ASSOCIATION_H_

#include <dual_people_leg_tracker/leg_feature.h>

class Association {

  private:
    LegFeaturePtr leg_;
    DetectionPtr  detection_;

    double association_probability_;

  public:
    Association(LegFeaturePtr, DetectionPtr, double);
    virtual ~Association();

    const DetectionPtr& getDetection() const {
      return detection_;
    }

    const LegFeaturePtr& getLeg() const {
      return leg_;
    }

    double getAssociationProbability() const {
      return this->association_probability_;
    }

    double getDistance() const {
      return (leg_->getEstimate().pos_ - detection_->getLocation()).length();
    }

    std::string toString() const{
      std::stringstream s;
      s << "<" << leg_->getIdStr() << " - " << detection_->getIdStr() << "|p: " << this->association_probability_ << ">";
      return s.str();
    }
};

#endif /* PEOPLE_DUAL_PEOPLE_LEG_TRACKER_SRC_ASSOCIATION_ASSOCIATION_H_ */
