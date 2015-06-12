/*
 * association_pair.h
 *
 *  Created on: Jun 8, 2015
 *      Author: frm-ag
 */

#ifndef PEOPLE_DUAL_PEOPLE_LEG_TRACKER_INCLUDE_DUAL_PEOPLE_LEG_TRACKER_JPDA_ASSOCIATION_PAIR_H_
#define PEOPLE_DUAL_PEOPLE_LEG_TRACKER_INCLUDE_DUAL_PEOPLE_LEG_TRACKER_JPDA_ASSOCIATION_PAIR_H_

#include <dual_people_leg_tracker/leg_feature.h>

class AssociationPair{
  public:
    DetectionPtr detection;
    LegFeaturePtr legFeature;

};

typedef boost::shared_ptr<AssociationPair> AssociationPairPtr;



#endif /* PEOPLE_DUAL_PEOPLE_LEG_TRACKER_INCLUDE_DUAL_PEOPLE_LEG_TRACKER_JPDA_ASSOCIATION_PAIR_H_ */
