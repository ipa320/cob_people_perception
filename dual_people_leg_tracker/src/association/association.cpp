/*
 * Association.cpp
 *
 *  Created on: Nov 19, 2015
 *      Author: frm-ag
 */

#include <dual_people_leg_tracker/association/association.h>
#include <dual_people_leg_tracker/leg_feature.h>
#include <dual_people_leg_tracker/detection/detection.h>

Association::Association(LegFeaturePtr leg, DetectionPtr detection, double assocation_probability) {
  this->leg_ = leg;
  this->detection_ = detection;
  this->association_probability_ = assocation_probability;
}

Association::~Association() {
  // TODO Auto-generated destructor stub
}

