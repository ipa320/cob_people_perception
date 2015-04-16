/*
 * people_tracker.cpp
 *
 *  Created on: Apr 16, 2015
 *      Author: frm-ag
 */

#ifndef PEOPLE_TRACKER_H_
#define PEOPLE_TRACKER_H_

#include <leg_detector/people_tracker.h>

bool PeopleTracker::addLeg(SavedFeature* leg){

  // Return false if this tracker already has two legs
  if(legs_.size() >= 2) return false;

  legs_.push_back(leg);
  return true;

}

#endif /*PEOPLE_TRACKER_H_*/
