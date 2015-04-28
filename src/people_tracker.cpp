/*
 * people_tracker.cpp
 *
 *  Created on: Apr 16, 2015
 *      Author: frm-ag
 */

#ifndef PEOPLE_TRACKER_H_
#define PEOPLE_TRACKER_H_

#include <leg_detector/people_tracker.h>

PeopleTracker::PeopleTracker(LegFeature* leg0, LegFeature* leg1){
  this->addLeg(leg0);
  this->addLeg(leg1);
}

LegFeature* PeopleTracker::getLeg0(){
  return this->legs_[0];
}

LegFeature* PeopleTracker::getLeg1(){
  return this->legs_[1];
}

bool PeopleTracker::addLeg(LegFeature* leg){

  // Return false if this tracker already has two legs
  if(legs_.size() >= 2) return false;

  legs_.push_back(leg);
  return true;
}

bool PeopleTracker::isTheSame (PeopleTrackerPtr peopleTracker){

  if(this->getLeg0()->int_id_ == peopleTracker->getLeg0()->int_id_ && this->getLeg1()->int_id_ == peopleTracker->getLeg1()->int_id_){
    return true;
  }

  if(this->getLeg1()->int_id_ == peopleTracker->getLeg0()->int_id_ && this->getLeg0()->int_id_ == peopleTracker->getLeg1()->int_id_){
    return true;
  }

  return false;
}


/**
 * Check if the PeopleTracker allready exists in this list
 * @param The People Tracker
 * @return True if the Tracker exists, false otherwise
 */
bool PeopleTrackerList::exists(PeopleTrackerPtr peopleTracker){
  // Iterate through the People Tracker
  for(std::vector<PeopleTrackerPtr>::iterator peopleTrackerIt = list_.begin(); peopleTrackerIt != list_.end(); peopleTrackerIt++){
    if((*peopleTrackerIt)->isTheSame(peopleTracker))
      return true;
  }

  return false;

}

/**
 * Add a tracker to the list, no check is performed if this tracker allready exists!
 * @param peopleTrackerPtr Pointer to the tracker that is added to the list
 * @return true
 */
bool PeopleTrackerList::addPeopleTracker(PeopleTrackerPtr peopleTrackerPtr){
  list_.push_back(peopleTrackerPtr);
  return true;
}

#endif /*PEOPLE_TRACKER_H_*/
