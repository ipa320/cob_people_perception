/*
 * people_tracker.cpp
 *
 *  Created on: Apr 16, 2015
 *      Author: frm-ag
 */

#ifndef PEOPLE_TRACKER_H_
#define PEOPLE_TRACKER_H_

#include <leg_detector/people_tracker.h>

bool isValid(const PeopleTrackerPtr & o){
  return !o->isValid();
}

PeopleTracker::PeopleTracker(LegFeaturePtr leg0, LegFeaturePtr leg1){
  this->addLeg(leg0);
  this->addLeg(leg1);
}

LegFeaturePtr PeopleTracker::getLeg0(){
  return this->legs_[0];
}

LegFeaturePtr PeopleTracker::getLeg1(){
  return this->legs_[1];
}

bool PeopleTracker::addLeg(LegFeaturePtr leg){

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

bool PeopleTracker::isValid(){
  //std::cout << "Checking people tracker with the legs " << getLeg0()->id_ << "(" << getLeg0() << ")" << " - " << getLeg1()->id_ << "(" << getLeg1() << ")" << " for validity";
  if(getLeg0()->isValid() && getLeg1()->isValid()){
    //std::cout << " -> valid" << std::endl;
    return true;
  }
  //std::cout << " -> invalid" << std::endl;
  return false;
}

void PeopleTracker::updateProbabilities(){
  // Calculate the leg_distance probability
  double dist = LegFeature::distance(getLeg0(), getLeg1());
  //dist_probability_ =
}


PeopleTrackerList::PeopleTrackerList():
  list_(new std::vector<PeopleTrackerPtr>())
  {

  }

/**
 * Check if the PeopleTracker already exists in this list
 * @param The People Tracker
 * @return True if the Tracker exists, false otherwise
 */
bool PeopleTrackerList::exists(PeopleTrackerPtr peopleTracker){
  // Iterate through the People Tracker
  for(std::vector<PeopleTrackerPtr>::iterator peopleTrackerIt = list_->begin(); peopleTrackerIt != list_->end(); peopleTrackerIt++){
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
  list_->push_back(peopleTrackerPtr);
  return true;
}

int PeopleTrackerList::removeInvalidTrackers(){
  std::cout << "Removing invalid Trackers" << std::endl;

  int counter = 0;
  list_->erase(std::remove_if(list_->begin(), list_->end(), isValid),list_->end());

}

void PeopleTrackerList::printTrackerList(){
  std::cout << "TrackerList:" << std::endl;
  for(std::vector<PeopleTrackerPtr>::iterator peopleTrackerIt = list_->begin(); peopleTrackerIt != list_->end(); peopleTrackerIt++){
    std::cout << "PeopleTracker: " << (*peopleTrackerIt)->getLeg0()->id_ << " - " << (*peopleTrackerIt)->getLeg1()->id_ << std::endl;
  }
}

#endif /*PEOPLE_TRACKER_H_*/
