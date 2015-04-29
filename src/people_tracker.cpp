/*
 * people_tracker.cpp
 *
 *  Created on: Apr 16, 2015
 *      Author: frm-ag
 */


// Own includes
#include <leg_detector/people_tracker.h>
#include <dual_people_leg_tracker/math/math_functions.h>
#include <leg_detector/color_definitions.h>

/////////////////////////////////////////////////////////////
//// PeopleTracker Class Definitions
/////////////////////////////////////////////////////////////

bool isValid(const PeopleTrackerPtr & o){
  return !o->isValid();
}

PeopleTracker::PeopleTracker(LegFeaturePtr leg0, LegFeaturePtr leg1, ros::Time time):
  creation_time_(time),
  total_probability_(0.0) // Initialize the probability with zero
{
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

bool PeopleTracker::isTheSame(LegFeaturePtr legA, LegFeaturePtr legB){

  if(this->getLeg0()->int_id_ == legA->int_id_ && this->getLeg1()->int_id_ == legB->int_id_){
    return true;
  }

  if(this->getLeg1()->int_id_ == legA->int_id_ && this->getLeg0()->int_id_ == legB->int_id_){
    return true;
  }
  return false;
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

void PeopleTracker::update(ros::Time time){
  // Update the probabilities
  updateProbabilities(time);
}

void PeopleTracker::updateProbabilities(ros::Time time){
  // Calculate the leg_distance probability
  double dist = LegFeature::distance(getLeg0(), getLeg1());
  ROS_ASSERT(dist > 0.0);

  double leg_distance_threshold = 1.0;
  dist_probability_ = 1.0-sigmoid(dist,5,leg_distance_threshold);
  ROS_ASSERT(dist_probability_ >= 0.0 && dist_probability_ <= 1.0);

  ROS_DEBUG_COND(DEBUG_PEOPLE_TRACKER,"PeopleTracker::%s - Distance %f.3 Probability: %f.2",__func__, dist, dist_probability_);

  // Calculate the existenz of both LegTrackers
  double leg_time_threshold = 0.1;
  double min_leg_time = min(getLeg0()->getLifetime(), getLeg1()->getLifetime());

  leg_time_probability_ = sigmoid(min_leg_time,5,leg_time_threshold);

  //std::cout << "Min LegTime: " << min_leg_time << " Probability: " << leg_time_probability_ << std::endl;

  // Calculate the total probability

  total_probability_ = dist_probability_ * leg_time_probability_;

  // Print
#ifdef DEBUG_PEOPLE_TRACKER
  if(total_probability_ > 0.85){
    ROS_DEBUG_COND(DEBUG_PEOPLE_TRACKER,"%sPeopleTracker::%s\n|dist %.3f prob: %.2f| leg_time: %.2f prob: %.2f|| total_p: %.2f|",BOLDMAGENTA,__func__, dist, dist_probability_,min_leg_time, leg_time_probability_, total_probability_);
  }else{
    ROS_DEBUG_COND(DEBUG_PEOPLE_TRACKER,"PeopleTracker::%s\n|dist %.3f prob: %.2f| leg_time: %.2f prob: %.2f|| total_p: %.2f|",__func__, dist, dist_probability_,min_leg_time, leg_time_probability_, total_probability_);
  }
#endif
}


/////////////////////////////////////////////////////////////
//// PeopleTrackerList Class Definitions
/////////////////////////////////////////////////////////////

PeopleTrackerList::PeopleTrackerList():
  list_(new std::vector<PeopleTrackerPtr>())
  {

  }

/**
 * Check if a People Tracker allready exists for these two legs
 * @param legA The one leg
 * @param legB The other leg
 * @return True if it allready exists
 */
bool PeopleTrackerList::exists(LegFeaturePtr legA, LegFeaturePtr legB){
  // Iterate through the People Tracker
  for(std::vector<PeopleTrackerPtr>::iterator peopleTrackerIt = list_->begin(); peopleTrackerIt != list_->end(); peopleTrackerIt++){
    if((*peopleTrackerIt)->isTheSame(legA,legB))
      return true;
  }

  return false;
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
  //std::cout << "Removing invalid Trackers" << std::endl;

  int size_before = list_->size();

  list_->erase(std::remove_if(list_->begin(), list_->end(), isValid),list_->end());

  return size_before - list_->size();

}

void PeopleTrackerList::printTrackerList(){
  std::cout << "TrackerList:" << std::endl;
  for(std::vector<PeopleTrackerPtr>::iterator peopleTrackerIt = list_->begin(); peopleTrackerIt != list_->end(); peopleTrackerIt++){
    std::cout << "PeopleTracker: " << (*peopleTrackerIt)->getLeg0()->id_ << " - " << (*peopleTrackerIt)->getLeg1()->id_ << std::endl;
  }
}

void PeopleTrackerList::updateProbabilities(ros::Time time){
  for(std::vector<PeopleTrackerPtr>::iterator peopleTrackerIt = list_->begin(); peopleTrackerIt != list_->end(); peopleTrackerIt++){
    (*peopleTrackerIt)->updateProbabilities(time);
  }
}

