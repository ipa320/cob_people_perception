/*
 * people_tracker.cpp
 *
 *  Created on: Apr 16, 2015
 *      Author: frm-ag
 */

#include <ros/console.h>
// Own includes
#include <leg_detector/people_tracker.h>
#include <dual_people_leg_tracker/math/math_functions.h>
#include <leg_detector/color_definitions.h>

/////////////////////////////////////////////////////////////
//// PeopleTracker Class Definitions
/////////////////////////////////////////////////////////////

// Helper function to delete invalid People Tracker from container objects
bool isValidPeopleTracker(const PeopleTrackerPtr & o){
  return !o->isValid();
}


PeopleTracker::PeopleTracker(LegFeaturePtr leg0, LegFeaturePtr leg1, ros::Time time):
  creation_time_(time),
  total_probability_(0.0) // Initialize the probability with zero
{
  // Add the legs to this people tracker
  this->addLeg(leg0);
  this->addLeg(leg1);

  // Set the id
  if(leg0->int_id_ < leg1->int_id_){
    id_[0] = leg0->int_id_;
    id_[1] = leg1->int_id_;
  }else{
    id_[1] = leg0->int_id_;
    id_[0] = leg1->int_id_;
  }

  // Add this people tracker to the legs

//  ROS_DEBUG_COND(DEBUG_PEOPLE_TRACKER,"PeopleTracker::%s Adding Tracker to the legs %s and %s",__func__,leg0->id_.c_str(), leg1->id_.c_str());
//  leg0->addPeopleTracker( boost::shared_ptr<PeopleTracker>(this) );
//  leg1->addPeopleTracker( boost::shared_ptr<PeopleTracker>(this) );

  ROS_DEBUG_COND(DEBUG_PEOPLE_TRACKER,"PeopleTracker::%s <NEW_PEOPLETRACKER %i-%i>", __func__, id_[0], id_[1]);

}

PeopleTracker::~PeopleTracker(){
  ROS_DEBUG_COND(DEBUG_PEOPLE_TRACKER,"PeopleTracker::%s <DELETE_PEOPLETRACKER %i-%i>", __func__, id_[0], id_[1]);
}

LegFeaturePtr PeopleTracker::getLeg0() const{
  return this->legs_[0];
}

LegFeaturePtr PeopleTracker::getLeg1() const{
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

/**
 * Check if this People Tracker is valid. This is the case if both its associated leg trackers are valid.
 * @return
 */
bool PeopleTracker::isValid() const{
  return (getLeg0()->isValid() && getLeg1()->isValid());
}

void PeopleTracker::update(ros::Time time){
  // Update the system state
  updateTrackerState(time);

  // Update the probabilities
  updateProbabilities(time);
}

/**
 * Update the state of this tracker
 * @param time
 */
void PeopleTracker::updateTrackerState(ros::Time time){
  //ROS_DEBUG_COND(DEBUG_PEOPLE_TRACKER,"PeopleTracker::%s", __func__);

  // Calculate the velocity vectors
  BFL::StatePosVel estLeg0 = getLeg0()->getEstimate();
  BFL::StatePosVel estLeg1 = getLeg1()->getEstimate();

  pos_vel_estimation_ =  (estLeg0+estLeg1);
  pos_vel_estimation_.pos_ = 0.5 * pos_vel_estimation_.pos_; // TODO ugly find a better way for this
  pos_vel_estimation_.vel_ = 0.5 * pos_vel_estimation_.vel_; // TODO ugly find a better way for this

  //std::cout << "leg0: " << getLeg0()->getEstimate() << " leg1: " << getLeg1()->getEstimate() << std::endl;
  //std::cout << "Estimation: " << pos_vel_estimation_ << std::endl << std::endl;

  // Calculate the hip width (only if some velocity exists)
  if(pos_vel_estimation_.vel_.length() > 0.01){

    // Set the hip vector
    hip_vec_[0] = -pos_vel_estimation_.vel_[1];
    hip_vec_[1] =  pos_vel_estimation_.vel_[0];
    hip_vec_[2] =  0.0;
    hip_vec_ = hip_vec_.normalize(); //Normalize

    std::cout << "pos_vel = [" << pos_vel_estimation_.vel_.getX() << ", " << pos_vel_estimation_.vel_.getY() << "," << pos_vel_estimation_.vel_.getZ() << "]" << std::endl;
    std::cout << "hip_vec = [" << hip_vec_.getX() << ", " << hip_vec_.getY() << ", " << hip_vec_.getZ() << "]" << std::endl;

    ROS_ASSERT(hip_vec_.dot(pos_vel_estimation_.vel_) < 0.000001);

    // Calculate the hip distance
    double d = distance(estLeg0.pos_, pos_vel_estimation_.pos_, pos_vel_estimation_.vel_);
    hipPos0_ = pos_vel_estimation_.pos_ + d * hip_vec_;
    hipPos1_ = pos_vel_estimation_.pos_ - d * hip_vec_;

    std::cout << "d= " << d << std::endl;

    // Calculate the step width
    //double step_length_ = (hipPos0_ - getLeg0()->getEstimate().pos_).length();
    //std::cout << "Step Length: " << step_length_ << std::endl;


    // Test if the three points are on a triangle
    tf::Vector3 a = pos_vel_estimation_.pos_ - hipPos0_;
    tf::Vector3 b = estLeg0.pos_ - hipPos0_;

    // Check if change necessary
    if(a.dot(b) > 0.0001){
      tf::Vector3 temp;
      temp = hipPos0_;
      hipPos0_ = hipPos1_;
      hipPos1_ = temp;
    }

    a = pos_vel_estimation_.pos_ - hipPos0_;
    b = estLeg0.pos_ - hipPos0_;

    ROS_ASSERT(a.dot(b) < 0.0001);

    std::cout << "ppl_pos = [" << pos_vel_estimation_.pos_.getX() << ", " << pos_vel_estimation_.pos_.getY() << "," << pos_vel_estimation_.pos_.getZ() << "]" << std::endl;

    std::cout << "hip0_pos = [" << hipPos0_.getX() << ", " << hipPos0_.getY() << "," << hipPos0_.getZ() << "]" << std::endl;
    std::cout << "hip1_pos = [" << hipPos1_.getX() << ", " << hipPos1_.getY() << "," << hipPos1_.getZ() << "]" << std::endl;

    std::cout << "leg0_pos = [" << estLeg0.pos_.getX() << ", " << estLeg0.pos_.getY() << "," << estLeg0.pos_.getZ() << "]" << std::endl;
    std::cout << "leg1_pos = [" << estLeg1.pos_.getX() << ", " << estLeg1.pos_.getY() << "," << estLeg1.pos_.getZ() << "]" << std::endl;


    std::cout << a.length() << std::endl;
    std::cout << b.length() << std::endl;

    std::cout << "CrossProduct" << a.dot(b) << std::endl;

  }else{
    hipPos0_ = pos_vel_estimation_.pos_;
    hipPos1_ = pos_vel_estimation_.pos_;
  }

  // Update static/dynamic
  is_static_ = !(getLeg0()->isDynamic() && getLeg1()->isDynamic());

}

void PeopleTracker::propagate(ros::Time time){
  ROS_WARN("PeopleTracker::%s is NOT YET IMPLEMENTED",__func__);
  //ROS_BREAK(); // TODO Implement this!
}

void PeopleTracker::updateProbabilities(ros::Time time){
  // Calculate the leg_distance probability
  double dist = LegFeature::distance(getLeg0(), getLeg1());
  ROS_ASSERT(dist > 0.0);

  double leg_distance_threshold = 0.7;
  dist_probability_ = 1.0-sigmoid(dist,5,leg_distance_threshold);
  ROS_ASSERT(dist_probability_ >= 0.0 && dist_probability_ <= 1.0);

  //ROS_DEBUG_COND(DEBUG_PEOPLE_TRACKER,"PeopleTracker::%s - Distance %f.3 Probability: %f.2",__func__, dist, dist_probability_);

  // Calculate the existenz of both LegTrackers
  double leg_time_threshold = 0.1;
  double min_leg_time = min(getLeg0()->getLifetime(), getLeg1()->getLifetime());

  leg_time_probability_ = sigmoid(min_leg_time,5,leg_time_threshold);

  // Calculate the association to the legs
  std::vector<PeopleTrackerPtr> assoLeg0 = getLeg0()->getPeopleTracker();
  std::vector<PeopleTrackerPtr> assoLeg1 = getLeg1()->getPeopleTracker();

  //std::cout << "Investigating the legs of " << *this << std::endl;


  //std::cout << "\t The Leg" << *getLeg0() << " is associated to: " << std::endl;
  double maxProb0 = 0.0;
  for(std::vector<PeopleTrackerPtr>::iterator peopleTrackerIt0 = assoLeg0.begin();
      peopleTrackerIt0 != assoLeg0.end();
      peopleTrackerIt0++)
  {
    //std::cout << "\t" << **peopleTrackerIt0 << "P: " << (*peopleTrackerIt0)->getTotalProbability();
    if((*peopleTrackerIt0)->id_ == id_){

      //std::cout << " this is me";
    }else{
      if((*peopleTrackerIt0)->isValid() && maxProb0 < (*peopleTrackerIt0)->getTotalProbability()){
        maxProb0 =  (*peopleTrackerIt0)->getTotalProbability();
      }
    }
    //std::cout << std::endl;
  }

  double associationProbabilityLeg0 = 1-maxProb0;
  ROS_ASSERT(associationProbabilityLeg0 <= 1.0);


  //std::cout << "\t The Leg" << *getLeg1() << " is associated to: " << std::endl;
  double maxProb1 = 0.0;
  for(std::vector<PeopleTrackerPtr>::iterator peopleTrackerIt0 = assoLeg1.begin();
      peopleTrackerIt0 != assoLeg1.end();
      peopleTrackerIt0++)
  {
    //std::cout << "\t" << **peopleTrackerIt0 << "P: " << (*peopleTrackerIt0)->getTotalProbability();
    if((*peopleTrackerIt0)->id_ == id_){

      //std::cout << " this is me";
    }else{
      if((*peopleTrackerIt0)->isValid() && maxProb1 < (*peopleTrackerIt0)->getTotalProbability()){
        maxProb1 =  (*peopleTrackerIt0)->getTotalProbability();
      }
    }
    //std::cout << std::endl;
  }

  double associationProbabilityLeg1 = 1-maxProb1;
  ROS_ASSERT(associationProbabilityLeg1 <= 1.0);

  leg_association_probability_ = min(associationProbabilityLeg0,associationProbabilityLeg1);

  //assert(id_[0] != 34 && id_[1] != 36);

  //std::cout << "Min LegTime: " << min_leg_time << " Probability: " << leg_time_probability_ << std::endl;

  // Update the probability based on the multiple assignments of its legs



  // std::cout << "Average: " << sum/move_sum.size() << std::endl;
  // Calculate the total probability
  total_probability_ = dist_probability_ * leg_time_probability_ * leg_association_probability_;


  /////////////////////////////////////////////////////////
  //// Calculate the two legged motion coefficient
  /////////////////////////////////////////////////////////

  std::vector<boost::shared_ptr<tf::Stamped<tf::Point> > > hist0 = getLeg0()->getHistory();
  std::vector<boost::shared_ptr<tf::Stamped<tf::Point> > > hist1 = getLeg1()->getHistory();

  int min_history = min(hist0.size(),hist1.size());

  if(total_probability_ > 0.5){
    std::cout << *this << std::endl;

    std::vector<double> move_sum;

    double sum = 0.0;
    for(int i = 1; i < min_history; i++){
      int idx = min_history-i;
      double temp = (*hist0[idx]-*hist0[idx-1]).length() * (*hist1[idx]-*hist1[idx-1]).length();
      sum += temp;
      move_sum.push_back(temp);

      std::cout << (*hist0[idx]).stamp_ << "," << (*hist0[idx]-*hist0[idx-1]).length() << "," << (*hist1[idx]-*hist1[idx-1]).length() << std::endl;
    }
  }


  // Print
#ifdef DEBUG_PEOPLE_TRACKER

  std::string color = RESET;
  if(total_probability_ > 0.6){
    color = BOLDMAGENTA;
    ROS_DEBUG_COND(DEBUG_PEOPLE_TRACKER,"%s#%i-%i|dist %.3f prob: %.2f| leg_time: %.2f prob: %.2f|leg_asso prob: %.2f|| total_p: %.2f|",color.c_str(), id_[0], id_[1], dist, dist_probability_,min_leg_time, leg_time_probability_,leg_association_probability_, total_probability_);
  }else if(isDynamic()){
    color = BOLDYELLOW;
    ROS_DEBUG_COND(DEBUG_PEOPLE_TRACKER,"%s#%i-%i|dist %.3f prob: %.2f| leg_time: %.2f prob: %.2f|leg_asso prob: %.2f|| total_p: %.2f|",color.c_str(), id_[0], id_[1], dist, dist_probability_,min_leg_time, leg_time_probability_,leg_association_probability_, total_probability_);
  }

#endif

}

BFL::StatePosVel PeopleTracker::getLegEstimate(int id){
  ROS_ASSERT_MSG(id == id_[0] || id == id_[1],"The estimate for a leg which is not part of this tracker was requested.");

  // Return the estimate of the leg // TODO the is only a temporarly solution
  if(id == id_[0])
    return getLeg0()->getEstimate();
  else
    return getLeg1()->getEstimate();

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

  ROS_DEBUG_COND(DEBUG_PEOPLETRACKERLIST,"PeopleTrackerList::%s",__func__);

  // Delete the legs
  for(std::vector<PeopleTrackerPtr>::iterator peopleTrackerIt = list_->begin(); peopleTrackerIt != list_->end(); peopleTrackerIt++){
    (*peopleTrackerIt)->removeLegs();
  }

  int size_before = list_->size();
  list_->erase(std::remove_if(list_->begin(), list_->end(), isValidPeopleTracker),list_->end());
  return size_before - list_->size();

  ROS_DEBUG_COND(DEBUG_PEOPLETRACKERLIST,"PeopleTrackerList::%s done",__func__);

}

void PeopleTrackerList::printTrackerList(){
  std::cout << "TrackerList:" << std::endl;
  for(std::vector<PeopleTrackerPtr>::iterator peopleTrackerIt = list_->begin(); peopleTrackerIt != list_->end(); peopleTrackerIt++){
    std::cout << **peopleTrackerIt << std::endl;
  }
}

void PeopleTrackerList::updateProbabilities(ros::Time time){
  for(std::vector<PeopleTrackerPtr>::iterator peopleTrackerIt = list_->begin(); peopleTrackerIt != list_->end(); peopleTrackerIt++){
    (*peopleTrackerIt)->updateProbabilities(time);
  }
}

void PeopleTrackerList::updateAllTrackers(ros::Time time){
  for(std::vector<PeopleTrackerPtr>::iterator peopleTrackerIt = list_->begin(); peopleTrackerIt != list_->end(); peopleTrackerIt++){
    (*peopleTrackerIt)->update(time);
  }
}

