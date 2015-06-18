/*
 * people_tracker.cpp
 *
 *  Created on: Apr 16, 2015
 *      Author: frm-ag
 */

#include <ros/console.h>
// Own includes
#include <dual_people_leg_tracker/people_tracker.h>
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
  total_probability_(0.0), // Initialize the probability with zero
  propagation_time_(time),
  maxStepWidth_(0.20), // TODO make this a parameter
  stepWidth_(-1.0),
  hipWidth_(-1.0)
{
  // Add the legs to this people tracker
  this->addLeg(leg0);
  this->addLeg(leg1);

  // Set the id of the tracker
  if(leg0->int_id_ < leg1->int_id_){
    id_[0] = leg0->int_id_;
    id_[1] = leg1->int_id_;
  }else{
    id_[1] = leg0->int_id_;
    id_[0] = leg1->int_id_;
  }

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

LegFeaturePtr PeopleTracker::getLeftLeg() const{
  if(this->leftLeg_)
    return this->leftLeg_;
  return getLeg0();
}

LegFeaturePtr PeopleTracker::getRightLeg() const{
  if(this->rightLeg_)
    return this->rightLeg_;
  return getLeg1();
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

  // Update the history
  // updateHistory(time);
}

/**
 * Update the state of this tracker
 * @param time
 */
void PeopleTracker::updateTrackerState(ros::Time time){
  //ROS_DEBUG_COND(DEBUG_PEOPLE_TRACKER,"PeopleTracker::%s", __func__);

  // Calculate the velocity vectors
  BFL::StatePosVel estLeg0 = getLeg0()->getEstimate();

  pos_vel_estimation_ = getEstimate();

  // Calculate the hip width (only if some velocity exists)
  if(pos_vel_estimation_.vel_.length() > 0.01){

    // Set the hip vector
    hip_vec_[0] = -pos_vel_estimation_.vel_[1];
    hip_vec_[1] =  pos_vel_estimation_.vel_[0];
    hip_vec_[2] =  0.0;
    hip_vec_ = hip_vec_.normalize(); //Normalize

    //std::cout << "pos_vel = [" << pos_vel_estimation_.vel_.getX() << ", " << pos_vel_estimation_.vel_.getY() << "," << pos_vel_estimation_.vel_.getZ() << "]" << std::endl;
    //std::cout << "hip_vec = [" << hip_vec_.getX() << ", " << hip_vec_.getY() << ", " << hip_vec_.getZ() << "]" << std::endl;

    ROS_ASSERT(hip_vec_.dot(pos_vel_estimation_.vel_) < 0.000001);

    // Calculate the hip distance
    double d = distance(estLeg0.pos_, pos_vel_estimation_.pos_, pos_vel_estimation_.vel_);

    hipPosLeft_   = pos_vel_estimation_.pos_ + d * hip_vec_; // The left hip
    hipPosRight_  = pos_vel_estimation_.pos_ - d * hip_vec_; // The right hip

    //std::cout << "d= " << d << std::endl;

    // Calculate the step width
    //double step_length_ = (hipPos0_ - getLeg0()->getEstimate().pos_).length();
    //std::cout << "Step Length: " << step_length_ << std::endl;


    // Test if the three points are on a triangle
    tf::Vector3 posHipLeft = pos_vel_estimation_.pos_ - hipPosLeft_;
    tf::Vector3 leg0hipPosLeft = estLeg0.pos_ - hipPosLeft_;

    // Is there a right angle between leg0 and the left hip
    if(posHipLeft.dot(leg0hipPosLeft) < 0.0001){
      leftLeg_  = getLeg0();
      rightLeg_ = getLeg1();
    }else{
      leftLeg_  = getLeg1();
      rightLeg_ = getLeg0();
    }


    // DEBUG
    tf::Vector3 a_vec = pos_vel_estimation_.pos_ - hipPosLeft_;
    tf::Vector3 b_vec = leftLeg_->getEstimate().pos_ - hipPosLeft_;
    ROS_ASSERT(a_vec.dot(b_vec) < 0.0001);

    tf::Vector3 c_vec = pos_vel_estimation_.pos_ - hipPosRight_;
    tf::Vector3 d_vec = rightLeg_->getEstimate().pos_ - hipPosRight_;
    ROS_ASSERT(c_vec.dot(d_vec) < 0.0001);



    //std::cout << "ppl_pos = [" << pos_vel_estimation_.pos_.getX() << ", " << pos_vel_estimation_.pos_.getY() << "," << pos_vel_estimation_.pos_.getZ() << "]" << std::endl;

    //std::cout << "hipLeft_pos = ["  << hipPosLeft_.getX() << ", " << hipPosLeft_.getY() << "," << hipPosLeft_.getZ() << "]" << std::endl;
    //std::cout << "hipRight_pos = [" << hipPosRight_.getX() << ", " << hipPosRight_.getY() << "," << hipPosRight_.getZ() << "]" << std::endl;

    //std::cout << "legLeft_pos = [" << leftLeg_->getEstimate().pos_.getX() << ", " << leftLeg_->getEstimate().pos_.getY() << "," << leftLeg_->getEstimate().pos_.getZ() << "]" << std::endl;
    //std::cout << "legRight_pos = [" << rightLeg_->getEstimate().pos_.getX() << ", " << rightLeg_->getEstimate().pos_.getY() << "," << rightLeg_->getEstimate().pos_.getZ() << "]" << std::endl;

    if(rightLeg_->getEstimate().vel_.length() > leftLeg_->getEstimate().vel_.length()){
      //std::cout << BOLDGREEN << "Right" << RESET << std::endl;
    }else{
      //std::cout << BOLDRED << "Left" << RESET << std::endl;
    }

    hipWidth_ = (hipPosLeft_ - hipPosRight_).length();
    stepWidth_ = (hipPosLeft_ - leftLeg_->getEstimate().pos_).length();

  }else{ // If there is no speed there is no left or right
    hipPos0_     = pos_vel_estimation_.pos_;
    hipPos1_     = pos_vel_estimation_.pos_;
    hipPosLeft_  = pos_vel_estimation_.pos_;
    hipPosRight_ = pos_vel_estimation_.pos_;

    stepWidth_ = 0.0; // No Step
    hipWidth_ = (getLeg0()->getEstimate().pos_ - getLeg1()->getEstimate().pos_).length();
  }


  // Update static/dynamic
  is_static_ = !(getLeg0()->isDynamic() && getLeg1()->isDynamic());

}

void PeopleTracker::propagate(ros::Time time){
  ROS_DEBUG_COND(DEBUG_PEOPLE_TRACKER,"PeopleTracker::%s", __func__);

  // Get the time delta (time since last propagation)
  double deltaT = time.toSec() - this->propagation_time_.toSec();


  if(this->getTotalProbability() > 0.8){

    std::cout << *this << " is now propagated" << std::endl;

    // Get the history of both assigned leg trackers
    std::vector<boost::shared_ptr<tf::Stamped<tf::Point> > > leftLegHistory = this->getLeftLeg()->getHistory();
    std::vector<boost::shared_ptr<tf::Stamped<tf::Point> > > rightLegHistory = this->getRightLeg()->getHistory();

    // Find the minimal history
    unsigned int shortestHistSize = min(leftLegHistory.size(), rightLegHistory.size());

    if(shortestHistSize > 1 && this->isDynamic()){

      std::cout << "Left: L" << getLeftLeg()->int_id_ << " Right: L" << getRightLeg()->int_id_ << std::endl;


      // Reverse iterate the history (left)
      for(unsigned int i = shortestHistSize-1; i>0; i--){
        double jumpLeft = (*leftLegHistory[i]-*leftLegHistory[i-1]).length();
        double jumpRight = (*rightLegHistory[i]-*rightLegHistory[i-1]).length();

        if(jumpLeft > jumpRight){
          std::cout << RED << jumpLeft << RESET << "   " << jumpRight << std::endl;
        }else{
          std::cout << jumpLeft << "   " << RED << jumpRight << RESET << std::endl;
        }
      }


      // Print for python debugging
      std::cout << "left_leg = [";
      for(unsigned int i = shortestHistSize-1; i>0; i--){
        double jumpLeft = (*leftLegHistory[i]-*leftLegHistory[i-1]).length();

        std::cout << jumpLeft;
        if(i!=1) std::cout << ",";
      }
      std::cout << "]" << std::endl;

      std::cout << "right_leg = [";
      for(unsigned int i = shortestHistSize-1; i>0; i--){
        double jumpRight = (*rightLegHistory[i]-*rightLegHistory[i-1]).length();
        std::cout << jumpRight;

        if(i!=1) std::cout << ",";
      }
      std::cout << "]" << std::endl;





      // Define the moving leg by the one with the last most movement

      if(deltaT > 0){

        // Differentiate between the moving and the static leg
        double lastStepWidthLeftLeg;
        double lastStepWidthRightLeg;
        getLeftLeg()->getLastStepWidth(lastStepWidthLeftLeg);
        getRightLeg()->getLastStepWidth(lastStepWidthRightLeg);

        //return;
        if(getLeftLeg()->getLastStepWidth(lastStepWidthLeftLeg) && getRightLeg()->getLastStepWidth(lastStepWidthRightLeg)){

          // Set the fixed leg
          LegFeaturePtr movLeg;
          LegFeaturePtr statLeg;

          if(lastStepWidthLeftLeg > lastStepWidthRightLeg){
            movLeg = getLeftLeg();
            statLeg = getRightLeg();

            std::cout << "The left leg is moving" << std::endl;
          }else{
            movLeg = getRightLeg();
            statLeg = getLeftLeg();

            std::cout << "The right leg is moving" << std::endl;
          }

          double alpha = cos(min(this->getStepWidth()/this->maxStepWidth_,1.0) * M_PI)/2.0 + 0.5;

          std::cout << "ALPHA" << alpha << std::endl;

          assert(alpha >= 0.0);
          assert(alpha <= 1.0);

          // StdCOUT propagation information
          std::cout << "PROPAGATION____________________" << std::endl;
          std::cout << "ALPHA" << alpha << std::endl;
          std::cout << "LEG MOVING: " << movLeg->int_id_ << std::endl;
          std::cout << "LEG STATIC: " << statLeg->int_id_ << std::endl;

          BFL::StatePosVel LegMovPrediction;
          BFL::StatePosVel LegStatPrediction;

          BFL::StatePosVel people_pos_vel_estimation_ = this->getEstimate();

          // Estimate the velocity
          LegMovPrediction.vel_   = 5 * people_pos_vel_estimation_.vel_ * alpha;   // Estimate Speed of the moving Leg
          LegStatPrediction.vel_  = 5 * people_pos_vel_estimation_.vel_ * (1-alpha);  // Estimated Speed of the constant Leg

          // First calculate the positions
          LegStatPrediction.pos_ =  LegStatPrediction.vel_*deltaT + statLeg->getEstimate().pos_;
          LegMovPrediction.pos_ =  LegMovPrediction.vel_*deltaT + movLeg->getEstimate().pos_;

          // Set the Estimates
          if(movLeg->int_id_ == getLeg0()->int_id_ && statLeg->int_id_ == getLeg1()->int_id_){
            leg0Prediction_ = LegMovPrediction;
            leg1Prediction_ = LegStatPrediction;
          }else{
            leg0Prediction_ = LegStatPrediction;
            leg1Prediction_ = LegMovPrediction;
          }

        }


      }

    }





/*
    // Reverse iterate the history
    for(unsigned int i = shortestHistSize-1; i>0; i--){

    }


    std::cout << "The history (size " << this->getLeftLeg()->getHistorySize() << ") of the left leg(L"<< this->getLeftLeg()->int_id_ << ") is" << std::endl;
    for(std::vector<boost::shared_ptr<tf::Stamped<tf::Point> > >::iterator histLeftIt = leftLegHistory.begin();
        histLeftIt != leftLegHistory.end();
        histLeftIt++){
      std::cout << (*histLeftIt)->getX() << " " << (*histLeftIt)->getY() << std::endl;
    }

    std::cout << "The history (size " << this->getRightLeg()->getHistorySize() << ") of the right leg(L" << this->getRightLeg()->int_id_<< ") is" << std::endl;
    for(std::vector<boost::shared_ptr<tf::Stamped<tf::Point> > >::iterator histRightIt = rightLegHistory.begin();
        histRightIt != rightLegHistory.end();
        histRightIt++){
      std::cout << (*histRightIt)->getX() << " " << (*histRightIt)->getY() << std::endl;
    }*/


  }

  // Set the propagation time to this time after the propagation is done
  this->propagation_time_ = time;

}

void PeopleTracker::updateProbabilities(ros::Time time){
  // Calculate the leg_distance probability
  double dist = LegFeature::distance(getLeg0(), getLeg1());
  ROS_ASSERT(dist > 0.0);

  double leg_distance_threshold = 0.8;
  dist_probability_ = 1.0-sigmoid(dist,20,leg_distance_threshold);
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
    //std::cout << *this << std::endl;

    std::vector<double> move_sum;

    double sum = 0.0;
    for(int i = 1; i < min_history; i++){
      int idx = min_history-i;
      double temp = (*hist0[idx]-*hist0[idx-1]).length() * (*hist1[idx]-*hist1[idx-1]).length();
      sum += temp;
      move_sum.push_back(temp);

      //std::cout << (*hist0[idx]).stamp_ << "," << (*hist0[idx]-*hist0[idx-1]).length() << "," << (*hist1[idx]-*hist1[idx-1]).length() << std::endl;
    }
  }


  // Print
#ifdef DEBUG_PEOPLE_TRACKER

  std::string color = RESET;
  if(total_probability_ > 0.6){
    color = BOLDMAGENTA;
    ROS_DEBUG_COND(DEBUG_PEOPLE_TRACKER,"%s#%i-%i|dist %.3f prob: %.2f| leg_time: %.2f prob: %.2f|leg_asso prob: %.2f|| total_p: %.2f|",color.c_str(), id_[0], id_[1], dist, dist_probability_,min_leg_time, leg_time_probability_,leg_association_probability_, total_probability_);
  }else if(isDynamic() && total_probability_ > 0.6){
    color = BOLDYELLOW;
    ROS_DEBUG_COND(DEBUG_PEOPLE_TRACKER,"%s#%i-%i|dist %.3f prob: %.2f| leg_time: %.2f prob: %.2f|leg_asso prob: %.2f|| total_p: %.2f|",color.c_str(), id_[0], id_[1], dist, dist_probability_,min_leg_time, leg_time_probability_,leg_association_probability_, total_probability_);
  }

#endif

}

void PeopleTracker::updateHistory(ros::Time time){

  BFL::StatePosVel est = getEstimate();

  boost::shared_ptr<tf::Stamped<tf::Point> > point(new tf::Stamped<tf::Point>());
  point->setX( est.pos_[0]);
  point->setY( est.pos_[1]);
  point->setZ( est.pos_[2]);
  point->stamp_ = time;

  position_history_.push_back(point);
}

/**
 * Get the current estimation of this tracker, the speed and position is calculated using the speed and position of both legs
 * @return
 */
BFL::StatePosVel PeopleTracker::getEstimate(){

  // Calculate the velocity vectors
  BFL::StatePosVel estLeg0 = getLeg0()->getEstimate();
  BFL::StatePosVel estLeg1 = getLeg1()->getEstimate();

  BFL::StatePosVel pos_vel_estimation;
  pos_vel_estimation =  (estLeg0+estLeg1);
  pos_vel_estimation.pos_ = 0.5 * pos_vel_estimation.pos_; // TODO ugly find a better way for this
  pos_vel_estimation.vel_ = 0.5 * pos_vel_estimation.vel_; // TODO ugly find a better way for this

  return pos_vel_estimation;
}

BFL::StatePosVel PeopleTracker::getLegEstimate(int id){
  ROS_ASSERT_MSG(id == id_[0] || id == id_[1],"The estimate for a leg which is not part of this tracker was requested.");

  // Check if there is a Estimation for this leg
  if(id == id_[0]){
    return leg0Prediction_;
  }

  if(id == id_[1]){
    return leg1Prediction_;
  }


}

unsigned int PeopleTracker::getHistorySize(){
  return position_history_.size();
}

std::vector<boost::shared_ptr<tf::Stamped<tf::Point> > >  PeopleTracker::getHistory(){
  return position_history_;
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

    (*peopleTrackerIt)->updateHistory(time);
  }
}

