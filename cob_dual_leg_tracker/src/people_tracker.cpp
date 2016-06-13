/*
 * people_tracker.cpp
 */

#undef NDEBUG

// ROS includes
#include <ros/console.h>

// System includes
#include <math.h>

// Own includes
#include <cob_dual_leg_tracker/people_tracker.h>
#include <cob_dual_leg_tracker/math/math_functions.h>
#include <cob_dual_leg_tracker/visualization/color_definitions.h>
#include <cob_dual_leg_tracker/social_interaction/formulas.h>

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
  stepWidthMax_(0.20),    // TODO make this a parameter
  stepWidth_(0.2),        // Initialize with this default stepWidth
  hipWidth_(-1.0),        // Initialize with default hipWidth -1
  is_static_(true),
  dist_probability_(0.0),
  leg_time_probability_(0.0),
  leg_association_probability_(0.0),
  hasGoal_(false)
{
  // Add the legs to this people tracker
  this->addLeg(leg0);
  this->addLeg(leg1);

  // Set the id of the tracker
  if(leg0->getId() < leg1->getId()){
    id_[0] = leg0->getId();
    id_[1] = leg1->getId();
  }else{
    id_[1] = leg0->getId();
    id_[0] = leg1->getId();
  }

  // Set the
  SymmetricMatrix measurementCov(3);
  for (unsigned int i = 0; i < 3; i++)
    for (unsigned int j = 0; j < 3; j++)
      measurementCov(i + 1, j + 1) = 0.1;

  StatePosVel prior_sigma(tf::Vector3(sqrt(measurementCov(1, 1)), sqrt(measurementCov(
                                        2, 2)), sqrt(measurementCov(3, 3))), tf::Vector3(0.0000001, 0.0000001, 0.0000001));
  //kalmanTracker = new estimation::TrackerKalman("people_tracker", sys_sigma_);
  //Tracker* new_tracker = new TrackerParticle(tracker_name.str(), num_particles_tracker, sys_sigma_);
  //kalmanTracker->initialize(measurementCov, prior_sigma, time.toSec());

  Eigen::Matrix<double,4,1> initialMeasurement;
  initialMeasurement(0,0) = this->getEstimate().pos_[0];
  initialMeasurement(1,0) = this->getEstimate().pos_[1];
  initialMeasurement(2,0) = this->getEstimate().vel_[0];
  initialMeasurement(3,0) = this->getEstimate().vel_[1];
  kalmanFilter_ = new filter::KalmanFilter(initialMeasurement, time);

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
  // Default return Leg0
  return getLeg0();
}

LegFeaturePtr PeopleTracker::getRightLeg() const{
  if(this->rightLeg_)
    return this->rightLeg_;
  //Default return Leg1
  return getLeg1();
}

LegFeaturePtr PeopleTracker::getMovingLeg() const{
  ROS_ASSERT(isDynamic());

	if(getLeg0()->getLastPositionJumpWidth() >= getLeg1()->getLastPositionJumpWidth()){
		return getLeg0();
	}
	return getLeg1();
}

LegFeaturePtr PeopleTracker::getStandingLeg() const{
	//return getLeg1();
	if(getLeg1()->getLastPositionJumpWidth() < getLeg0()->getLastPositionJumpWidth()){
		return getLeg1();
	}
	return getLeg0();
}

bool PeopleTracker::addLeg(LegFeaturePtr leg){

  // Return false if this tracker already has two legs
  if(legs_.size() >= 2) return false;

  if(legs_.size() == 1){
	  ROS_ASSERT(legs_[0]->getId() != leg->getId());
  }

  legs_.push_back(leg);
  return true;
}

bool PeopleTracker::isTheSame(LegFeaturePtr legA, LegFeaturePtr legB){

  if(this->getLeg0()->getId() == legA->getId() && this->getLeg1()->getId() == legB->getId()){
    return true;
  }

  if(this->getLeg1()->getId() == legA->getId() && this->getLeg0()->getId() == legB->getId()){
    return true;
  }
  return false;
}

bool PeopleTracker::isTheSame (PeopleTrackerPtr peopleTracker){

  if(this->getLeg0()->getId() == peopleTracker->getLeg0()->getId() && this->getLeg1()->getId() == peopleTracker->getLeg1()->getId()){
    return true;
  }

  if(this->getLeg1()->getId() == peopleTracker->getLeg0()->getId() && this->getLeg0()->getId() == peopleTracker->getLeg1()->getId()){
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

  // Update the KF
  kalmanFilter_->predict(time);

  Eigen::Matrix<double,4,1> currentPos;
  currentPos(0,0) = this->getEstimate().pos_[0];
  currentPos(1,0) = this->getEstimate().pos_[1];
  currentPos(2,0) = this->getEstimate().vel_[0];
  currentPos(3,0) = this->getEstimate().vel_[1];

  kalmanFilter_->update(currentPos);

  if(this->getTotalProbability() > 0.8)
    broadCastTf(time); // TODO choose better position for this

  // Update the history
  // updateHistory(time);
}

void PeopleTracker::configure(config_struct filter_config){
  ROS_DEBUG_COND(DEBUG_PEOPLE_TRACKER,"PeopleTracker::%s", __func__);

  // TODO Implement
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

    // Set the hip vector, rectangluar to the velocity
    hip_vec_[0] = -pos_vel_estimation_.vel_[1];
    hip_vec_[1] =  pos_vel_estimation_.vel_[0];
    hip_vec_[2] =  0.0;
    hip_vec_ = hip_vec_.normalize(); //Normalize

    // Check if the vectors are orthogonal
    double orthogonalCheckDotProductValue = hip_vec_.dot(pos_vel_estimation_.vel_);
    if(orthogonalCheckDotProductValue > 0.000001){
      std::cout << "pos_vel = [" << pos_vel_estimation_.vel_.getX() << ", " << pos_vel_estimation_.vel_.getY() << "," << pos_vel_estimation_.vel_.getZ() << "]" << std::endl;
      std::cout << "hip_vec = [" << hip_vec_.getX() << ", " << hip_vec_.getY() << ", " << hip_vec_.getZ() << "]" << std::endl;
      std::cout << RED << " The hip vector and the velocity vector are no orthogonal!" << RESET << std::endl;
      ROS_ASSERT(false);
    }


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

    // Calculate the current moving state
    tf::Vector3 foo1 = (leftLeg_->getEstimate().pos_ - hipPosLeft_);
    tf::Vector3 foo2 = pos_vel_estimation_.vel_;

    double state = foo1.getX() / foo2.getX();
    if(state > 0){
      frontLeg_ = leftLeg_;
      backLeg_ = rightLeg_;
    }else{
      frontLeg_ = rightLeg_;
      backLeg_ = leftLeg_;
    }

//    std::cout << foo1.getX() / foo2.getX() << std::endl;
//    std::cout << foo1.getY() / foo2.getY() << std::endl;
//
//    std::cout << "foo1 " << foo1.getX() << " " << foo1.getY() << " " << foo1.getZ() << std::endl;
//    std::cout << "foo2 " << foo2.getX() << " " << foo2.getY() << " " << foo2.getZ() << std::endl;

    // DEBUG
    tf::Vector3 a_vec = pos_vel_estimation_.pos_ - hipPosLeft_;
    tf::Vector3 b_vec = leftLeg_->getEstimate().pos_ - hipPosLeft_;
    if(a_vec.dot(b_vec) < 0.0001){
      //std::cout << "a_vec = [" << a_vec[0] << " " << a_vec[1] << " " << a_vec[2] << std::endl;
      //std::cout << "b_vec " << b_vec[0] << " " << b_vec[1] << " " << b_vec[2] << std::endl;

      //ROS_ASSERT(false);
    }
    //ROS_ASSERT(a_vec.dot(b_vec) < 0.0001);

    tf::Vector3 c_vec = pos_vel_estimation_.pos_ - hipPosRight_;
    tf::Vector3 d_vec = rightLeg_->getEstimate().pos_ - hipPosRight_;
    //ROS_ASSERT(c_vec.dot(d_vec) < 0.0001);
    if(c_vec.dot(d_vec) < 0.0001){
      //std::cout << "c_vec " << c_vec << std::endl;
      //std::cout << "d_vec " << d_vec << std::endl;

      //ROS_ASSERT(false);
    }



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
    stepWidth_ = min(0.5,(hipPosLeft_ - leftLeg_->getEstimate().pos_).length());

    //std::cout << "Setting stepWidth_: " << stepWidth_ << std::endl;
    //std::cout << "Setting stepWidthMax_: " << stepWidthMax_ << std::endl;
    ROS_ASSERT(stepWidth_ < 4); // Debug

    if(stepWidth_ > stepWidthMax_){
      stepWidthMax_ = stepWidth_;
      ROS_ASSERT(stepWidthMax_ < 4);
    }




  }

  else{ // If there is no speed there is no left or right

    hipPos0_     = pos_vel_estimation_.pos_;
    hipPos1_     = pos_vel_estimation_.pos_;
    hipPosLeft_  = pos_vel_estimation_.pos_;
    hipPosRight_ = pos_vel_estimation_.pos_;

    stepWidth_ = 0.2; // No Step
    stepWidthMax_ = 0.5;
    hipWidth_ = (getLeg0()->getEstimate().pos_ - getLeg1()->getEstimate().pos_).length();
  }

  ROS_ASSERT(stepWidthMax_ < 4); // TODO critical remove!

  // Update static/dynamic
  if(is_static_)
  {
    // static only if the person never moved; once set to dynamic it does not go back.
    is_static_ = !(getLeg0()->isDynamic() && getLeg1()->isDynamic());
  }

}

void PeopleTracker::propagate(ros::Time time){
  ROS_DEBUG_COND(DEBUG_PEOPLE_TRACKER,"PeopleTracker::%s", __func__);

  // Get the time delta (time since last propagation)
  double deltaT = time.toSec() - this->propagation_time_.toSec();


  if(this->getTotalProbability() > 0.6){ // TODO make this variable

    //std::cout << *this << " is now propagated" << std::endl;

    // Get the history of both assigned leg trackers
    std::vector<boost::shared_ptr<tf::Stamped<tf::Point> > > leftLegHistory = this->getLeftLeg()->getHistory();
    std::vector<boost::shared_ptr<tf::Stamped<tf::Point> > > rightLegHistory = this->getRightLeg()->getHistory();

    // Find the minimal history
    unsigned int shortestHistSize = min(leftLegHistory.size(), rightLegHistory.size());

    if(shortestHistSize > 1 && this->isDynamic()){

      //std::cout << "Left: L" << getLeftLeg()->getId() << " Right: L" << getRightLeg()->getId() << std::endl;

      double product = 0;
      // Reverse iterate the history (left)
      for(unsigned int i = shortestHistSize-1; i>0; i--){
        double jumpLeft = (*leftLegHistory[i]-*leftLegHistory[i-1]).length();
        double jumpRight = (*rightLegHistory[i]-*rightLegHistory[i-1]).length();

        if(jumpLeft < 0.01)
        	jumpLeft = 0;

        if(jumpRight < 0.01)
        	jumpRight = 0;

        product = jumpLeft * jumpRight;

        if(jumpLeft > jumpRight){
          //std::cout << RED << jumpLeft << RESET << "   " << jumpRight << std::setw(6) << std::setprecision(6) << " prod=" << product <<  std::endl;
        }else{
          //std::cout << jumpLeft << "   " << RED << jumpRight << RESET << std::setw(6) << std::setprecision(6) << " prod=" << product <<  std::endl;
        }

      }

      // Define the moving leg by the one with the last most movement

      if(deltaT > 0){

        // Differentiate between the moving and the static leg
        double lastStepWidthLeftLeg = getLeftLeg()->getLastPositionJumpWidth();
        double lastStepWidthRightLeg = getRightLeg()->getLastPositionJumpWidth();

        // If both legs had a step
        if(lastStepWidthLeftLeg > 0 && lastStepWidthRightLeg > 0){

          // Set the fixed leg
          LegFeaturePtr movLeg;
          LegFeaturePtr statLeg;

          if(lastStepWidthLeftLeg > lastStepWidthRightLeg){
            movLeg = getLeftLeg();
            statLeg = getRightLeg();

            //std::cout << "The left leg is moving" << std::endl;
          }else{
            movLeg = getRightLeg();
            statLeg = getLeftLeg();

            //std::cout << "The right leg is moving" << std::endl;
          }

          double alpha = cos(min(this->getStepWidth()/this->getStepWidthMax(),1.0) * M_PI)/2.0 + 0.5;

          //std::cout << "ALPHA" << alpha << std::endl;

          assert(alpha >= 0.0);
          assert(alpha <= 1.0);

          // StdCOUT propagation information
          //std::cout << "PROPAGATION____________________" << std::endl;
          //std::cout << "ALPHA" << alpha << std::endl;
          //std::cout << "LEG MOVING: " << movLeg->getId() << std::endl;
          //std::cout << "LEG STATIC: " << statLeg->getId() << std::endl;

          BFL::StatePosVel LegMovPrediction;
          BFL::StatePosVel LegStatPrediction;

          BFL::StatePosVel people_pos_vel_estimation_ = this->getEstimate();

          // Estimate the velocity
          LegMovPrediction.vel_   = 5 * people_pos_vel_estimation_.vel_ * alpha;   // Estimate Speed of the moving Leg // TODO Where comes this parameter from?
          LegStatPrediction.vel_  = 5 * people_pos_vel_estimation_.vel_ * (1-alpha);  // Estimated Speed of the constant Leg

          // First calculate the positions
          LegStatPrediction.pos_ =  LegStatPrediction.vel_*deltaT + statLeg->getEstimate().pos_;
          LegMovPrediction.pos_ =  LegMovPrediction.vel_*deltaT + movLeg->getEstimate().pos_;

          // Set the Estimates
          if(movLeg->getId() == getLeg0()->getId() && statLeg->getId() == getLeg1()->getId()){
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


    std::cout << "The history (size " << this->getLeftLeg()->getHistorySize() << ") of the left leg(L"<< this->getLeftLeg()->getId() << ") is" << std::endl;
    for(std::vector<boost::shared_ptr<tf::Stamped<tf::Point> > >::iterator histLeftIt = leftLegHistory.begin();
        histLeftIt != leftLegHistory.end();
        histLeftIt++){
      std::cout << (*histLeftIt)->getX() << " " << (*histLeftIt)->getY() << std::endl;
    }

    std::cout << "The history (size " << this->getRightLeg()->getHistorySize() << ") of the right leg(L" << this->getRightLeg()->getId()<< ") is" << std::endl;
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
  double leg_time_threshold = 0.08;
  double min_leg_time = min(getLeg0()->getLifetime(), getLeg1()->getLifetime());

  leg_time_probability_ = sigmoid(min_leg_time,2,leg_time_threshold);

  // Calculate the association to the legs
  std::vector<PeopleTrackerPtr> assoLeg0 = getLeg0()->getAssociatedPeopleTracker();
  std::vector<PeopleTrackerPtr> assoLeg1 = getLeg1()->getAssociatedPeopleTracker();

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

  if(total_probability_ > 0.5){ // TODO make this variable
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

  people_history_entry hist_entry;
  hist_entry.position_ = point;
  hist_entry.probability_ = this->getTotalProbability();

  history_.push_back(hist_entry);

  // Trim the history
  while(history_.size() > historySize)
    history_.pop_front();

}

/**
 * Get the current estimation of this tracker, the speed and position is calculated using the speed and position of both legs
 * @return
 */
BFL::StatePosVel PeopleTracker::getEstimate() const{
  ROS_DEBUG_COND(DEBUG_PEOPLE_TRACKER,"PeopleTracker[%i-%i]::%s", this->getLeg0()->getId(), this->getLeg1()->getId(), __func__);

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

BFL::StatePosVel PeopleTracker::getEstimateKalman(){
  BFL::StatePosVel kalmanEstimation;

  Eigen::Matrix<double,4,1> estimationMatrix;

  estimationMatrix = kalmanFilter_->getEstimation();
  kalmanEstimation.pos_[0] = estimationMatrix(0,0);
  kalmanEstimation.pos_[1] = estimationMatrix(1,0);
  kalmanEstimation.pos_[2] = 0.0;
  kalmanEstimation.vel_[0] = estimationMatrix(2,0);
  kalmanEstimation.vel_[1] = estimationMatrix(3,0);
  kalmanEstimation.vel_[2] = 0.0;

  return kalmanEstimation;

}

unsigned int PeopleTracker::getHistorySize(){
  return history_.size();
}

const std::list< people_history_entry >&  PeopleTracker::getHistory() const{
  return history_;
}

/***
 * Broadcast the position of humans in a tf
 * @param time
 */
void PeopleTracker::broadCastTf(ros::Time time){
  ROS_DEBUG_COND(DEBUG_PEOPLE_TRACKER,"PeopleTracker[%i-%i]::%s", this->getLeg0()->getId(), this->getLeg1()->getId(), __func__);

  tf::Transform transform;

  tf::Quaternion q;
  q.setRPY(0,0,0);

  transform.setOrigin( tf::Vector3(this->getEstimate().pos_[0], this->getEstimate().pos_[1], 0.0) );
  transform.setRotation(q);

  tf::StampedTransform test;
  test = tf::StampedTransform(transform, time, "odom_combined", this->getName());
  br.sendTransform(test);

}

/**
 * Calculate the next desired velocity
 * @param list List of all the trackers
 */
void PeopleTracker::calculateNextDesiredVelocity(boost::shared_ptr<std::vector<PeopleTrackerPtr> > list, size_t predictionStep, double timeInterval){
  ROS_DEBUG_COND(DEBUG_PEOPLE_TRACKER,"PeopleTracker[%i-%i]::%s", this->getLeg0()->getId(), this->getLeg1()->getId(), __func__);

  // The next desired velocity
  Eigen::Vector2d nextDesiredVelocity;

  // Optimization parameters
  double otherTrackerMinProbability = 0.6; //TODO make variable

  Eigen::Vector2d v_i; //Current velocity
  Eigen::Vector2d p_i; // Current position

  // Goal velocity
  double u_i;

  // Goal (Manually set)
  Eigen::Vector2d z_i;
  z_i[0] = -10.5;
  z_i[1] = 1.5;

  this->hasGoal_ = true;
  tf::Vector3 goal(z_i[0], z_i[1], 0);
  this->setGoal(goal); // Set the goal


  // if this is the first prediction step
  if(predictionStep == 0){
    // Clear old data
    this->nextDesiredPosition.clear();
    this->nextDesiredVelocity.clear();

    // Set the own position
    p_i(0) = this->getEstimate().pos_[0];
    p_i(1) = this->getEstimate().pos_[1];

    // Set the own velocity
    v_i(0) = this->getEstimate().vel_[0];
    v_i(1) = this->getEstimate().vel_[1];

    // Add the current estimation
    tf::Vector3 currentPos = this->getEstimate().pos_;
    this->nextDesiredPosition.push_back(currentPos);

    // Desired speed (currently - keep constant)
    u_i = v_i.norm();

    // Goal (lies on a straight line)
    double timeDelta = 2;
    //z_i = p_i + timeDelta * v_i;
  }
  else
  {

    tf::Vector3 currentVel = this->nextDesiredVelocity.back();
    tf::Vector3 currentPos = this->nextDesiredPosition.back() + this->nextDesiredVelocity.back() * timeInterval;

    this->nextDesiredPosition.push_back(currentPos);

    // Set the own position
    p_i(0) = currentPos[0];
    p_i(1) = currentPos[1];

    // Set the own velocity
    v_i(0) = currentVel[0];
    v_i(1) = currentVel[1];
  }

  // Abort if this person is standing
  if(v_i.norm() < 0.1){
    //std::cout << this->getName() << " is considered standing" << std::endl;
    this->nextDesiredVelocity.push_back(this->getEstimate().vel_);

    return;
  }

  //std::cout << this->getName() << " calculates the next desired velocity" << std::endl;

  std::vector<Eigen::Vector2d> positionOthers;
  std::vector<Eigen::Vector2d> velocityOthers;

  // Iterate all the trackers to get the positions and velocities
  //std::cout << "\t it takes into account: ";
  for(std::vector<PeopleTrackerPtr>::iterator peopleTrackerIt = list->begin(); peopleTrackerIt != list->end(); peopleTrackerIt++){

    if((*peopleTrackerIt)->getTotalProbability() > otherTrackerMinProbability){
      if(!(*this == **peopleTrackerIt)){
        Eigen::Vector2d pos, vel;

        //std::cout << (*peopleTrackerIt)->getName() << "  ";

        // Get the estimate
        pos[0] = (*peopleTrackerIt)->getNextDesiredPosVel(predictionStep).pos_[0];
        pos[1] = (*peopleTrackerIt)->getNextDesiredPosVel(predictionStep).pos_[1];

        vel[0] = (*peopleTrackerIt)->getNextDesiredPosVel(predictionStep).vel_[0];
        vel[1] = (*peopleTrackerIt)->getNextDesiredPosVel(predictionStep).vel_[1];

        // Store
        positionOthers.push_back(pos);
        velocityOthers.push_back(vel);
      }
    }
  }

  //std::cout << std::endl;


  Eigen::Vector2d grad;
  Eigen::Vector2d gradInteractionEnergy;

  // Parameters
  double sigma_d = 0.361; // Distance to avoid of other objects, the bigger the more influence
  double sigma_w = 2.088; // Radius of influence of other objects
  double lambda_0 = 1*1;    // Influence of other people
  double lambda_1 = 1*2.33; // Influence of desired velocity
  double lambda_2 = 1*2.073;// Influence of goal
  double beta = 1.1462;
  double alpha = 0.730;

  Eigen::Vector2d v_tilde_i = v_i;


  //std::cout << "\t Starting loop | Start velocity:" << v_tilde_i.transpose() << std::endl;

  ////////////////////////////////////////////////////////////////
  /// Gradient Descent (with Backtracking Line Search)
  ///////////////////////////////////////////////////////////////

  size_t iteration_counter = 0;
  size_t maxIterations = 100;
  double energyDeltaAbort = 0.0001;
  double velocityDeltaAbort = 0.001;

  Eigen::Vector2d v_tilde_i_last = v_tilde_i;

  double energyLast  = E_i(positionOthers, velocityOthers, v_tilde_i, p_i, v_i, u_i, z_i, sigma_d, sigma_w, beta, lambda_0, lambda_1, lambda_2);



  double a = 0.5;
  double beta_gradient_descent = 0.8;

  //std::cout << std::endl << WHITE << "Starting gradient descent for " << this->getName() << RESET << std::endl;

  // Check if really goes down this way
  double testStepSize = 0.00000001;
  grad = E_i_gradient(positionOthers, velocityOthers, v_tilde_i, p_i, v_i, u_i, z_i, sigma_d, sigma_w, beta, lambda_0, lambda_1, lambda_2);
  double energyTest  = E_i(positionOthers, velocityOthers, v_tilde_i - testStepSize * grad, p_i, v_i, u_i, z_i, sigma_d, sigma_w, beta, lambda_0, lambda_1, lambda_2);

  if(energyTest > energyLast && abs(energyTest-energyLast) > 0.000001){
    std::cout << "Number of considered persons " << positionOthers.size() << std::endl;
    std::cout << RED << "Test [FAILED] -> descent direction " << grad.transpose() << " at stepsize:" << std::scientific << testStepSize << " energyLast" << energyLast << " newEnergy:" << energyTest << " Difference:"  << energyTest-energyLast << RESET << std::endl;

    assert(false);
  }else{
    //std::cout << "Test [PASSED] -> descent direction " << grad.transpose() << " at stepsize:" << std::scientific << testStepSize << " energyLast" << energyLast << " newEnergy:" << energyTest << " Difference:" << energyTest-energyLast << std::endl;
  }


  while(iteration_counter < maxIterations){

    double eta = 0.1;

    // calculate the gradient
    grad = E_i_gradient(positionOthers, velocityOthers, v_tilde_i, p_i, v_i, u_i, z_i, sigma_d, sigma_w, beta, lambda_0, lambda_1, lambda_2);
    //std::cout << "gradient: " << grad << std::endl;

    // Get the energy at this point
    double currentEnergy  = E_i(positionOthers, velocityOthers, v_tilde_i - eta * grad, p_i, v_i, u_i, z_i, sigma_d, sigma_w, beta, lambda_0, lambda_1, lambda_2);

    //if(positionOthers.size() > 0)
    //  std::cout << "Starting backtracking line search! currentEnergy: " << currentEnergy << std::endl;


    while(energyLast - currentEnergy < a * eta * grad.squaredNorm() && eta > testStepSize){
      //std::cout << "currentEnergy " << currentEnergy << " <= " << energyLast - a * eta * grad.squaredNorm() << "(energyLast - a * eta * grad.squaredNorm() " << std::endl;
      //std::cout << "current Energy: " << currentEnergy << " energyLast: " << energyLast << std::endl;
      eta = beta_gradient_descent * eta;

      currentEnergy  = E_i(positionOthers, velocityOthers, v_tilde_i - eta * grad, p_i, v_i, u_i, z_i, sigma_d, sigma_w, beta, lambda_0, lambda_1, lambda_2);
     // std::cout << "eta: " << eta << std::endl;
    }
    if(positionOthers.size() > 0){
      //std::cout << "EnergyStep: [" << energyLast << "] --> [" <<  currentEnergy << "]  eta:" << eta << std::endl;

    }

    // Make a gradient descent
    v_tilde_i_last = v_tilde_i;
    v_tilde_i = v_tilde_i - eta * grad;

    // Save energy
    double energyDelta = abs(energyLast - currentEnergy);
    double velocityDelta = (v_tilde_i_last - v_tilde_i).norm();

    if(!(energyLast >= currentEnergy)){
      std::cout << "Number of considered persons " << positionOthers.size() << std::endl;
      std::cout << "NO ENERGY REDUCTION! eta:" << eta << std::endl;
      std::cout << RED;
    }

    //if(positionOthers.size() > 0) // Only output if other persons where considered
    //  std::cout << "[" << iteration_counter << "] energy: " << currentEnergy << " dEn:" << energyDelta << "  dVel: " << velocityDelta << " | curVel: " << v_tilde_i.transpose() << RESET << std::endl;

    // Check abort criteria
    if(energyDelta < energyDeltaAbort){
      if(positionOthers.size() > 0)
      //  std::cout << "Abort -> energyDelta:" << energyDelta << std::endl;
      //assert(false);
      break;
    }


    if(velocityDelta < velocityDelta){
      if(positionOthers.size() > 0)
        //std::cout << "Abort -> velocityDelta:" << velocityDelta << std::endl;
      break;
    }

    energyLast = currentEnergy;
    iteration_counter++;
  }

  // Check the difference
  //std::cout << "Change in velocity:" << (v_tilde_i - v_i).norm() << std::endl;

  // Filter
  Eigen::Vector2d v_tilde_filtered;
  v_tilde_filtered = (alpha * v_i + (1-alpha) * v_tilde_i);

  // Store the desired Velocity
  tf::Vector3 desiredVelocity(v_tilde_filtered(0),v_tilde_filtered(1),0);
  this->nextDesiredVelocity.push_back(desiredVelocity);

  //assert(positionOthers.size() == 0);

}

/**
 * Get the desired Pos/Vel at a given prediction Step
 * @param predictionStep
 * @param timeInterval
 * @return
 */
BFL::StatePosVel PeopleTracker::getNextDesiredPosVel(size_t predictionStep) const{
  ROS_DEBUG_COND(DEBUG_PEOPLETRACKERLIST,"PeopleTracker::%s",__func__);
  BFL::StatePosVel nextDesiredPosVel;

  //std::cout << "predictionStep: " << predictionStep << std::endl;

  if(predictionStep == 0){
    nextDesiredPosVel = this->getEstimate();
  }
  else{
    if(predictionStep > this->nextDesiredPosition.size() && predictionStep > this->nextDesiredVelocity.size()){
      std::cout << "Name: " << this->getName() << std::endl;
      std::cout << "predictionStep: " << predictionStep << std::endl;
      std::cout << "this->nextDesiredPosition.size(): " << this->nextDesiredPosition.size() << std::endl;
      std::cout << "this->nextDesiredVelocity.size(): " << this->nextDesiredVelocity.size() << std::endl;
      ROS_ASSERT(false);
    }

    nextDesiredPosVel.pos_ = this->nextDesiredPosition[predictionStep-1];
    nextDesiredPosVel.vel_ = this->nextDesiredVelocity[predictionStep-1];
  }

  return nextDesiredPosVel;
}


bool operator== (PeopleTracker &p0, PeopleTracker &p1)
{
  return (p0.getName() == p1.getName());
}

/////////////////////////////////////////////////////////////
//// PeopleTrackerList Class Definitions
/////////////////////////////////////////////////////////////

PeopleTrackerList::PeopleTrackerList():
  list_(new std::vector<PeopleTrackerPtr>()){}

/**
 * Check if a People Tracker allready exists for these two legs
 * @param legA The one leg
 * @param legB The other leg
 * @return True if it allready exists
 */
bool PeopleTrackerList::exists(LegFeaturePtr legA, LegFeaturePtr legB) const{
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
bool PeopleTrackerList::exists(PeopleTrackerPtr peopleTracker) const{
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
  ROS_DEBUG_COND(DEBUG_PEOPLETRACKERLIST,"PeopleTrackerList::%s",__func__);
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

void PeopleTrackerList::printTrackerList() const{
  ROS_DEBUG_COND(DEBUG_PEOPLETRACKERLIST,"PeopleTrackerList::%s",__func__);
  std::cout << "TrackerList:" << std::endl;
  for(std::vector<PeopleTrackerPtr>::iterator peopleTrackerIt = list_->begin(); peopleTrackerIt != list_->end(); peopleTrackerIt++){
    std::cout << **peopleTrackerIt << std::endl;
  }
}

void PeopleTrackerList::updateProbabilities(ros::Time time){
  ROS_DEBUG_COND(DEBUG_PEOPLETRACKERLIST,"PeopleTrackerList::%s",__func__);
  for(std::vector<PeopleTrackerPtr>::iterator peopleTrackerIt = list_->begin(); peopleTrackerIt != list_->end(); peopleTrackerIt++){
    (*peopleTrackerIt)->updateProbabilities(time);
  }
}

void PeopleTrackerList::updateAllTrackers(ros::Time time){
  ROS_DEBUG_COND(DEBUG_PEOPLETRACKERLIST,"PeopleTrackerList::%s",__func__);
  for(std::vector<PeopleTrackerPtr>::iterator peopleTrackerIt = list_->begin(); peopleTrackerIt != list_->end(); peopleTrackerIt++){

    (*peopleTrackerIt)->update(time);
    (*peopleTrackerIt)->updateHistory(time);

  }
}

void PeopleTrackerList::calculateTheNextDesiredVelocities(double timeInterval, size_t predictionSteps){
  ROS_DEBUG_COND(DEBUG_PEOPLETRACKERLIST,"PeopleTrackerList::%s",__func__);
  // Iterate the prediction steps
  for(size_t predictionStep = 0; predictionStep < predictionSteps; predictionStep++){

    // Do this step for every tracker
    for(std::vector<PeopleTrackerPtr>::iterator peopleTrackerIt = list_->begin(); peopleTrackerIt != list_->end(); peopleTrackerIt++){

      // Only consider persons with a certain probability
      if((*peopleTrackerIt)->getTotalProbability() > 0.6){ // TODO make variable
        (*peopleTrackerIt)->calculateNextDesiredVelocity(this->getList(), predictionStep, timeInterval);
      }

    }

  }
}





