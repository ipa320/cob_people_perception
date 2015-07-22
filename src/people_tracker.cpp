/*
 * people_tracker.cpp
 *
 *  Created on: Apr 16, 2015
 *      Author: frm-ag
 */
#undef NDEBUG
#include <ros/console.h>
// Own includes
#include <dual_people_leg_tracker/people_tracker.h>
#include <dual_people_leg_tracker/math/math_functions.h>
#include <leg_detector/color_definitions.h>

#include <math.h>

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
  stepWidth_(0),
  hipWidth_(-1.0),
  is_static_(true),
  dist_probability_(0.0),
  leg_time_probability_(0.0),
  leg_association_probability_(0.0)
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
  //delete kalmanTracker;

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

LegFeaturePtr PeopleTracker::getMovingLeg() const{
  ROS_ASSERT(isDynamic());

	if(getLeg0()->getLastStepWidth() >= getLeg1()->getLastStepWidth()){
		return getLeg0();
	}
	return getLeg1();
}

LegFeaturePtr PeopleTracker::getStandingLeg() const{
	//return getLeg1();
	if(getLeg1()->getLastStepWidth() < getLeg0()->getLastStepWidth()){
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

  // Update the KF
  kalmanFilter_->predict(time);

  Eigen::Matrix<double,4,1> currentPos;
  currentPos(0,0) = this->getEstimate().pos_[0];
  currentPos(1,0) = this->getEstimate().pos_[1];
  currentPos(2,0) = this->getEstimate().vel_[0];
  currentPos(3,0) = this->getEstimate().vel_[1];

  kalmanFilter_->update(currentPos);

  broadCastTf(time);
  //std::cout << "------------------" << std::endl;
  //std::cout << kalmanFilter_->getEstimation() << std::endl;


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

    // Set the hip vector, rectangluar to the velocity
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

    if(stepWidth_ > stepWidthMax_){
      stepWidthMax_ = stepWidth_;
    }




  }

  else{ // If there is no speed there is no left or right

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


  if(this->getTotalProbability() > 0.6){

    //std::cout << *this << " is now propagated" << std::endl;

    // Get the history of both assigned leg trackers
    std::vector<boost::shared_ptr<tf::Stamped<tf::Point> > > leftLegHistory = this->getLeftLeg()->getHistory();
    std::vector<boost::shared_ptr<tf::Stamped<tf::Point> > > rightLegHistory = this->getRightLeg()->getHistory();

    // Find the minimal history
    unsigned int shortestHistSize = min(leftLegHistory.size(), rightLegHistory.size());

    if(shortestHistSize > 1 && this->isDynamic()){

      //std::cout << "Left: L" << getLeftLeg()->int_id_ << " Right: L" << getRightLeg()->int_id_ << std::endl;

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



      // Print for python debugging
      //std::cout << "left_leg = [";
/*      for(unsigned int i = shortestHistSize-1; i>0; i--){
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
      std::cout << "]" << std::endl;*/





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

            //std::cout << "The left leg is moving" << std::endl;
          }else{
            movLeg = getRightLeg();
            statLeg = getLeftLeg();

            //std::cout << "The right leg is moving" << std::endl;
          }

          double alpha = cos(min(this->getStepWidth()/this->maxStepWidth_,1.0) * M_PI)/2.0 + 0.5;

          //std::cout << "ALPHA" << alpha << std::endl;

          assert(alpha >= 0.0);
          assert(alpha <= 1.0);

          // StdCOUT propagation information
          //std::cout << "PROPAGATION____________________" << std::endl;
          //std::cout << "ALPHA" << alpha << std::endl;
          //std::cout << "LEG MOVING: " << movLeg->int_id_ << std::endl;
          //std::cout << "LEG STATIC: " << statLeg->int_id_ << std::endl;

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
  double leg_time_threshold = 0.08;
  double min_leg_time = min(getLeg0()->getLifetime(), getLeg1()->getLifetime());

  leg_time_probability_ = sigmoid(min_leg_time,2,leg_time_threshold);

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

  if(this->getTotalProbability() > 0.5){

  BFL::StatePosVel est = getEstimate();

  boost::shared_ptr<tf::Stamped<tf::Point> > point(new tf::Stamped<tf::Point>());
  point->setX( est.pos_[0]);
  point->setY( est.pos_[1]);
  point->setZ( est.pos_[2]);
  point->stamp_ = time;

  position_history_.push_back(point);
  }
}

/**
 * Get the current estimation of this tracker, the speed and position is calculated using the speed and position of both legs
 * @return
 */
BFL::StatePosVel PeopleTracker::getEstimate(){
  ROS_DEBUG_COND(DEBUG_PEOPLE_TRACKER,"PeopleTracker[%i-%i]::%s", this->getLeg0()->int_id_, this->getLeg1()->int_id_, __func__);


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
  return position_history_.size();
}

std::vector<boost::shared_ptr<tf::Stamped<tf::Point> > >  PeopleTracker::getHistory(){
  return position_history_;
}

std::vector<tf::Vector3> PeopleTracker::getEstimationLines(int NumberOfLines, double angle_inkrement){
  // Check that the Number of Lines is unequal
  ROS_ASSERT(NumberOfLines % 2 != 0);

  tf::Vector3 mainLine = this->getEstimateKalman().vel_;

  //std::cout << "mainLine x:" << mainLine.getX() << " y:" << mainLine.getY() << " z:" << mainLine.getZ() << std::endl;


  std::vector<tf::Vector3> lines_vec;
  tf::Vector3 rotationVector(0,0,1);

  for(size_t i = 0; i < NumberOfLines; i++){
    double angle = pow(-1,(i % 2)) * ceil(i/2.0) * angle_inkrement;
    //std::cout << "angle" << angle << std::endl;


    tf::Vector3 newLine = mainLine;
    newLine = newLine.rotate(rotationVector, angle);
    newLine = newLine.normalized();

    //std::cout << "newline x:" << newLine.getX() << " y:" << newLine.getY() << " z:" << newLine.getZ() << std::endl;


    lines_vec.push_back(newLine);
  }
  //std::cout << "-------" << std::endl;
  //ROS_ASSERT(false);

  return lines_vec;
}

/***
 * Broadcast the position of humans in a tf
 * @param time
 */
void PeopleTracker::broadCastTf(ros::Time time){
  tf::Transform transform;

  tf::Quaternion q;
  q.setRPY(0,0,0);


  transform.setOrigin( tf::Vector3(this->getEstimate().pos_[0], this->getEstimate().pos_[1], 0.0) );
  transform.setRotation(q);

  br.sendTransform(tf::StampedTransform(transform, time, "odom_combined", this->getName()));

  std::cout << this->getName() << " published transform!" << std::endl;

}


/**
 * Calculate the energy function of this tracker
 */
void PeopleTracker::calculateEnergyFunction(boost::shared_ptr<std::vector<PeopleTrackerPtr> > list){
  std::cout << BOLDBLUE << "Calculating Energy function!!!!!" << RESET << std::endl;

  double sigma_d = 1;
  double beta = 1; // peakness of the weighting function

  Eigen::Vector2d nextDesiredVelocity;
  nextDesiredVelocity.setZero();

  double interactionEnergySum = 0;

  // Get the position of this person
  BFL::StatePosVel posvelSelf = this->getEstimate();

  Eigen::Vector2d posSelf;
  Eigen::Vector2d velSelf;

  posSelf(0) = posvelSelf.pos_[0];
  posSelf(1) = posvelSelf.pos_[1];
  velSelf(0) = posvelSelf.vel_[0];
  velSelf(1) = posvelSelf.vel_[1];

  // Iterate through the People Tracker
  for(std::vector<PeopleTrackerPtr>::iterator peopleTrackerIt = list->begin(); peopleTrackerIt != list->end(); peopleTrackerIt++){




    if((*peopleTrackerIt)->getTotalProbability() > 0.5){
      if(*this == **peopleTrackerIt){
        std::cout << "I am:__________";
      }
      else{
        double wd = 1.0;
        double wr = 1.0;
        double sigma_d = 1;

        Eigen::Vector2d pos;
        Eigen::Vector2d vel;


        // Get the position of this person
        BFL::StatePosVel posvel = (*peopleTrackerIt)->getEstimate();

        pos(0) = posvel.pos_[0];
        pos(1) = posvel.pos_[1];
        vel(0) = posvel.vel_[0];
        vel(1) = posvel.vel_[1];

        Eigen::Vector2d k = posSelf-pos;
        Eigen::Vector2d q = velSelf-vel;

        double d_square = (k-((k.dot(q))/q.squaredNorm()) * q).squaredNorm();

        interactionEnergySum += wd * wr * exp(- d_square / (2* sigma_d));

      }
      std::cout << (**peopleTrackerIt) << std::endl;
    }

  }

  std::cout << "interactionEnergySum " << interactionEnergySum << std::endl;
}


bool operator== (PeopleTracker &p0, PeopleTracker &p1)
{
  return (p0.getName() == p1.getName());
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

BFL::StatePosVel PeopleTrackerList::getEstimationFrom(std::string name){
  for(std::vector<PeopleTrackerPtr>::iterator peopleTrackerIt = list_->begin(); peopleTrackerIt != list_->end(); peopleTrackerIt++){
    if((*peopleTrackerIt)->getName() ==  name){
      return (*peopleTrackerIt)->getEstimate();
    }
  }
}

void PeopleTrackerList::calculateEnergys(){
  for(std::vector<PeopleTrackerPtr>::iterator peopleTrackerIt = list_->begin(); peopleTrackerIt != list_->end(); peopleTrackerIt++){

    if((*peopleTrackerIt)->getTotalProbability() > 0.5)
      (*peopleTrackerIt)->calculateEnergyFunction(this->getList());
  }
}





