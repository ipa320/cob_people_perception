/*
 * features.cpp
 *
 *  Created on: Apr 15, 2015
 *      Author: frm-ag
 */


// Own includes
#include <dual_people_leg_tracker/leg_feature.h>
#include <dual_people_leg_tracker/advanced_tracker_particle.h>

int LegFeature::nextid = 0;

static std::string fixed_frame              = "odom_combined";  // The fixed frame in which ? //TODO find out

static int NumberOfParticles = 700; // TODO experiment around this


// Constructor
LegFeature::LegFeature(tf::Stamped<tf::Point> loc,
                       tf::TransformListener& tfl,
                       double leg_feature_predict_pos_cov,
                       double leg_feature_predict_vel_cov,
                       double leg_feature_update_cov,
                       double leg_feature_measurement_cov,
                       double initial_leg_feature_predict_pos_cov,
                       double initial_leg_feature_predict_vel_cov,
                       double min_people_probability_for_hl_prediction,
                       double static_threshold_distance,
                       double v_max,
                       double position_factor,
                       double velocity_factor)
  : tfl_(tfl),
    leg_feature_predict_pos_cov_(leg_feature_predict_pos_cov), //0.4 Around 0.05 // Variance of the
    leg_feature_predict_vel_cov_(leg_feature_predict_vel_cov),  //1.8 Around 1.0 should be fine, the bigger the more spread
    leg_feature_update_cov_(leg_feature_update_cov), // The update measurement cov (should be around 0.0025, the smaller the peakier)
    leg_feature_measurement_cov_(leg_feature_measurement_cov),
    min_people_probability_for_hl_prediction_(min_people_probability_for_hl_prediction),
    static_threshold_distance_(static_threshold_distance),
    sys_sigma_(tf::Vector3(leg_feature_predict_pos_cov_, leg_feature_predict_pos_cov_, 0.0), tf::Vector3(leg_feature_predict_vel_cov_, leg_feature_predict_vel_cov_, 0.0)), // The initialized system noise(the variance)
    filter_("tracker_name", NumberOfParticles, sys_sigma_, 4.0, 0.8, 1.6), // Name, NumberOfParticles, Noise
    is_valid_(true), // On construction the leg feature is always valid
    is_static_(true) // At the beginning the leg feature is considered static
{
  // Assign id
  int_id_ = nextid++;

  ROS_DEBUG_COND(DEBUG_LEG_TRACKER,"LegFeature::%s Created <NEW_LEGFEATURE %s> at %f - %f - %f", __func__, getIdStr().c_str(), loc.getX(), loc.getY(), loc.getZ());

  // Set the times to the initialization location
  time_last_scan_ = loc.stamp_;
  time_meas_ = loc.stamp_;

  // Transform to local frame
  try
  {
    tfl_.transformPoint(fixed_frame, loc, loc);
  }
  catch (...)
  {
    ROS_WARN("Could not transform to the fixed frame");
  }

  // Set the initial position
  initial_position_ = loc;
  position_predicted_ = loc;

  // Create transform to the tracker position
  tf::StampedTransform pose(tf::Pose(tf::Quaternion(0.0, 0.0, 0.0, 1.0), loc), loc.stamp_, getIdStr(), loc.frame_id_);
  tfl_.setTransform(pose);

  BFL::StatePosVel prior_sigma(
      tf::Vector3(initial_leg_feature_predict_pos_cov, initial_leg_feature_predict_pos_cov, 0.0),
      tf::Vector3(initial_leg_feature_predict_vel_cov, initial_leg_feature_predict_vel_cov, 0.0)
  );

  // Initialization is around the measurement which initialized this leg feature using a uniform distribution
  BFL::StatePosVel mu(loc);
  filter_.initialize(mu, prior_sigma, time_last_scan_.toSec());

  // Update the position of this leg feature
  updatePosition();
}

/**
 * Destructor
 */
LegFeature::~LegFeature(){
  ROS_DEBUG_COND(DEBUG_LEG_TRACKER,"LegFeature::%s <DELETE_LEGFEATURE %s>", __func__, getIdStr().c_str());
}

/**
 * Prepare for multithreading propagation by gathering the necessary parameters
 * @param time
 */
void LegFeature::preparePropagation(ros::Time time){
  ROS_DEBUG_COND(DEBUG_LEG_TRACKER,"LegFeature::%s ID:%i", __func__, int_id_);
  benchmarking::Timer preparePropagationTimer; preparePropagationTimer.start();

  // Get the associated people tracker with the highest probability
  PeopleTrackerPtr mostProbableAssociatedPPL = getMostProbableAssociatedPeopleTracker();

  // Update of the moving leg
  if(mostProbableAssociatedPPL                                          // If there is a associated PPLTracker
     && mostProbableAssociatedPPL->isValid()                            // The associated PPLTracker has to be valid
     && mostProbableAssociatedPPL->getTotalProbability() > min_people_probability_for_hl_prediction_         // If it has a certain Probability   TODO make variable
     && mostProbableAssociatedPPL->isDynamic()                          // If the person is moving
     && mostProbableAssociatedPPL->getMovingLeg()->getId() == getId())  // If this is the moving leg
  {

    //std::cout << RED << "Updating L" << this->int_id_ << " most probable associatet people tracker is" << *mostProbableAssociatedPPL << RESET << std::endl;
    ROS_DEBUG_COND(DEBUG_LEG_TRACKER,"LegFeature::%s ID:%i is preparing for a hl update", __func__, int_id_);

    // Check that the high level filter was propagated to this time
    ROS_ASSERT((mostProbableAssociatedPPL->getPropagationTime()  - time).isZero());

    // Get the estimation of the associated people tracker
    prepared_estimation_ = mostProbableAssociatedPPL->getEstimate();

    // Get the current StepWidth
    double s = mostProbableAssociatedPPL->getStepWidth();

    // Get the maximum StepWidth
    double s_max = mostProbableAssociatedPPL->getStepWidthMax();

    // Calculate the factor depending if the leg is the back or the front leg
    double factor = 0;

    // Back leg and moving
    if(mostProbableAssociatedPPL->getBackLeg()->getId() == this->int_id_){
      prepared_gait_factor_ = s/s_max;
      //std::cout << "LT[" << int_id_ << "] is the back leg and moving! " << s/s_max * 100 << "% done of this step, factor:" << factor << std::endl;
    }
    // Front leg and moving
    else{
      prepared_gait_factor_ = - s/s_max;
      //std::cout << "LT[" << int_id_ << "] is the front leg and moving!" << s/s_max * 100 << "% done of this step, factor:" << factor << std::endl;
    }

    prepared_do_hl_propagation_ = true;
    // Do the prediction with HighLevel Influence
    //filter_.updatePrediction(time.toSec(), cov, factor, est.vel_, mostProbableAssociatedPPL->getHipVec(), mostProbableAssociatedPPL->getTotalProbability());

  }

  // If there is no relevant people tracker assigned-> Consider only the low level filter
  // OR if this is the static leg
  else
  {
    ROS_DEBUG_COND(DEBUG_LEG_TRACKER,"LegFeature::%s ID:%i prepared for a simple update", __func__, int_id_);

    //filter_.updatePrediction(time.toSec(),cov);
    prepared_do_hl_propagation_ = false;
  }

  // set flag
  prepared_for_propagation_ = true;

  ROS_DEBUG_COND(DEBUG_LEG_TRACKER,"LegFeature::%s %s prepared for propagation - %f ms", __func__, getIdStr().c_str(), preparePropagationTimer.stopAndGetTimeMs());

}

/**
 * Propagate the Position of the Feature using the Particle Filter
 */
void LegFeature::propagate(ros::Time time)
{
  //ROS_DEBUG_COND(DEBUG_LEG_TRACKER,"LegFeature::%s ID:%i", __func__, int_id_);
  //ROS_ASSERT(prepared_for_propagation_);

  benchmarking::Timer propagationTimer; propagationTimer.start();

  // Set the estimate change to true
  current_estimate_changed_ = true;

  // Update the time
  time_last_scan_ = time;
  time_prediction_ = time;

  MatrixWrapper::SymmetricMatrix cov(6);
  cov = 0.0;
  cov(1, 1) = leg_feature_predict_pos_cov_;//conf.leg_feature_predict_pos_cov;
  cov(2, 2) = leg_feature_predict_pos_cov_;//conf.leg_feature_predict_pos_cov;
  cov(3, 3) = 0.0;
  cov(4, 4) = leg_feature_predict_vel_cov_;//conf.leg_feature_predict_vel_cov;
  cov(5, 5) = leg_feature_predict_vel_cov_;//conf.leg_feature_predict_vel_cov;
  cov(6, 6) = 0.0;

  // Update of the moving leg
  if(prepared_do_hl_propagation_){

    //std::cout << RED << "Updating L" << this->int_id_ << " most probable associatet people tracker is" << *mostProbableAssociatedPPL << RESET << std::endl;
    //ROS_DEBUG_COND(DEBUG_LEG_TRACKER,"LegFeature::%s ID:%i considers a high level filter for its update", __func__, int_id_);


    // Do the prediction with HighLevel Influence
    filter_.updatePrediction(time.toSec(),
                             cov,
                             prepared_gait_factor_,
                             prepared_estimation_.vel_,
                             prepared_hip_vector_,
                             prepared_ppl_total_probability_);

  }

  // If there is no relevant people tracker assigned-> Consider only the low level filter
  // OR if this is the static leg
  else
  {
    //ROS_DEBUG_COND(DEBUG_LEG_TRACKER,"LegFeature::%s ID:%i does a simple update", __func__, int_id_);

    filter_.updatePrediction(time.toSec(),cov);
  }

  // Set the predicted Position of this tracker, mainly for debugging purposes
  position_predicted_ = position_;

  prepared_for_propagation_ = false;

  //ROS_DEBUG_COND(DEBUG_LEG_TRACKER,"LegFeature::%s %s - propgated - %f ms", __func__, getIdStr().c_str(), propagationTimer.stopAndGetTimeMs());

}

// Here the measurement is used to update the filter location
void LegFeature::update(tf::Stamped<tf::Point> loc, double probability)
{
  ROS_DEBUG_COND(DEBUG_LEG_TRACKER,"LegFeature[%i]::%s",int_id_,__func__);
  //std::cout << "Received update: " << loc.getX() << "  " << loc.getY() << "  " << loc.getZ() << std::endl;

  // Set estimate change to true
  current_estimate_changed_ = true;

  meas_loc_last_update_ = loc;

  // Update the measurement time
  time_meas_ = loc.stamp_;
  time_last_scan_ = time_meas_;

  // Covariance of the Measurement
  MatrixWrapper::SymmetricMatrix cov(3);
  cov = 0.0;
  cov(1, 1) = leg_feature_update_cov_;
  cov(2, 2) = leg_feature_update_cov_;
  cov(3, 3) = leg_feature_update_cov_;

  filter_.updateCorrection(loc, cov);

  // Update the position based on the latest measurements
  updatePosition();

  // Update history
  updateHistory();
}

void LegFeature::configure(config_struct filter_config){
  ROS_DEBUG_COND(DEBUG_LEG_TRACKER,"LegFeature::%s",__func__);

  // TODO Implement
}

// Not used
double LegFeature::getOcclusionProbability(OcclusionModelPtr occlusionModel) {
  ROS_DEBUG_COND(DEBUG_LEG_TRACKER,"LegFeature::%s",__func__);

  // Check if occlusion model exists
  ROS_ASSERT(occlusionModel);

  return this->getFilter().getOcclusionProbability(occlusionModel);
}

double LegFeature::getMeasurementProbability(tf::Stamped<tf::Point> loc){
  ROS_DEBUG_COND(DEBUG_LEG_TRACKER,"LegFeature::%s",__func__);

  // Covariance of the Measurement
  MatrixWrapper::SymmetricMatrix cov(3);
  cov = 0.0;
  cov(1, 1) = leg_feature_measurement_cov_;
  cov(2, 2) = leg_feature_measurement_cov_;
  cov(3, 3) = leg_feature_measurement_cov_;

  return filter_.getMeasProbability(loc,cov);
}

// Update own position based on the Estimation of the Filter

/**
 * Update the Position(position_) of the Leg Feature using the latest estimate of the filter_
 */
void LegFeature::updatePosition()
{
  ROS_DEBUG_COND(DEBUG_LEG_TRACKER,"LegFeature[%d]::%s",(int)int_id_,__func__);

  // Set estimate change flag
  current_estimate_changed_ = true;

  // Estimate using the weighted mean
  BFL::StatePosVel est;
  filter_.getEstimate(est);

  // Estimate using the most likely particle
  // filter_.getMostLikelyPosition(est);

  position_[0] = est.pos_[0];
  position_[1] = est.pos_[1];
  position_[2] = est.pos_[2];
  position_.stamp_ = time_last_scan_;
  position_.frame_id_ = fixed_frame_;

  // Check if static
  if((initial_position_-position_).length() > static_threshold_distance_){
    is_static_ = false;
  }
}

void LegFeature::updateHistory(){
  ROS_DEBUG_COND(DEBUG_LEG_TRACKER,"LegFeature::%s",__func__);

  // Update history
  BFL::StatePosVel est;
  filter_.getEstimate(est);

  boost::shared_ptr<tf::Stamped<tf::Point> > point(
      new tf::Stamped<tf::Point>(est.pos_, time_last_scan_, fixed_frame_)
  );

  position_history_.push_back(point);
}


double LegFeature::getLastPositionJumpWidth() const{
  ROS_DEBUG_COND(DEBUG_LEG_TRACKER,"LegFeature::%s",__func__);

  // Return false if only one history is recorded
  if(this->getHistory().size() < 2){
    return 0;
  }

  return (*getHistory()[this->getHistory().size()-1] - *getHistory()[this->getHistory().size()-2]).length();
}

double LegFeature::distance(LegFeaturePtr leg0,  LegFeaturePtr leg1){
  ROS_DEBUG_COND(DEBUG_LEG_TRACKER,"LegFeature::%s",__func__);

  return (leg0->getPosition() - leg1->getPosition()).length();
}

void LegFeature::removeInvalidAssociations(){
  ROS_DEBUG_COND(DEBUG_LEG_TRACKER,"LegFeature::%s",__func__);

  peopleTrackerList_.erase(std::remove_if(peopleTrackerList_.begin(),peopleTrackerList_.end(), isValidPeopleTracker),peopleTrackerList_.end());
}

void LegFeature::addPeopleTracker(PeopleTrackerPtr peopleTracker){
  ROS_DEBUG_COND(DEBUG_LEG_TRACKER,"LegFeature::%s",__func__);
  // Add this tracker to the list
  this->peopleTrackerList_.push_back(peopleTracker);
}

PeopleTrackerPtr LegFeature::getMostProbableAssociatedPeopleTracker() const{
  PeopleTrackerPtr mostProbableAssociatedPPL;

  double max_prob = 0.0;

  std::vector<PeopleTrackerPtr> associatedPT = this->getAssociatedPeopleTracker();

  for(std::vector<PeopleTrackerPtr>::iterator pplIt = associatedPT.begin();
      pplIt != associatedPT.end();
      pplIt++)
  {
    if((*pplIt)->getTotalProbability() > max_prob){
      mostProbableAssociatedPPL = *pplIt;
      max_prob = (*pplIt)->getTotalProbability();
    }
    //std::cout << "\t" << **pplIt << std::endl;
  }

  return mostProbableAssociatedPPL;
}
