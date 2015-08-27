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

static int NumberOfParticles = 700;

/*LegFeature::LegFeature(tf::Stamped<tf::Point> loc, tf::TransformListener& tfl, OcclusionModelPtr ocm)
  :LegFeature(loc,tfl),
   occlusion_model_(ocm)
{

}*/

// The is the one leg tracker
LegFeature::LegFeature(tf::Stamped<tf::Point> loc, tf::TransformListener& tfl)
  : tfl_(tfl),
    leg_feature_predict_pos_cov_(0.2), //0.4 Around 0.05 // Variance of the
    leg_feature_predict_vel_cov_(1.2),  //1.8 Around 1.0 should be fine, the bigger the more spread
    sys_sigma_(tf::Vector3(leg_feature_predict_pos_cov_, leg_feature_predict_pos_cov_, 0.0), tf::Vector3(leg_feature_predict_vel_cov_, leg_feature_predict_vel_cov_, 0.0)), // The initialized system noise(the variance)
    filter_("tracker_name", NumberOfParticles, sys_sigma_), // Name, NumberOfParticles, Noise
    //reliability(-1.), p(4),
    use_filter_(true),
    is_valid_(true), // On construction the leg feature is always valid
    leg_feature_update_cov_(0.05), // The update measurement cov (should be around 0.0025, the smaller the peakier)
    is_static_(true) // At the beginning the leg feature is considered static
{
  // Increase the id
  int_id_ = nextid++;

  // Generate the string id
  char id[100];
  snprintf(id, 100, "legtrack%d", int_id_);
  id_ = std::string(id);

  // Configuration server
  // dynamic_reconfigure::Server<leg_detector::DualTrackerConfig>::CallbackType f;
  // f = boost::bind(&LegFeature::configure, this, _1, _2);


  ROS_DEBUG_COND(DEBUG_LEG_TRACKER,"LegFeature::%s Created <NEW_LEGFEATURE %s> at %f - %f - %f", __func__, id_.c_str(), loc.getX(), loc.getY(), loc.getZ());

  object_id = "";
  time_ = loc.stamp_;
  meas_time_ = loc.stamp_;
  //other = NULL;

  try
  {
    tfl_.transformPoint(fixed_frame, loc, loc);
  }
  catch (...)
  {
    ROS_WARN("TF exception spot 6.");
  }

  // Create transform to the tracker position
  tf::StampedTransform pose(tf::Pose(tf::Quaternion(0.0, 0.0, 0.0, 1.0), loc), loc.stamp_, id_, loc.frame_id_);
  tfl_.setTransform(pose);

  // Initialize the filter (Create the initial particles)
  //BFL::StatePosVel prior_sigma(tf::Vector3(0.1, 0.1, 0.0), tf::Vector3(0.0000001, 0.0000001, 0.000000));

  double maxSpeed = 3; // [m/T] T = Period

  double sigmaSpeed = maxSpeed / 2.0; // Because we want the two sigma area

  BFL::StatePosVel prior_sigma(tf::Vector3(0.2, 0.2, 0.0), tf::Vector3(sigmaSpeed, sigmaSpeed, 0.000000));

  // Initialization is around the measurement which initialized this leg feature using a uniform distribution
  BFL::StatePosVel mu(loc);
  filter_.initialize(mu, prior_sigma, time_.toSec());

  // Get the first estimation of the particle state
  BFL::StatePosVel est;
  filter_.getEstimate(est);

  // Set the initial position
  initial_position_[0] = loc.getX();
  initial_position_[1] = loc.getY();
  initial_position_[2] = loc.getZ();

  position_predicted_[0] = loc.getX();
  position_predicted_[1] = loc.getY();
  position_predicted_[2] = loc.getZ();

  //leg_feature_update_cov_    = config.leg_feature_update_cov;     ROS_DEBUG_COND(DEBUG_LEG_TRACKER, "DEBUG_LEG_TRACKER::%s - leg_feature_update_cov_ %f", __func__, leg_feature_update_cov_ );
  //leg_feature_predict_pos_cov_    = config.leg_feature_predict_pos_cov;     ROS_DEBUG_COND(DEBUG_LEG_TRACKER, "DEBUG_LEG_TRACKER::%s - leg_feature_predict_pos_cov_ %f", __func__, leg_feature_predict_pos_cov_ );
  //leg_feature_predict_vel_cov_    = config.leg_feature_predict_vel_cov;     ROS_DEBUG_COND(DEBUG_LEG_TRACKER, "DEBUG_LEG_TRACKER::%s - leg_feature_predict_vel_cov_ %f", __func__, leg_feature_predict_vel_cov_ );


  // Update the position of this leg feature
  updatePosition();
}

/**
 * Destructor
 */
LegFeature::~LegFeature(){
  //peopleTrackerList_.clear();

  ROS_DEBUG_COND(DEBUG_LEG_TRACKER,"LegFeature::%s <DELETE_LEGFEATURE %s>", __func__, id_.c_str());
}

/**
 * Propagate the Position of the Feature using the Particle Filter
 */
void LegFeature::propagate(ros::Time time)
{
  ROS_DEBUG_COND(DEBUG_LEG_TRACKER,"LegFeature::%s ID:%i", __func__, int_id_);

  // Update the time
  time_ = time;
  time_prediction_ = time;

  // Get the associated people tracker with the highest probability
  PeopleTrackerPtr mostProbableAssociatedPPL;
  double max_prob = 0.0;

  std::vector<PeopleTrackerPtr> associatedPT = this->getPeopleTracker();

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

  MatrixWrapper::SymmetricMatrix cov(6);
  cov = 0.0;
  cov(1, 1) = leg_feature_predict_pos_cov_;//conf.leg_feature_predict_pos_cov;
  cov(2, 2) = leg_feature_predict_pos_cov_;//conf.leg_feature_predict_pos_cov;
  cov(3, 3) = 0.0;
  cov(4, 4) = leg_feature_predict_vel_cov_;//conf.leg_feature_predict_vel_cov;
  cov(5, 5) = leg_feature_predict_vel_cov_;//conf.leg_feature_predict_vel_cov;
  cov(6, 6) = 0.0;

  // Update of the moving leg
  if(mostProbableAssociatedPPL                                          // If there is a associated PPLTracker
     && mostProbableAssociatedPPL->isValid()                            // The associated PPLTracker has to be valid
     && mostProbableAssociatedPPL->getTotalProbability() > 0.6          // If it has a certain Probability   TODO make variable
     && mostProbableAssociatedPPL->isDynamic()                          // If the person is moving
     && mostProbableAssociatedPPL->getMovingLeg()->getId() == getId())  // If this is the moving leg
  {

    //std::cout << RED << "Updating L" << this->int_id_ << " most probable associatet people tracker is" << *mostProbableAssociatedPPL << RESET << std::endl;
    ROS_DEBUG_COND(DEBUG_LEG_TRACKER,"LegFeature::%s ID:%i considers a high level filter for its update", __func__, int_id_);

    // Check that the high level filter was propagated to this time
    ROS_ASSERT((mostProbableAssociatedPPL->propagation_time_  - time).isZero());

    // Get the estimation of the associated people tracker
    StatePosVel est = mostProbableAssociatedPPL->getEstimate();

    // Get the current StepWidth
    double s = mostProbableAssociatedPPL->getStepWidth();

    // Get the maximum StepWidth
    double s_max = mostProbableAssociatedPPL->getStepWidthMax();

    // Calculate the factor depending if the leg is the back or the front leg
    double factor = 0;
    if(mostProbableAssociatedPPL->getBackLeg()->getId() == this->int_id_){
      factor = s/s_max;
      std::cout << "LT[" << int_id_ << "] is the back leg and moving! " << s/s_max * 100 << "% done of this step, factor:" << factor << std::endl;
    }else{
      factor = - s/s_max;
      std::cout << "LT[" << int_id_ << "] is the front leg and moving!" << s/s_max * 100 << "% done of this step, factor:" << factor << std::endl;
    }

    // Do the prediction with HighLevel Influence
    filter_.updatePrediction(time.toSec(), cov, factor, est.vel_, mostProbableAssociatedPPL->getHipVec(), mostProbableAssociatedPPL->getTotalProbability());

  }

  // If there is no relevant people tracker assigned-> Consider only the low level filter
  // OR if this is the static leg
  else
  {
    ROS_DEBUG_COND(DEBUG_LEG_TRACKER,"LegFeature::%s ID:%i does a simple update", __func__, int_id_);

    filter_.updatePrediction(time.toSec(),cov);
  }

  //TODO special case for the standing leg?! use a really low variance for it?

  // Set the predicted Position of this tracker, mainly for debugging purposes
  position_predicted_ = position_;

  ROS_DEBUG_COND(DEBUG_LEG_TRACKER,"LegFeature::%s Done Propagating leg_tracker with ID %s", __func__, id_.c_str());

}

// Here the measurement is used to update the filter location
void LegFeature::update(tf::Stamped<tf::Point> loc, double probability)
{
  ROS_DEBUG_COND(DEBUG_LEG_TRACKER,"LegFeature[%i]::%s",int_id_,__func__);
  //std::cout << "Received update: " << loc.getX() << "  " << loc.getY() << "  " << loc.getZ() << std::endl;

  meas_loc_last_update_ = loc;

  // Set the tf to represent this
  //tf::StampedTransform pose(tf::Pose(tf::Quaternion(0.0, 0.0, 0.0, 1.0), loc), loc.stamp_, id_, loc.frame_id_);
  //tfl_.setTransform(pose);

  // Update the measurement time
  meas_time_ = loc.stamp_;
  time_ = meas_time_;

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
/**
 * Perform a update using the probabilities calculated by the JPDA
 * @param detections    // The current detections
 * @param probabilities // The assignment probabilities (The first entry is the occlusion probability!)
 */
void LegFeature::JPDAUpdate(std::vector<DetectionPtr>& detections, Eigen::VectorXd& assignmentProbabilities, OcclusionModelPtr occlusionModel, ros::Time measTime){
  ROS_DEBUG_COND(DEBUG_LEG_TRACKER,"LegFeature::%s [%i]",__func__, int_id_);
  //std::cout << "LT" << this->int_id_ << " - JPDAUpdate" << std::endl;
  //std::cout << "Probabilities" << std::endl << assignmentProbabilities << std::endl;

  meas_time_ = measTime;
  time_ = measTime;

  // Covariance of the Measurement
  MatrixWrapper::SymmetricMatrix cov(3);
  cov(1, 1) = leg_feature_update_cov_;
  cov(2, 2) = leg_feature_update_cov_;
  cov(3, 3) = leg_feature_update_cov_;

  filter_.updateJPDA(cov,detections,assignmentProbabilities, occlusionModel);
  //filter_.updateCorrection()

  // Update the position based on the latest measurements
  updatePosition();

  // Resample if necessary
  //filter_.dynamicResample();

  // Update history
  updateHistory();

}

double LegFeature::getOcclusionProbability(OcclusionModelPtr occlusionModel){
  ROS_DEBUG_COND(DEBUG_LEG_TRACKER,"LegFeature::%s",__func__);

  // Check if occlusion model exists
  ROS_ASSERT(occlusionModel);


  return filter_.getOcclusionProbability(occlusionModel);
  //return occlusion_model_->getOcclusionProbability(loc);
}

double LegFeature::getMeasurementProbability(tf::Stamped<tf::Point> loc){
  ROS_DEBUG_COND(DEBUG_LEG_TRACKER,"LegFeature::%s",__func__);


  double leg_feature_measurement_cov_ = 0.004;

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

  BFL::StatePosVel est;

  // Estimate using the weighted mean
  filter_.getEstimate(est);

  // Estimate using the most likely particle
  // filter_.getMostLikelyPosition(est);

  //std::cout << pos_vel_ << " --Update--> ";
  pos_vel_ = est;
  //std::cout << pos_vel_ << std::endl;

  position_[0] = est.pos_[0];
  position_[1] = est.pos_[1];
  position_[2] = est.pos_[2];
  position_.stamp_ = time_;
  position_.frame_id_ = fixed_frame_;
  double nreliability = fmin(1.0, fmax(0.1, est.vel_.length() / 0.5)); //TODO ???????
  //reliability = fmax(reliability, nreliability);

  // Check if static
  double static_threshold = 0.4; // TODO make this configurable
  if((initial_position_-position_).length() > static_threshold){
    this->is_static_ = false;
  }
}

void LegFeature::updateHistory()
{
  ROS_DEBUG_COND(DEBUG_LEG_TRACKER,"LegFeature::%s",__func__);

  // Update history
  BFL::StatePosVel est;
  filter_.getEstimate(est);

  boost::shared_ptr<tf::Stamped<tf::Point> > point(new tf::Stamped<tf::Point>());
  point->setX( est.pos_[0]);
  point->setY( est.pos_[1]);
  point->setZ( est.pos_[2]);
  point->stamp_ = time_;

  position_history_.push_back(point);
}

bool LegFeature::getLastStepWidth(double& width){

  if(getHistorySize()<2){
    return false;
  }


  unsigned int histSize = getHistorySize();

  width = (*getHistory()[histSize-1] - *getHistory()[histSize-2]).length();

  return true;

}

// TODO do this static
/**
 *  @brief The distance between two legs.
 *
 *  Calculates the euclidian distance between to features(legs)
 */
double LegFeature::distance(LegFeaturePtr leg0,  LegFeaturePtr leg1){
    tf::Stamped<tf::Point> one = leg0->position_;
    tf::Stamped<tf::Point> two = leg1->position_;

    double distance = (one-two).length();
    return distance;
}

void LegFeature::removeInvalidAssociations(){
  peopleTrackerList_.erase(std::remove_if(peopleTrackerList_.begin(),peopleTrackerList_.end(), isValidPeopleTracker),peopleTrackerList_.end());
}

/**
 * Add a People Tracker to associated with this leg, every leg holds its associated People Trackers
 * @param peopleTracker
 */
void LegFeature::addPeopleTracker(PeopleTrackerPtr peopleTracker){
  // Add this tracker to the list
  this->peopleTrackerList_.push_back(peopleTracker);
}
