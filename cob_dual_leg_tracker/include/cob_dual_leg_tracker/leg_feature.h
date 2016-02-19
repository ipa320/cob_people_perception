/*
 * features.h
 *
 *  Created on: Apr 15, 2015
 *      Author: frm-ag
 */

#ifndef LEG_FEATURE_H_
#define LEG_FEATURE_H_

// ROS includes
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <tf/transform_listener.h>

// Own includes
#include <cob_dual_leg_tracker/advanced_tracker_particle.h>
#include <cob_dual_leg_tracker/visualization/color_definitions.h>
#include <cob_dual_leg_tracker/people_tracker.h>
#include <cob_dual_leg_tracker/detection/detection.h>
#include <cob_dual_leg_tracker/config_struct.h>
#include <cob_dual_leg_tracker/benchmarking/timer.h>

// People Stack
#include <people_tracking_filter/state_pos_vel.h>
#include <people_tracking_filter/rgb.h>

// Default variables
#define DEBUG_LEG_TRACKER 0

class PeopleTracker; // Forward declaration
typedef boost::shared_ptr<PeopleTracker> PeopleTrackerPtr; // Forward declaration

class LegFeature; // Forward declaration
typedef boost::shared_ptr<LegFeature> LegFeaturePtr;

typedef std::vector<boost::shared_ptr<tf::Stamped<tf::Point> > > LegHistory;


/**
 *  \brief The low level tracker to track each leg
 */
class LegFeature
{
private:

  static int nextid;           /**< Id counter */

  int int_id_;     /**< Id of the instance */

  ros::Time time_last_update_; /**< Time of the last scan */

  ros::Time time_last_prediction_; /**< The time the prediction was made to */

  //ros::Time time_update_; /**< The time of the last measurement */

  double reliability_; /**< Reliability */

  bool is_valid_; /**< True if valid (otherwise marked for deletion) */

  bool is_static_; /**< Flag that is set the true after a certain motion has been observed */

  std::string fixed_frame_;    /**< The fixed frame to use */

  std::vector<PeopleTrackerPtr> peopleTrackerList_; /**< List of associated people trackers */

  BFL::StatePosVel sys_sigma_; /**< System variance */

  tf::Stamped<tf::Point> meas_loc_last_update_; /**< The measurement used in the last update */

  OcclusionModelPtr occlusion_model_; /**< Occlusion model according to the rays (Not used!) */

  double leg_feature_update_cov_; /**< The measurement update covariance */
  double leg_feature_predict_pos_cov_; /**< The prediction position covariance */
  double leg_feature_predict_vel_cov_; /**< The prediction velocity covariance */

  double leg_feature_measurement_cov_; /**< The leg measurement covariance */

  bool use_highlevel_prediction_; /**< Flag whether high level prediction should be used */

  tf::Stamped<tf::Point> position_; /**< The currently estimated leg position */

  tf::Stamped<tf::Point> position_predicted_; /**< The currently estimated leg position */

  tf::Stamped<tf::Point> position_updated_; /**< The currently estimated leg position */

  tf::Stamped<tf::Point> initial_position_; /**< The initial position */

  std::vector<boost::shared_ptr<tf::Stamped<tf::Point> > > position_history_; /**< History of this leg */

  estimation::AdvancedTrackerParticle filter_; /**< The particle filter */

  tf::TransformListener& tfl_; /**< Associated transform listener */

  double min_people_probability_for_hl_prediction_; /**< Min Probability the Associated PeopleTracker needs for a highlevel update */

  double static_threshold_distance_; /**< The distance the leg has to move at least to be considered dynamic */

  mutable BFL::StatePosVel current_estimate_;

  mutable bool current_estimate_changed_;


  // Variables used to enable multithread propagation
  bool prepared_for_propagation_;

  bool prepared_do_hl_propagation_;
  double prepared_gait_factor_;
  tf::Vector3 prepared_hip_vector_;
  double prepared_ppl_total_probability_;
  StatePosVel prepared_estimation_;

public:
  /**
   * Constructor
   * @param loc Initial location
   * @param tfl TransformListener to use
   */
  LegFeature(tf::Stamped<tf::Point> loc,
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
             double velocity_factor);

  ~LegFeature();

  /**
   * Return the id
   * @return id
   */
  int getId() const {
    return this->int_id_;
  }

  /**
   * Return the time of the last processed scan
   * @return
   */
  ros::Time getLastScanTime() const{
    return this->time_last_update_;
  }

  /**
   * Return the time of the last prediction
   * @return
   */
  ros::Time getLastPredictionTime() const{
    return this->time_last_prediction_;
  }

  /**
   * Return the last measurement time
   * @return
   */
  ros::Time getLastUpdateTime() const{
    return this->time_last_update_;
  }

  /**
   * Return the reliability of this legtracker
   * @return
   */
  double getReliability() const
  {
    return reliability_;
  }

  /**
   * Check if the leg tracker is valid
   * @return True if tracker is valid, False otherwise
   */
  bool isValid() const{
    return is_valid_;
  }

  /**
   * Check if the leg is static (didn't move since detection)
   * @return True if static
   */
  bool isStatic() const{
    return is_static_;
  }

  /**
   * Check if leg is dynamic (moved since detection)
   * @return
   */
  bool isDynamic() const{
    return !is_static_;
  }

  /**
   * Return the fixed frame
   * @return
   */
  std::string getFixedFrame() const{
    return this->fixed_frame_;
  }

  /**
   * Set the system variance
   * @param system_sigma
   */
  void setSystemSigma(BFL::StatePosVel system_sigma){
    this->sys_sigma_ = system_sigma;
  }

  /**
   * Get the location of the last measurement (mostly used for visualization/debugging)
   * @return Stamped Point of the last measurement position
   */
  tf::Stamped<tf::Point> getLocationOfLastMeasurementUpdate() const{
    return this ->meas_loc_last_update_;
  }

  /**
   * Return the current position of this leg
   * @return The current position
   */
  tf::Stamped<tf::Point> getPosition() const{
    return this->position_;
  }

  /**
   * Return the current predicted position of this leg
   * @return The current predicted position
   */
  tf::Stamped<tf::Point> getPredictedPosition() const{
    return this->position_predicted_;
  }

  tf::Stamped<tf::Point> getInitialPosition() const{
    return this->initial_position_;
  }

  /**
   * Prepare the propagation for multithreading by gathering dependend variables into local member variables
   * @param time
   */
  void preparePropagation(ros::Time time);

  /**
   * Propagate/Predict to a given time
   * @param time
   */
  void propagate(ros::Time time);

  /**
   * Update with measurement
   * @param loc Location of the measurement
   * @param probability Probability of the measurement
   */
  void update(tf::Stamped<tf::Point> loc, double probability);

  /**
   * Update the configuration/parameters of the tracker and the associated filter
   * @param filter_config
   */
  void configure(config_struct filter_config);

  //void JPDAUpdate(std::vector<DetectionPtr>& detections, Eigen::VectorXd& probabilities, OcclusionModelPtr occlusionModel, ros::Time measTime);
  /**
   * Get the occlusion probability (currently not used), maybe effective to avoid leg switch
   * @param occlusionModel
   * @return
   */
  double getOcclusionProbability(OcclusionModelPtr occlusionModel) ;

  /**
   * Get the probability of a measurement given this leg
   * @param loc Location of the measurement
   * @return
   */
  double getMeasurementProbability(tf::Stamped<tf::Point> loc);

  /**
   * Get the lifetime of this leg tracker
   * @return
   */
  double getLifetime() const
  {
    return this->getFilter().getLifetime();
  }

  /**
   * Set the validity for this tracker
   * @param valid True if valid, False otherwise (should lead to tracker deletion)
   */
  void setValidity(bool valid){
    is_valid_ = valid;
  }

  /**
   * Return the estimation
   * @return
   */
  BFL::StatePosVel getEstimate() const{

    //ROS_DEBUG_COND(DEBUG_LEG_TRACKER,"LegFeature::%s ID:%i", __func__, int_id_);

    if(this->current_estimate_changed_){
      BFL::StatePosVel est;
      filter_.getEstimate(est);
      current_estimate_ = est;
      this->current_estimate_changed_ = false;
    }
    else{
      //std::cout << "I was able to use old value" << std::endl;
    }

    if(abs(current_estimate_.pos_.getX()) >= 10000 || abs(current_estimate_.pos_.getX()) >= 10000){
      std::cout << RED << "Something went wrong with LegFeature[" << int_id_ << "]" << RESET << std::endl;
    }

    ROS_ASSERT(abs(current_estimate_.pos_.getX()) < 10000);
    ROS_ASSERT(abs(current_estimate_.pos_.getY()) < 10000);

    return current_estimate_;
    //return pos_vel_;
  }

  const std::vector<boost::shared_ptr<tf::Stamped<tf::Point> > >&  getHistory() const{
    return position_history_;
  }

  MCPdf<StatePosVel>* postGet() const{
    return this->getFilter().postGet();
  }

  /**
   *  @brief The distance between two legs.
   *
   *  Calculates the euclidian distance between to features(legs)
   */
  static double distance(LegFeaturePtr leg0,  LegFeaturePtr leg1);

  /**
   * Add a People Tracker to associated with this leg, every leg holds its associated People Trackers
   * @param peopleTracker
   */
  void addPeopleTracker(PeopleTrackerPtr);

  std::vector<PeopleTrackerPtr> getAssociatedPeopleTracker() const{
    return peopleTrackerList_;
  }

  void setOcclusionModel(OcclusionModelPtr ocm){
    occlusion_model_ = ocm;
    this->getFilter().setOcclusionModel(ocm);
  }

  // Remove People Tracker that are invalid from the associations list
  void removeInvalidAssociations();

  /// output stream
  friend std::ostream& operator<< (std::ostream& os, const LegFeature& s)
  {

    os << "LegFeature: " << s.getId();

    if(s.isValid())
      os << BOLDGREEN << " [valid]" << RESET;
    else
      os << BOLDRED << " [invalid]" << RESET;
    return os;
  };

  void updateHistory();

  /**
   * Return the distance between the last two positions
   * @return distance between the last two positions
   */
  double getLastPositionJumpWidth() const;

  std::string getIdStr() const{
    // Generate the string id
    char id[100];
    snprintf(id, 100, "LT[%d]", this->int_id_);

    return std::string(id);
  }

private:
  void updatePosition();

  estimation::AdvancedTrackerParticle& getFilter() {
    return this->filter_;
  }

  const estimation::AdvancedTrackerParticle& getFilter() const {
    return this->filter_;
  }

  /**
   * Return the most probable associated peopleTracker
   * @return
   */
  PeopleTrackerPtr getMostProbableAssociatedPeopleTracker() const;

};


#endif /* LEG_FEATURE_H_ */
