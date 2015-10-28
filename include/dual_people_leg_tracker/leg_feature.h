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
#include <dual_people_leg_tracker/advanced_tracker_particle.h>
#include <dual_people_leg_tracker/visualization/color_definitions.h>
#include <dual_people_leg_tracker/people_tracker.h>
#include <dual_people_leg_tracker/detection/detection.h>
#include <dual_people_leg_tracker/config_struct.h>

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
  BFL::StatePosVel pos_vel_; /**< The currently estimated pos_vel_ */

  double reliability_; /**< Reliability */

  bool is_valid_; /**< True if valid (otherwise marked for deletion) */

  bool is_static_; /**< Flag that is set the true after a certain motion has been observed */

  std::string fixed_frame_;    /**< The fixed frame to use */

  static int nextid;           /**< Id counter */

public:

  estimation::AdvancedTrackerParticle filter_; /**< The particle filter */

  tf::TransformListener& tfl_; /**< Associated transform listener */


  std::vector<PeopleTrackerPtr> peopleTrackerList_; /**< List of associated people trackers */

  BFL::StatePosVel sys_sigma_; /**< System variance */


  int int_id_;     /**< Id of the instance */
  ros::Time time_; /**< Time of the last scan */
  ros::Time time_prediction_; /**< The time the prediction was made to */
  ros::Time meas_time_; /**< The time of the last measurement */
  tf::Stamped<tf::Point> meas_loc_last_update_; /**< The measurement used in the last update */

  OcclusionModelPtr occlusion_model_; /**< Occlusion model according to the rays (Not used!) */

  double leg_feature_update_cov_; /**< The measurement update covariance */
  double leg_feature_predict_pos_cov_; /**< The prediction position covariance */
  double leg_feature_predict_vel_cov_; /**< The prediction velocity covariance */

  double leg_feature_measurement_cov_; /**< The leg measurement covariance */


  bool use_highlevel_prediction; /**< Flag whether high level prediction should be used */

  bool use_filter_; /**< Flag if the Filter should be used currently */

  tf::Stamped<tf::Point> position_; /**< The currently estimated leg position */
  tf::Stamped<tf::Point> position_predicted_; /**< The currently estimated leg position */
  tf::Stamped<tf::Point> position_updated_; /**< The currently estimated leg position */

  tf::Stamped<tf::Point> initial_position_; /**< The initial position */

  std::vector<boost::shared_ptr<tf::Stamped<tf::Point> > > position_history_; /**< History of this leg */

  /**
   * Constructor
   * @param loc Initial location
   * @param tfl TransformListener to use
   */
  LegFeature(tf::Stamped<tf::Point> loc, tf::TransformListener& tfl);

  ~LegFeature();

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
  double getOcclusionProbability(OcclusionModelPtr occlusionModel);

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
  double getLifetime()
  {
    return filter_.getLifetime();
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
   * Set the validity for this tracker
   * @param valid True if valid, False otherwise (should lead to tracker deletion)
   */
  void setValidity(bool valid){
    is_valid_ = valid;
  }

  /**
   * Check if the leg tracker is valid
   * @return True if tracker is valid, False otherwise
   */
  bool isValid() const{
    return is_valid_;
  }

  /**
   * Check if the leg is static (didnt move since detection)
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

  BFL::StatePosVel getEstimate(){
    BFL::StatePosVel est;
    filter_.getEstimate(est);
    return est;
    //return pos_vel_;
  }

  unsigned int getHistorySize(){
    return position_history_.size();
  }

  std::vector<boost::shared_ptr<tf::Stamped<tf::Point> > >  getHistory(){
    return position_history_;
  }

  static double distance(LegFeaturePtr leg0,  LegFeaturePtr leg1);

  void addPeopleTracker(PeopleTrackerPtr);

  std::vector<PeopleTrackerPtr> getPeopleTracker(){
    return peopleTrackerList_;
  }

  void setOcclusionModel(OcclusionModelPtr ocm){
    occlusion_model_ = ocm;
    this->filter_.setOcclusionModel(ocm);
  }

  std::string getFixedFrame() const{
    return this->fixed_frame_;
  }

  // Remove People Tracker that are invalid from the associations list
  void removeInvalidAssociations();

  /// output stream
  friend std::ostream& operator<< (std::ostream& os, const LegFeature& s)
  {

    os << "LegFeature: " << s.int_id_;

    if(s.isValid())
      os << BOLDGREEN << " [valid]" << RESET;
    else
      os << BOLDRED << " [invalid]" << RESET;
    return os;
  };

  void updateHistory();

  double getLastPositionJumpWidth();

  int getId() const {
	  return this->int_id_;
  }

  std::string getIdStr() const{
    // Generate the string id
    char id[100];
    snprintf(id, 100, "legtrack%d", this->int_id_);

    return std::string(id);
  }

private:
  void updatePosition();


};


#endif /* LEG_FEATURE_H_ */
