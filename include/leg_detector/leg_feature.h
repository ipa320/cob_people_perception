/*
 * features.h
 *
 *  Created on: Apr 15, 2015
 *      Author: frm-ag
 */

#ifndef LEG_FEATURE_H_
#define LEG_FEATURE_H_

#include <ros/ros.h>

// Own includes
// Configuration
#include <dynamic_reconfigure/server.h>
#undef DEFAULT
#include <leg_detector/DualTrackerConfig.h>
#define DEFAULT 0
#include <leg_detector/color_definitions.h>

// Transforms
#include <tf/transform_listener.h>

// People tracking
#include <people_tracking_filter/advanced_tracker_particle.h>
#include <people_tracking_filter/state_pos_vel.h>
#include <people_tracking_filter/rgb.h>



#include <leg_detector/people_tracker.h>

//using namespace std;
//using namespace ros;
//using namespace tf;
//using namespace estimation;
//using namespace BFL;
//using namespace MatrixWrapper;

// Default variables
#define DEBUG_LEG_TRACKER 1

class PeopleTracker; // Forward declaration
typedef boost::shared_ptr<PeopleTracker> PeopleTrackerPtr; // Forward declaration

class LegFeature; // Forward declaration
typedef boost::shared_ptr<LegFeature> LegFeaturePtr;

/**
 *  \brief The low level tracker to track each leg
 */
class LegFeature
{
public:
  static int nextid;
  tf::TransformListener& tfl_;
  std::string fixed_frame_;

  std::vector<PeopleTrackerPtr> peopleTrackerList_; /**< List of associated people trackers */

  BFL::StatePosVel sys_sigma_;
  estimation::AdvancedTrackerParticle filter_;

  std::string id_;
  std::string object_id;
  int int_id_;
  ros::Time time_;
  ros::Time meas_time_;

  double update_cov_;

  bool is_valid_;

  bool is_static_; /**< Flag that is set the true after a certain motion has been observed */

  double reliability, p;

  bool use_filter_; /**< Flag if the Filter should be used currently */

  tf::Stamped<tf::Point> position_; /**< The currently estimated leg position */
  tf::Stamped<tf::Point> initial_position_; /**< The initial position */

  BFL::StatePosVel pos_vel_; /**< The currently estimated pos_vel_ */

  std::vector<boost::shared_ptr<tf::Stamped<tf::Point> > > position_history_;

  dynamic_reconfigure::Server<leg_detector::DualTrackerConfig> server_; /**< The configuration server*/
  void configure(leg_detector::DualTrackerConfig &config, uint32_t level); /**< Configuration config */
  //LegFeaturePtr other;
  //float dist_to_person_;

  LegFeature(tf::Stamped<tf::Point> loc, tf::TransformListener& tfl);

  ~LegFeature();

  void propagate(ros::Time time);

  void update(tf::Stamped<tf::Point> loc, double probability);

  double getLifetime()
  {
    return filter_.getLifetime();
  }

  double getReliability()
  {
    return reliability;
  }

  void setValidity(bool valid){
    is_valid_ = valid;
  }

  bool isValid() const{
    return is_valid_;
  }

  bool isStatic(){
    return is_static_;
  }

  bool isDynamic(){
    return !is_static_;
  }

  BFL::StatePosVel getEstimate(){
    return pos_vel_;
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

private:
  void updatePosition();
};


#endif /* LEG_FEATURE_H_ */
