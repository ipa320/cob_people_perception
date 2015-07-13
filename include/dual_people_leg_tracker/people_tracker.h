/*
 * people_tracker.h
 *
 *  Created on: Apr 16, 2015
 *      Author: frm-ag
 */

#ifndef PEOPLE_LEG_DETECTOR_INCLUDE_LEG_DETECTOR_PEOPLE_TRACKER_H_
#define PEOPLE_LEG_DETECTOR_INCLUDE_LEG_DETECTOR_PEOPLE_TRACKER_H_

// ROS includes
#include <ros/ros.h>

// System includes
#include <vector>

// Boost includes
#include <boost/shared_ptr.hpp>
#include <boost/array.hpp>

// Own includes
#include <dual_people_leg_tracker/leg_feature.h>
#include <dual_people_leg_tracker/kalman/KalmanFilter.h>

#define DEBUG_PEOPLE_TRACKER 0
#define DEBUG_PEOPLETRACKERLIST 0

class LegFeature; //Forward declaration
typedef boost::shared_ptr<LegFeature> LegFeaturePtr;

class PeopleTracker; //Forward declaration
typedef boost::shared_ptr<PeopleTracker> PeopleTrackerPtr;

/**
 * High level People Tracker consisting of two low level leg tracks
 */
class PeopleTracker{
  public:
    // State Variables
    BFL::StatePosVel pos_vel_estimation_; /**< The currently estimated pos_vel_ of this people */

    tf::Vector3 hip_vec_; /**< Vector orthogonal to the velocity vector, representing the hip direction pos_vel_estimation.vel_.dot(hip_vec_) = 0 should hold every time*/

    tf::Vector3 hipPos0_, hipPos1_; /**< Vector of the endpoints of vector */
    tf::Vector3 hipPosLeft_, hipPosRight_; /**< Vector of the endpoints of vector */

    boost::array<int, 2> id_;

    std::vector<boost::shared_ptr<tf::Stamped<tf::Point> > > position_history_; /**< The position history of the people tracker */

    double hipWidth_;
    double stepWidth_;
    double stepWidthMax_;

    BFL::StatePosVel leg0Prediction_;
    BFL::StatePosVel leg1Prediction_;

    ros::Time propagation_time_; /**< Time the propagation is towards */

  private:
    bool is_static_;

    std::vector<LegFeaturePtr> legs_; /**< the legs, should be maximum 2! */

    LegFeaturePtr leftLeg_; /**< The left leg */
    LegFeaturePtr rightLeg_;/**< The right leg */
    LegFeaturePtr frontLeg_;/**< The front leg */
    LegFeaturePtr backLeg_; /**< The back leg */

    bool addLeg(LegFeaturePtr leg);/**< Add a leg two this tracker */

    double total_probability_;/**< Overall probability in this tracker */
    double dist_probability_;/**< Probability of this tracker based on the distance of the legs */
    double leg_association_probability_; /**< The probability that the associated legs belong to this tracker */
    double leg_time_probability_;/**< Probability considering the lifetime of both leg trackers */

    ros::Time creation_time_;/**< Time that this tracker was created */

    double maxStepWidth_; /**< Maximal Step Width */

    //// KALMAN SMOOTHING
    //BFL::StatePosVel sys_sigma_;
    //estimation::Tracker* kalmanTracker; /**< Kalman Tracker for Smoothing */

    filter::KalmanFilter* kalmanFilter_;

  public:
    PeopleTracker(LegFeaturePtr, LegFeaturePtr, ros::Time);/**< Construct a People tracker based on this two legs */

    ~PeopleTracker();

    bool isTheSame(LegFeaturePtr, LegFeaturePtr); /**< Check if this People Tracker is the same as one that would be constructed of the two given legs */

    bool isTheSame(PeopleTrackerPtr);

    LegFeaturePtr getLeg0() const;/**< Get Leg0 */

    LegFeaturePtr getLeg1() const;/**< Get Leg1 */

    LegFeaturePtr getLeftLeg() const; /**< Get left leg */

    LegFeaturePtr getRightLeg() const; /**< Get right leg */

    LegFeaturePtr getMovingLeg() const; /**< Get left leg */

    LegFeaturePtr getStandingLeg() const; /**< Get right leg */

    bool isValid() const;/**< Check if the people tracker is still valid */

    /**
     * Update everything of this tracker
     */
    void update(ros::Time);

    /**
     * Update the state of the Tracker
     * @param The current Time
     */
    void updateTrackerState(ros::Time);

    /**
     * Do a propagation of the two leg motion model
     * @param time (Time of the scan)
     */
    void propagate(ros::Time time);

    /**
     * Update the probabilities of this tracker
     */
    void updateProbabilities(ros::Time);

    /**
     * Update the history using the current position estimation
     * @param time The current time
     */
    void updateHistory(ros::Time time);

    /**
     * Get the Total Probability
     * @return  The total probability
     */
    double getTotalProbability() const{
      return this->total_probability_;
    }

    /**
     * Get the estimation of the leg with id
     * @param Id of the leg
     * @return The estimation
     */
    BFL::StatePosVel getLegEstimate(int id);

    /**
     * Get the estimation of the People Tracker, this is calculated using the estimation of both legs
     * @return The estimation
     */
    BFL::StatePosVel getEstimate();

    /**
     * Get the estimation of the Kalman filter
     * @return
     */
    BFL::StatePosVel getEstimateKalman();

    /**
     * Get the size of the people history
     * @return Size of the History
     */
    unsigned int getHistorySize();

    /**
     * Get the history of this people tracker
     * @return Vector of Shared Pointers of Stamped Points
     */
    std::vector<boost::shared_ptr<tf::Stamped<tf::Point> > >  getHistory();

    tf::Vector3 getHipVec(){
      return this->hip_vec_;
    }

    std::vector<tf::Vector3> getEstimationLines(int NumberOfLines, double angle_inkrement);

    //BFL::StatePosVel getLegEstimate(int id){
    //  return pos_vel_estimation_;
    //}

    bool isStatic(){
      return is_static_;
    }

    bool isDynamic() const{
      return !is_static_;
    }

    void removeLegs(){
      legs_.clear();
    }

    double getStepWidth() const{
      return this->stepWidth_;
    }

    double getMaxStepWidth() const{
      return this->stepWidthMax_;
    }

    double getHipWidth() const{
      return this->hipWidth_;
    }

    LegFeaturePtr getFrontLeg(){
      return this->frontLeg_;
    }

    LegFeaturePtr getBackLeg(){
      return this->backLeg_;
    }


    /// output stream
    friend std::ostream& operator<< (std::ostream& os, const PeopleTracker& s)
    {

      os << "PeopleTracker: " << s.id_[0] << " - " << s.id_[1];

      if(s.getTotalProbability() > 0.5) // TODO make this dependend on some reliability limit
        os << BOLDMAGENTA;
      os  << " p_t: " << s.getTotalProbability();

      if(s.isValid())
        os << BOLDGREEN << " [valid]" << RESET;
      else
        os << BOLDRED << " [invalid]" << RESET;

      if(s.isDynamic()){
        os << " [dyn]";
      }else{
        os << " [stat]";
      }

      // Print Parameters
      os << " | hipWidth: " << s.hipWidth_ << " | stepWidth:" << s.getStepWidth() << " | maxStepWidth:" << s.getMaxStepWidth();

      return os;

    };

};

bool isValidPeopleTracker(const PeopleTrackerPtr & o);

/**
 * List of all people trackers, checks that there are only unique assignments
 */
class PeopleTrackerList{
  private:
    boost::shared_ptr<std::vector<PeopleTrackerPtr> > list_; /**< the legs, should be maximum 2! */

  public:
    PeopleTrackerList();

    /**
     * Add a Tracker to the list
     * @param The Tracker to be added
     * @return  True on success
     */
    bool addPeopleTracker(PeopleTrackerPtr);

    /**
     * Check if there is already a tracker associated to these two legs
     * @param legA The one leg
     * @param legB The other leg
     * @return True if exists, false otherwise
     */
    bool exists(LegFeaturePtr legA, LegFeaturePtr legB);

    /**
     * Check if a tracker with the same two legs is already in the list
     * @param The tracker
     * @return True if the Tracker already exists
     */
    bool exists(PeopleTrackerPtr);

    /**
     * Get the list of People Trackers
     * @return Pointer to the list of people trackers
     */
    boost::shared_ptr<std::vector<PeopleTrackerPtr> > getList(){
      return list_;
    }

    /**
     * Remove Invalid trackers
     * @return Number of removed Trackers
     */
    int removeInvalidTrackers();

    /**
     * std::cout a list if the current trackers
     */
    void printTrackerList();

    /**
     * Call update on all trackers
     */
    void updateAllTrackers(ros::Time);

    /**
     * Update the probabilities of all trackers
     */
    void updateProbabilities(ros::Time);





};



#endif /* PEOPLE_LEG_DETECTOR_INCLUDE_LEG_DETECTOR_PEOPLE_TRACKER_H_ */
