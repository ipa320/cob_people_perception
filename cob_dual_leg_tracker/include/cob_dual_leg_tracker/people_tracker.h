#ifndef PEOPLE_LEG_DETECTOR_INCLUDE_LEG_DETECTOR_PEOPLE_TRACKER_H_
#define PEOPLE_LEG_DETECTOR_INCLUDE_LEG_DETECTOR_PEOPLE_TRACKER_H_

// ROS includes
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

// System includes
#include <vector>

// Boost includes
#include <boost/shared_ptr.hpp>
#include <boost/array.hpp>

// Own includes
#include <cob_dual_leg_tracker/leg_feature.h>
#include <cob_dual_leg_tracker/config_struct.h>
#include <cob_dual_leg_tracker/kalman/kalman_filter.h>

// Debug Flags
#define DEBUG_PEOPLE_TRACKER 0
#define DEBUG_PEOPLETRACKERLIST 0

class LegFeature; //Forward declaration
typedef boost::shared_ptr<LegFeature> LegFeaturePtr;

class PeopleTracker; //Forward declaration
typedef boost::shared_ptr<PeopleTracker> PeopleTrackerPtr;

class PeopleTrackerList; //Forward

static unsigned int historySize = 50;

/**
 * A struct defining a entry of the people history
 */
struct people_history_entry{

    boost::shared_ptr< tf::Stamped<tf::Point> > position_;

    double probability_;
};

/**
 * High level People Tracker consisting of two low level leg tracks
 */
class PeopleTracker{

  private:
    ros::Time propagation_time_; /**< Time the propagation is towards */

    tf::TransformBroadcaster br; /**< A transform broadcaster */

    bool is_static_;  /**< Flag if this person is static (never did move) */

    std::vector<LegFeaturePtr> legs_; /**< the legs, should be maximum 2! */

    LegFeaturePtr leftLeg_; /**< The left leg */
    LegFeaturePtr rightLeg_;/**< The right leg */
    LegFeaturePtr frontLeg_;/**< The front leg */
    LegFeaturePtr backLeg_; /**< The back leg */

    double total_probability_;/**< Overall probability in this tracker */
    double dist_probability_;/**< Probability of this tracker based on the distance of the legs */
    double leg_association_probability_; /**< The probability that the associated legs belong to this tracker */
    double leg_time_probability_;/**< Probability considering the lifetime of both leg trackers */

    ros::Time creation_time_;/**< Time that this tracker was created */

    //double maxStepWidth_; /**< Maximal Step Width */

    filter::KalmanFilter* kalmanFilter_; /**< Kalman Filter associated to the people Tracker to provide smoothing */

    std::vector<tf::Vector3> nextDesiredPosition; /**< Vector containing the calculated desired position of this people tracker */

    std::vector<tf::Vector3> nextDesiredVelocity; /**< Vector containing the calculated desired Velocities of this people tracker */

    tf::Vector3 currentGoal_; /**< The current goal position */

    bool hasGoal_; /**< True if a goal is set */

    config_struct current_config_; /**< The current set config file */

    BFL::StatePosVel leg0Prediction_;
    BFL::StatePosVel leg1Prediction_;

    double hipWidth_;
    double stepWidth_;
    double stepWidthMax_;

    std::list< people_history_entry > history_; /**< The position history of the people tracker */

    tf::Vector3 hipPos0_, hipPos1_; /**< Vector of the endpoints of vector */
    tf::Vector3 hipPosLeft_, hipPosRight_; /**< Vector of the endpoints of vector */

    boost::array<int, 2> id_;/**< Numerical id of this people tracker */

    BFL::StatePosVel pos_vel_estimation_; /**< The currently estimated pos_vel_ of this people */

    tf::Vector3 hip_vec_; /**< Vector orthogonal to the velocity vector, representing the hip direction pos_vel_estimation.vel_.dot(hip_vec_) = 0 should hold every time*/


  public:
    PeopleTracker(LegFeaturePtr, LegFeaturePtr, ros::Time);/**< Construct a People tracker based on this two legs */

    ~PeopleTracker(); /**< Destructor */

    bool isTheSame(LegFeaturePtr, LegFeaturePtr); /**< Check if this People Tracker is the same as one that would be constructed of the two given legs */

    bool isTheSame(PeopleTrackerPtr); /**< Check if this People Tracker is the same as the provided one */

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
     * Update the configuration/parameters of the filter/trackers
     * @param filter_config
     */
    void configure(config_struct filter_config);

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
    BFL::StatePosVel getEstimate() const;

    /**
     * Get the estimation of the Kalman filter
     * @return
     */
    BFL::StatePosVel getEstimateKalman();

    /**
     * Get the current propagation Time
     * @return
     */
    ros::Time getPropagationTime() const{ return this->propagation_time_; }

    /**
     * Get the size of the people history
     * @return Size of the History
     */
    unsigned int getHistorySize();

    /**
     * Get the history of this people tracker
     * @return Vector of Shared Pointers of Stamped Points
     */
    const std::list< people_history_entry >&  getHistory() const;

    /**
     * Return the hip Vector
     * @return
     */
    tf::Vector3 getHipVec() const{
      return this->hip_vec_;
    }

    bool isStatic() const{
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

    double getStepWidthMax() const{
      return this->stepWidthMax_;
    }

    double getHipWidth() const{
      return this->hipWidth_;
    }

    LegFeaturePtr getFrontLeg() const{
      return this->frontLeg_;
    }

    LegFeaturePtr getBackLeg() const{
      return this->backLeg_;
    }

    BFL::StatePosVel getLeg0Prediction() const{
      return this->leg0Prediction_;
    }

    BFL::StatePosVel getLeg1Prediction() const{
      return this->leg1Prediction_;
    }

    std::string getName() const{
      std::stringstream name;

      name << "ppl" << this->id_[0] << "_" << this->id_[1];

      return name.str();
    }

    tf::Vector3 getHipPosRight() const{
      return this->hipPosRight_;
    }

    tf::Vector3 getHipPosLeft() const{
      return this->hipPosLeft_;
    }

    boost::array<int, 2> getId() const{
      return this->id_;
    }

    /**
     * Broadcast this people trackers transformation
     * @param time
     */
    void broadCastTf(ros::Time time);

    /**
     * Calculate the next desired velocity (experimental)
     * @param list            List of all trackers
     * @param predictionStep  The current prediction Step
     * @param timeInterval    The time interval between the predictions
     */
    void calculateNextDesiredVelocity(boost::shared_ptr<std::vector<PeopleTrackerPtr> > list, size_t predictionStep, double timeInterval);

    /**
     * Return the calculated next desired velocity
     * @return
     */
    std::vector<tf::Vector3> getNextDesiredVelocities() const{
      return this->nextDesiredVelocity;
    }

    /**
     * Return the desired velocity for a given prediction step
     * @param predictionStep
     * @return
     */
    BFL::StatePosVel getNextDesiredPosVel(size_t predictionStep) const;

    /**
     * Set a goal for this tracker (experimental: currently done manually - further development needed)
     * @param goal
     */
    void setGoal(tf::Vector3& goal){
      currentGoal_ = goal;
    }

    /**
     * Return the current goal
     * @return
     */
    tf::Vector3 getGoal() const{
      return this->currentGoal_;
    }

    /**
     * Get the number of predictions
     * @return Number of Predictions
     */
    size_t getNumberOfPredictions() const{
      ROS_ASSERT(this->nextDesiredPosition.size() == this->nextDesiredVelocity.size());

      return this->nextDesiredPosition.size();
    }



    /**
     * Overload the operator the provide easy cout functionality
     */
    friend std::ostream& operator<< (std::ostream& os, const PeopleTracker& s)
    {

      os << "PeopleTracker: " << s.id_[0] << " - " << s.id_[1];

      os << std::fixed << std::setprecision(3);

      if(s.getTotalProbability() > 0.5) // TODO make this dependend on some reliability limit
        os << BOLDMAGENTA;
      os  << " p_t: " << s.getTotalProbability();

      if(s.isValid())
        os << BOLDGREEN << " [valid]  " << RESET;
      else
        os << BOLDRED   << " [invalid]" << RESET;

      if(s.isDynamic()){
        os << " [dyn] ";
      }else{
        os << " [stat]";
      }

      // Print Parameters
      os << " | hipW: " << s.getHipWidth();
      os << " | stepW:" << s.getStepWidth();
      os << " | maxStepW:" << s.getStepWidthMax();
      os << " | vAbs:" << s.getEstimate().vel_.length();
      return os;

    };

    /**
     * Overloaded operator to compare two PeopleTracker
     * @param p0
     * @param p1
     * @return
     */
    friend bool operator== (PeopleTracker &p0, PeopleTracker &p1);

  private:

    /**
     * Add a leg two this tracker
     */
    bool addLeg(LegFeaturePtr leg);

};



bool isValidPeopleTracker(const PeopleTrackerPtr & o);

/**
 * List of all people trackers, checks that there are only unique assignments
 */
class PeopleTrackerList{
  private:
    boost::shared_ptr<std::vector<PeopleTrackerPtr> > list_; /**< the legs, should be maximum 2! */

  public:
    /**
     * Constructor
     */
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
    bool exists(LegFeaturePtr legA, LegFeaturePtr legB) const;

    /**
     * Check if a tracker with the same two legs is already in the list
     * @param The tracker
     * @return True if the Tracker already exists
     */
    bool exists(PeopleTrackerPtr) const;

    /**
     * Get the list of People Trackers
     * @return Pointer to the list of people trackers
     */
    boost::shared_ptr<std::vector<PeopleTrackerPtr> > getList() const{
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
    void printTrackerList() const;

    /**
     * Call update on all trackers
     */
    void updateAllTrackers(ros::Time);

    /**
     * Update the probabilities of all trackers
     */
    void updateProbabilities(ros::Time);

    /**
     * Calculate the next desired velocity (experimental!)
     * @param timeInterval
     * @param predictionSteps
     */
    void calculateTheNextDesiredVelocities(double timeInterval, size_t predictionSteps);

};



#endif /* PEOPLE_LEG_DETECTOR_INCLUDE_LEG_DETECTOR_PEOPLE_TRACKER_H_ */
