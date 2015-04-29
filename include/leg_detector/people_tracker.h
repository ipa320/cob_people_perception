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

#include <boost/shared_ptr.hpp>
#include <leg_detector/leg_feature.h>

#define DEBUG_PEOPLE_TRACKER 0

//class LegFeature; //Forward declaration
//typedef boost::shared_ptr<LegFeature> LegFeaturePtr;

class PeopleTracker; //Forward declaration
typedef boost::shared_ptr<PeopleTracker> PeopleTrackerPtr;

/**
 * High level People Tracker consisting of two low level leg tracks
 */
class PeopleTracker{
  private:
    std::vector<LegFeaturePtr> legs_; /**< the legs, should be maximum 2! */

    bool addLeg(LegFeaturePtr leg);/**< Add a leg two this tracker */

    double total_probability_;/**< Overall probability in this tracker */
    double dist_probability_;/**< Probability of this tracker based on the distance of the legs */
    double leg_time_probability_;/**< Probability considering the lifetime of both leg trackers */

    ros::Time creation_time_;/**< Time that this tracker was created */

  public:
    PeopleTracker(LegFeaturePtr, LegFeaturePtr, ros::Time);/**< Construct a People tracker based on this two legs */

    bool isTheSame(LegFeaturePtr, LegFeaturePtr); /**< Check if this People Tracker is the same as one that would be constructed of the two given legs */

    bool isTheSame(PeopleTrackerPtr);

    LegFeaturePtr getLeg0();/**< Get Leg0 */

    LegFeaturePtr getLeg1();/**< Get Leg1 */

    bool isValid();/**< Check if the people tracker is still valid */

    /**
     * Update everything of this tracker
     */
    void update(ros::Time);

    /**
     * Update the probabilities of this tracker
     */
    void updateProbabilities(ros::Time);

    /**
     * Get the Total Probability
     * @return  The total probability
     */
    double getTotalProbability(){
      return this->total_probability_;
    }

};



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
