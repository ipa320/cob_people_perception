/*
 * people_tracker.h
 *
 *  Created on: Apr 16, 2015
 *      Author: frm-ag
 */

#ifndef PEOPLE_LEG_DETECTOR_INCLUDE_LEG_DETECTOR_PEOPLE_TRACKER_H_
#define PEOPLE_LEG_DETECTOR_INCLUDE_LEG_DETECTOR_PEOPLE_TRACKER_H_

// Own includes
#include <leg_detector/leg_feature.h>

// System includes
#include <vector>

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

  public:
    PeopleTracker(LegFeaturePtr, LegFeaturePtr);/**< Construct a People tracker based on this two legs */

    bool isTheSame(PeopleTrackerPtr);

    LegFeaturePtr getLeg0();/**< Get Leg0 */

    LegFeaturePtr getLeg1();/**< Get Leg1 */

    bool isValid();/**< Check if the people tracker is still valid */

    void updateProbabilities();

};



/**
 * List of all people trackers, checks that there are only unique assignments
 */
class PeopleTrackerList{
  private:
    boost::shared_ptr<std::vector<PeopleTrackerPtr> > list_; /**< the legs, should be maximum 2! */

  public:
    PeopleTrackerList();

    bool addPeopleTracker(PeopleTrackerPtr);

    bool exists(PeopleTrackerPtr);

    boost::shared_ptr<std::vector<PeopleTrackerPtr> > getList(){
      return list_;
    }

    int removeInvalidTrackers();

    void printTrackerList();




};



#endif /* PEOPLE_LEG_DETECTOR_INCLUDE_LEG_DETECTOR_PEOPLE_TRACKER_H_ */
