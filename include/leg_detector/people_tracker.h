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
    std::vector<LegFeature*> legs_; /**< the legs, should be maximum 2! */

    bool addLeg(LegFeature* leg);/**< Add a leg two this tracker */

  public:
    PeopleTracker(LegFeature*, LegFeature*);/**< Construct a People tracker based on this two legs */

    bool isTheSame(PeopleTrackerPtr);

    LegFeature* getLeg0();/**< Get Leg0 */

    LegFeature* getLeg1();/**< Get Leg1 */

};



/**
 * List of all people trackers, checks that there are only unique assignments
 */
class PeopleTrackerList{
  private:
    std::vector<PeopleTrackerPtr> list_; /**< the legs, should be maximum 2! */

  public:
    bool addPeopleTracker(PeopleTrackerPtr);

    bool exists(PeopleTrackerPtr);


};



#endif /* PEOPLE_LEG_DETECTOR_INCLUDE_LEG_DETECTOR_PEOPLE_TRACKER_H_ */
