/*
 * people_tracker.h
 *
 *  Created on: Apr 16, 2015
 *      Author: frm-ag
 */

#ifndef PEOPLE_LEG_DETECTOR_INCLUDE_LEG_DETECTOR_PEOPLE_TRACKER_H_
#define PEOPLE_LEG_DETECTOR_INCLUDE_LEG_DETECTOR_PEOPLE_TRACKER_H_

/**
 * High level People Tracker consisting of two low level leg tracks
 */
class PeopleTracker{
  private:
    std::vector<SavedFeature*> legs_; /**< the legs, should be maximum 2! */

  public:
    bool addLeg(SavedFeature* leg);
};

typedef boost::shared_ptr<PeopleTracker> PeopleTrackerPtr;

#endif /* PEOPLE_LEG_DETECTOR_INCLUDE_LEG_DETECTOR_PEOPLE_TRACKER_H_ */
