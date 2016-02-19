/*
 * config_struct.h
 *
 *  Created on: Oct 26, 2015
 *      Author: frm-ag
 */

#ifndef PEOPLE_cob_dual_leg_tracker_INCLUDE_cob_dual_leg_tracker_CONFIG_STRUCT_H_
#define PEOPLE_cob_dual_leg_tracker_INCLUDE_cob_dual_leg_tracker_CONFIG_STRUCT_H_

struct config_struct/** < Structure for filter configuration */
{
    double fakeLegProb;
    double minFakeLegPersonProbability;

    double fakeLegRealLegDistance;
    double fakeLegRangeThres; //Distance to look for other legs around a detected leg
    double fakeLegMeasurementProbabiltyFactor;

    double minUpdateProbability;
};

#endif /* PEOPLE_cob_dual_leg_tracker_INCLUDE_cob_dual_leg_tracker_CONFIG_STRUCT_H_ */
