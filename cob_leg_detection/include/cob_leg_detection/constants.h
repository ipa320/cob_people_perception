/*
 * constants.h
 *
 *  Created on: Apr 17, 2015
 *      Author: frm-ag
 */

#ifndef PEOPLE_LEG_DETECTOR_INCLUDE_LEG_DETECTOR_CONSTANTS_H_
#define PEOPLE_LEG_DETECTOR_INCLUDE_LEG_DETECTOR_CONSTANTS_H_

static std::string fixed_frame              = "odom_combined";  // The fixed frame in which ? //TODO find out

static double kal_p = 4, kal_q = .002, kal_r = 10;
static bool use_filter = false;

#endif /* PEOPLE_LEG_DETECTOR_INCLUDE_LEG_DETECTOR_CONSTANTS_H_ */
