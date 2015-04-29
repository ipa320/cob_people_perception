/*
 * math_functions.h
 *
 *  Created on: Apr 28, 2015
 *      Author: frm-ag
 */

#ifndef PEOPLE_DUAL_PEOPLE_LEG_TRACKER_INCLUDE_DUAL_PEOPLE_LEG_TRACKER_MATH_MATH_FUNCTIONS_H_
#define PEOPLE_DUAL_PEOPLE_LEG_TRACKER_INCLUDE_DUAL_PEOPLE_LEG_TRACKER_MATH_MATH_FUNCTIONS_H_

#include <math.h>

double sigmoid(double x, double a=1.0, double b=0.0)
{
  return 1.0 / (1.0 + exp(-a*(x-b)));
}

#endif /* PEOPLE_DUAL_PEOPLE_LEG_TRACKER_INCLUDE_DUAL_PEOPLE_LEG_TRACKER_MATH_MATH_FUNCTIONS_H_ */
