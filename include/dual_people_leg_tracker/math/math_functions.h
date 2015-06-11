/*
 * math_functions.h
 *
 *  Created on: Apr 28, 2015
 *      Author: frm-ag
 */

#ifndef PEOPLE_DUAL_PEOPLE_LEG_TRACKER_INCLUDE_DUAL_PEOPLE_LEG_TRACKER_MATH_MATH_FUNCTIONS_H_
#define PEOPLE_DUAL_PEOPLE_LEG_TRACKER_INCLUDE_DUAL_PEOPLE_LEG_TRACKER_MATH_MATH_FUNCTIONS_H_

#include <math.h>

/**
 * @brief Sigmoid
 * @param x The point to evaluate the sigmoid at
 * @param a Slope factor
 * @param b Shift
 * @return
 */
double sigmoid(double x, double a=1.0, double b=0.0)
{
  return 1.0 / (1.0 + exp(-a*(x-b)));
}

/**
 * Distance between a point and the line define by a point and a vector
 * @param point The point the distance is measured towards
 * @param pointOnLine An arbitrary point on the line
 * @param lineVector Vector of the line
 * @return
 */
double distance(tf::Vector3 point, tf::Vector3 pointOnLine, tf::Vector3 lineVector){

  // Normalize
  lineVector = lineVector.normalize();

  tf::Vector3 ap = pointOnLine - point;

  double distance = (ap-(ap.dot(lineVector))*lineVector).length();

  return distance;
}

#endif /* PEOPLE_DUAL_PEOPLE_LEG_TRACKER_INCLUDE_DUAL_PEOPLE_LEG_TRACKER_MATH_MATH_FUNCTIONS_H_ */
