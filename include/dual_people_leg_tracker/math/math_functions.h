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

double distance(tf::Vector3 point, tf::Vector3 pointOnLine, tf::Vector3 lineVector){

  // Normalize
  // lineVector = lineVector.normalize();

  // Orthogonal vector
  tf::Vector3 v;
  v[0] = lineVector[1];
  v[1] = -lineVector[0];
  v[2] = 0.0;

  v = v.normalize();

  //std::cout << "LineVector: " << lineVector[0] << "  " << lineVector[1] << "  " << lineVector[2] << std::endl;
  //std::cout << "V         : " << v[0] << "  " << v[1] << "  " << v[2] << std::endl;

  // Check that v is really orthogonal
  ROS_ASSERT(lineVector.dot(v) < 0.00001);

  //std::cout << "V: " << lineVector.dot(v) << std::endl;

  double distance = (lineVector.cross(pointOnLine-point)).length()/lineVector.length();

  // tf::Vector3 crossPoint;
  // crossPoint = point + distance * v;
  // assert(false);

  return distance;
}

#endif /* PEOPLE_DUAL_PEOPLE_LEG_TRACKER_INCLUDE_DUAL_PEOPLE_LEG_TRACKER_MATH_MATH_FUNCTIONS_H_ */
