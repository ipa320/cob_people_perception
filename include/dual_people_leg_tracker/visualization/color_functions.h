#ifndef PEOPLE_DUAL_PEOPLE_LEG_TRACKER_INCLUDE_DUAL_PEOPLE_LEG_TRACKER_VISUALIZATION_COLOR_FUNCTIONS_H_
#define PEOPLE_DUAL_PEOPLE_LEG_TRACKER_INCLUDE_DUAL_PEOPLE_LEG_TRACKER_VISUALIZATION_COLOR_FUNCTIONS_H_

#include <math.h>

void getColor(int value, int &red, int &green, int &blue)
{
  int N = 20;

  value = value % N;

  double h;
  double r = 0.9999;
  h = 1.0/N;

  red   = (r * cos(2*M_PI*value/N)*0.5 + 0.5)*255;
  green = (r * sin(2*M_PI*value/N)*0.5 + 0.5)*255;
  blue  = h*value * 255;

}

#endif /* PEOPLE_DUAL_PEOPLE_LEG_TRACKER_INCLUDE_DUAL_PEOPLE_LEG_TRACKER_VISUALIZATION_COLOR_FUNCTIONS_H_ */
