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

void getCycledColor(int value, int &r, int &g, int &b)
{
  int n = 3;

  int r_set[3] = { 0, 125, 255};
  int g_set[3] = { 0, 125, 255};
  int b_set[3] = { 0, 125, 255};

  int r_ind = ((value/1) % n) ;
  int g_ind = ((value/3) % n) ;
  int b_ind = ((value/9) % n) ;

  std::cout << r_ind << " " << g_ind << " " << b_ind << std::endl;

  r = (int) 255 * (1.0 / (n-1)) * r_ind;
  g = (int) 255 * (1.0 / (n-1)) * g_ind;
  b = (int) 255 * (1.0 / (n-1)) * b_ind;

}

#endif /* PEOPLE_DUAL_PEOPLE_LEG_TRACKER_INCLUDE_DUAL_PEOPLE_LEG_TRACKER_VISUALIZATION_COLOR_FUNCTIONS_H_ */
