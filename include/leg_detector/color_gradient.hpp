/*
 * color_gradient.hpp
 *
 *  Created on: Mar 16, 2015
 *      Author: frm-ag
 */

#ifndef COLOR_GRADIENT_HPP_
#define COLOR_GRADIENT_HPP_

void redGreenGradient(double value, int &red, int &green, int &blue)
{
  int aR = 255;   int aG = 0; int aB=0;  // RGB for our 1st color (blue in this case).
  int bR = 0; int bG = 255; int bB=0;    // RGB for our 2nd color (red in this case).

  red   = (double)(bR - aR) * value + aR;      // Evaluated as -255*value + 255.
  green = (double)(bG - aG) * value + aG;      // Evaluates as 0.
  blue  = (double)(bB - aB) * value + aB;      // Evaluates as 255*value + 0.
}

#endif /* COLOR_GRADIENT_HPP_ */
