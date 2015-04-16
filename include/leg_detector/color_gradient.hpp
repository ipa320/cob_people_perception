/*
 * color_gradient.hpp
 *
 *  Created on: Mar 16, 2015
 *      Author: frm-ag
 */

#ifndef IPA_NAVIGATION_SANDBOX_IPA_HUMAN_MOTION_TRACKER_FREIBURG_ROS_MISC_COLOR_GRADIENT_HPP_
#define IPA_NAVIGATION_SANDBOX_IPA_HUMAN_MOTION_TRACKER_FREIBURG_ROS_MISC_COLOR_GRADIENT_HPP_

// Source: http://www.andrewnoske.com/wiki/Code_-_heatmaps_and_color_gradients
void getValueBetweenTwoFixedColors(float value, int &red, int &green, int &blue)
{
  int aR = 0;   int aG = 0; int aB=255;  // RGB for our 1st color (blue in this case).
  int bR = 0; int bG = 255; int bB=0;    // RGB for our 2nd color (red in this case).

  red   = (float)(bR - aR) * value + aR;      // Evaluated as -255*value + 255.
  green = (float)(bG - aG) * value + aG;      // Evaluates as 0.
  blue  = (float)(bB - aB) * value + aB;      // Evaluates as 255*value + 0.
}



#endif /* IPA_NAVIGATION_SANDBOX_IPA_HUMAN_MOTION_TRACKER_FREIBURG_ROS_MISC_COLOR_GRADIENT_HPP_ */
