/*
 * color_gradient.hpp
 *
 *  Created on: Mar 16, 2015
 *      Author: frm-ag
 */

#ifndef _COLORMAP_1D_HPP_
#define _COLORMAP_1D_HPP_

void getRGB(float value, int &r, int &g, int &b)
{
  int aR = 0; int aG = 0; int aB=255;
  int bR = 0; int bG = 255; int bB=0;

  r = (float)(bR - aR) * value + aR;
  g = (float)(bG - aG) * value + aG;
  b = (float)(bB - aB) * value + aB;
}



#endif /* _COLORMAP_1D_HPP_ */
