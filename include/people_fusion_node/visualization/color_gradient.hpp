#ifndef COLOR_GRADIENT_HPP_
#define COLOR_GRADIENT_HPP_

/**
 * Maps a given value [0..1] to r,g,b within range [0..255]
 * @param value The given value
 * @param red
 * @param green
 * @param blue
 */
void redGreenGradient(double value, int &red, int &green, int &blue)
{

  ROS_WARN_COND(value < 0 || value > 1, "redGreenGradient expects value in range [0..1]!");

  // Ensure right range
  double v = std::max(std::min(value,1.0),0.0);

  int aR = 255;   int aG = 0; int aB=0;  // RGB for our 1st color (blue in this case).
  int bR = 0; int bG = 255; int bB=0;    // RGB for our 2nd color (red in this case).

  red   = (double)(bR - aR) * value + aR;      // Evaluated as -255*value + 255.
  green = (double)(bG - aG) * value + aG;      // Evaluates as 0.
  blue  = (double)(bB - aB) * value + aB;      // Evaluates as 255*value + 0.
}

#endif /* COLOR_GRADIENT_HPP_ */
