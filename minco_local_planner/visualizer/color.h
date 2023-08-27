/**
 * @Author: Yunkai Xia
 * @Date:   2023-01-29 15:53:57
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-08-27 22:12:51
 */
#include <stdint.h>

#ifndef __COLOR_H__
#define __COLOR_H__
#include <std_msgs/ColorRGBA.h>

#include <vector>

namespace visualizer {
using std_msgs::ColorRGBA;

static ColorRGBA newColorRGBADouble(double red, double green, double blue,
                                    double alpha = 1.0) {
  std_msgs::ColorRGBA color;
  color.r = red;
  color.g = green;
  color.b = blue;
  color.a = alpha;
  return color;
}

static ColorRGBA newColorRGBA(uint8_t red, uint8_t green, uint8_t blue,
                              double alpha = 1.0) {
  std_msgs::ColorRGBA color;
  color.r = red / 255.0;
  color.g = green / 255.0;
  color.b = blue / 255.0;
  color.a = alpha;
  return newColorRGBADouble(red / 255.0, green / 255.0, blue / 255.0, alpha);
}

// pre-defined color
const ColorRGBA WHITE = newColorRGBA(255, 255, 255);
const ColorRGBA BLACK = newColorRGBA(0, 0, 0);
const ColorRGBA RED = newColorRGBA(255, 0, 0);
const ColorRGBA GREEN = newColorRGBA(0, 255, 0);
const ColorRGBA BLUE = newColorRGBA(0, 0, 255);
const ColorRGBA YELLOW = newColorRGBA(255, 255, 0);
const ColorRGBA CYAN = newColorRGBA(0, 255, 255);
const ColorRGBA MAGENTA = newColorRGBA(255, 0, 255);
const ColorRGBA GRAY = newColorRGBA(128, 128, 128);
const ColorRGBA PURPLE = newColorRGBA(128, 0, 128);
const ColorRGBA PINK = newColorRGBA(255, 192, 203);
const ColorRGBA LIGHT_BLUE = newColorRGBA(173, 216, 230);
const ColorRGBA LIME_GREEN = newColorRGBA(50, 205, 50);
const ColorRGBA SLATE_GRAY = newColorRGBA(112, 128, 144);

}  // namespace visualizer

#endif /* __COLOR_H__ */
