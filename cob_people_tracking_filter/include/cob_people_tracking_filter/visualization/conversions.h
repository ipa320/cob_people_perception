/*
 * conversions.h
 *
 *  Created on: Apr 14, 2015
 *      Author: frm-ag
 */

#ifndef PEOPLE_PEOPLE_TRACKING_FILTER_INCLUDE_PEOPLE_TRACKING_FILTER_VISUALIZATION_CONVERSIONS_H_
#define PEOPLE_PEOPLE_TRACKING_FILTER_INCLUDE_PEOPLE_TRACKING_FILTER_VISUALIZATION_CONVERSIONS_H_

#include <people_tracking_filter/visualization/colormap.hpp>

bool legMeasMsgToPointcloud(sensor_msgs::PointCloud::Ptr &pointCloudMsg, people_msgs::PositionMeasurementArray::ConstPtr message){

  sensor_msgs::ChannelFloat32 rgb_channel;
  rgb_channel.name = "rgb";
  pointCloudMsg->channels.push_back(rgb_channel);

  unsigned int counter = 0;
  for(int i = 0; i < message->people.size(); i++)
  {
    people_msgs::PositionMeasurement posMeas = message->people[i];

      int r, g, b;
      getRGB(posMeas.reliability,r,g,b);

      float color_val = 0;
      int rgb = (r  << 16) | (g << 8) | b;
      color_val = *(float*) & (rgb);

      geometry_msgs::Point32 point;
      point.x = posMeas.pos.x;
      point.y = posMeas.pos.y;
      point.z = 0;

      pointCloudMsg->points.push_back(point);

      if (pointCloudMsg->channels[0].name == "rgb")
         pointCloudMsg->channels[0].values.push_back(color_val);

      counter++;
  }
  return true;
}



#endif /* PEOPLE_PEOPLE_TRACKING_FILTER_INCLUDE_PEOPLE_TRACKING_FILTER_VISUALIZATION_CONVERSIONS_H_ */
