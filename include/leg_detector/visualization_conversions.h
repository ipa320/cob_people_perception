/*
 * visualization_conversions.h
 *
 *  Created on: Mar 19, 2015
 *      Author: frm-ag
 */

#ifndef PEOPLE_LEG_DETECTOR_INCLUDE_LEG_DETECTOR_VISUALIZATION_CONVERSIONS_H_
#define PEOPLE_LEG_DETECTOR_INCLUDE_LEG_DETECTOR_VISUALIZATION_CONVERSIONS_H_

//#include <leg_detector/leg_detector.h>
#include <leg_detector/color_gradient.hpp>
#include <leg_detector/saved_feature.h>

namespace visualization{

bool savedFeatureToSphereLegMarkerMsg(SavedFeature* sf, visualization_msgs::Marker::Ptr markerMsg, std::string& fixed_frame, unsigned int id = 0){

    markerMsg->header.stamp = sf->time_;
    markerMsg->header.frame_id = fixed_frame;
    markerMsg->ns = "LEGS";
    markerMsg->id = id;
    markerMsg->type = markerMsg->SPHERE;
    markerMsg->pose.position.x = sf->position_[0];
    markerMsg->pose.position.y = sf->position_[1];
    markerMsg->pose.position.z = sf->position_[2];

    markerMsg->scale.x = .1;
    markerMsg->scale.y = .1;
    markerMsg->scale.z = .1;
    markerMsg->color.a = 0.7;
    markerMsg->lifetime = ros::Duration(0.5);
    if (sf->object_id != "")
    {
        markerMsg->color.r = 1;
    }
    else
    {
        markerMsg->color.b = (float) sf->getReliability();
    }

    return true;
}

bool clusterToTextMarkerMsg(SavedFeature* sf, visualization_msgs::Marker::Ptr markerMsg, std::string& fixed_frame, unsigned int id = 0){

    markerMsg->header.stamp = sf->time_;
    markerMsg->header.frame_id = fixed_frame;
    markerMsg->ns = "LEGS_LABEL";
    markerMsg->id = id;
    markerMsg->type = markerMsg->TEXT_VIEW_FACING;
    markerMsg->pose.position.x = sf->position_[0];
    markerMsg->pose.position.y = sf->position_[1];
    markerMsg->pose.position.z = sf->position_[2];
    markerMsg->scale.z = .2;
    markerMsg->color.a = 1;
    markerMsg->lifetime = ros::Duration(0.5);

    // Add text
    char buf[100];

    if (sf->object_id != "")
    {
        markerMsg->color.r = 1;
        sprintf(buf, "%s\n%s-%.2f", sf->object_id.c_str(), sf->id_.c_str(), sf->getReliability());
    }
    else
    {
        markerMsg->color.b = sf->getReliability();
        sprintf(buf, "#%d-%s-%.2f", id, sf->id_.c_str(), sf->getReliability());
    }
    markerMsg->text = buf;

    return true;
}

bool savedFeatureToPeopleMarkerMsg(SavedFeature* leg1, SavedFeature* leg2, visualization_msgs::Marker::Ptr pPeopleSphereMsg, visualization_msgs::Marker::Ptr legLineMsg, std::string& fixed_frame, tf::Vector3& peoplePos, unsigned int id = 0){
  pPeopleSphereMsg->header.stamp = leg1->time_;
  pPeopleSphereMsg->header.frame_id = fixed_frame;
  pPeopleSphereMsg->ns = "PEOPLE";
  pPeopleSphereMsg->id = id;
  pPeopleSphereMsg->type = pPeopleSphereMsg->SPHERE;
  pPeopleSphereMsg->pose.position.x = peoplePos[0];
  pPeopleSphereMsg->pose.position.y = peoplePos[1];
  pPeopleSphereMsg->pose.position.z = peoplePos[2];
  pPeopleSphereMsg->scale.x = .2;
  pPeopleSphereMsg->scale.y = .2;
  pPeopleSphereMsg->scale.z = .2;
  pPeopleSphereMsg->color.a = 0.7;

  int r, g, b;
  getValueBetweenTwoFixedColors(leg1->getReliability(),r,g,b);

  pPeopleSphereMsg->color.r = r / 255.0;
  pPeopleSphereMsg->color.g = g / 255.0;
  pPeopleSphereMsg->color.b = b / 255.0;

  pPeopleSphereMsg->lifetime = ros::Duration(0.5);


  return true;
}

}


#endif /* PEOPLE_LEG_DETECTOR_INCLUDE_LEG_DETECTOR_VISUALIZATION_CONVERSIONS_H_ */
