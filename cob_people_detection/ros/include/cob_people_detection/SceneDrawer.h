/*****************************************************************************
 *                                                                            *
 *  OpenNI 1.0 Alpha                                                          *
 *  Copyright (C) 2010 PrimeSense Ltd.                                        *
 *                                                                            *
 *  This file is part of OpenNI.                                              *
 *                                                                            *
 *  OpenNI is free software: you can redistribute it and/or modify            *
 *  it under the terms of the GNU Lesser General Public License as published  *
 *  by the Free Software Foundation, either version 3 of the License, or      *
 *  (at your option) any later version.                                       *
 *                                                                            *
 *  OpenNI is distributed in the hope that it will be useful,                 *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of            *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the              *
 *  GNU Lesser General Public License for more details.                       *
 *                                                                            *
 *  You should have received a copy of the GNU Lesser General Public License  *
 *  along with OpenNI. If not, see <http://www.gnu.org/licenses/>.            *
 *                                                                            *
 *****************************************************************************/

#ifndef XNV_POINT_DRAWER_H_
#define XNV_POINT_DRAWER_H_

#include <openni/XnCppWrapper.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/CvBridge.h>

void PublishPeopleImage(const xn::DepthMetaData& dmd, const xn::SceneMetaData& smd, image_transport::Publisher& pub);

#endif
