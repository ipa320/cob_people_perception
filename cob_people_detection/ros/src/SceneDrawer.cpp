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

//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include "cob_people_detection/SceneDrawer.h"

extern xn::UserGenerator g_UserGenerator;
extern xn::DepthGenerator g_DepthGenerator;

#define MAX_DEPTH 10000
float g_pDepthHist[MAX_DEPTH];
unsigned int getClosestPowerOfTwo(unsigned int n)
{
	unsigned int m = 2;
	while (m < n)
		m <<= 1;

	return m;
}

XnFloat Colors[][3] =
{
{ 0, 1, 1 },
{ 0, 0, 1 },
{ 0, 1, 0 },
{ 1, 1, 0 },
{ 1, 0, 0 },
{ 1, .5, 0 },
{ .5, 1, 0 },
{ 0, .5, 1 },
{ .5, 0, 1 },
{ 1, 1, .5 },
{ 1, 1, 1 } };
XnUInt32 nColors = 10;

void DrawLimb(XnUserID player, XnSkeletonJoint eJoint1, XnSkeletonJoint eJoint2)
{
	if (!g_UserGenerator.GetSkeletonCap().IsTracking(player))
	{
		printf("not tracked!\n");
		return;
	}

	XnSkeletonJointPosition joint1, joint2;
	g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(player, eJoint1, joint1);
	g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(player, eJoint2, joint2);

	if (joint1.fConfidence < 0.5 || joint2.fConfidence < 0.5)
	{
		return;
	}

	XnPoint3D pt[2];
	pt[0] = joint1.position;
	pt[1] = joint2.position;

	g_DepthGenerator.ConvertRealWorldToProjective(2, pt, pt);
}

void PublishPeopleImage(const xn::DepthMetaData& dmd, const xn::SceneMetaData& smd, image_transport::Publisher& pub)
{
	const XnDepthPixel* pDepth = dmd.Data();
	const XnLabel* pLabels = smd.Data();
	unsigned int nValue = 0;
	XnUInt16 g_nXRes = dmd.XRes();
	XnUInt16 g_nYRes = dmd.YRes();
	cv::Mat peopleSegmentation(g_nYRes, g_nXRes, CV_8UC3);

	// Prepare the texture map
	for (unsigned int nY = 0; nY < g_nYRes; nY++)
	{
		uchar* pDestImage = peopleSegmentation.ptr(nY);
		for (unsigned int nX = 0; nX < g_nXRes; nX++)
		{

			pDestImage[0] = 0;
			pDestImage[1] = 0;
			pDestImage[2] = 0;
			if (*pLabels != 0)
			{
				nValue = *pDepth;
				XnLabel label = *pLabels;
				XnUInt32 nColorID = label % nColors;

				if (nValue != 0)
				{
					pDestImage[0] = 255 * Colors[nColorID][0];
					pDestImage[1] = 255 * Colors[nColorID][1];
					pDestImage[2] = 255 * Colors[nColorID][2];
				}
			}

			pDepth++;
			pLabels++;
			pDestImage += 3;
		}
	}

	// todo: stop and start with respect to odometry: segmentation works best if robot is standing still

	// publish
	try
	{
		IplImage img = (IplImage)peopleSegmentation;
		sensor_msgs::ImagePtr msg = (sensor_msgs::CvBridge::cvToImgMsg(&img, "bgr8"));
		msg->header.stamp = ros::Time::now();
		pub.publish(msg);
	} catch (sensor_msgs::CvBridgeException error)
	{
		ROS_ERROR("[openni_tracker] Could not convert IplImage to ROS message");
	}

	// 	cv_bridge::CvImage bridgeImage;		did not work
	// 	bridgeImage.image = peopleSegmentation.clone();
	// 	sensor_msgs::ImagePtr msg = bridgeImage.toImageMsg();
	// 	pub.publish(msg);


	// display for checking the output
	// 	cv::namedWindow("Test");
	// 	imshow("Test", peopleSegmentation);
	// 	uchar key = cv::waitKey(10);
	// 	if (key == 'r')
	// 	{
	// 	  g_UserGenerator.StopGenerating();
	// 	  std::cout << "stop\n";
	// 	}
	// 	if (key == 'c')
	// 	{
	// 	  g_UserGenerator.StartGenerating();
	// 	  std::cout << "start\n";
	// 	}
}
