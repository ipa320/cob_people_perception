/*********************************************************************
* Software License Agreement (BSD License)
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/



#ifndef PEOPLE_PEOPLE_TRACKING_FILTER_INCLUDE_PEOPLE_TRACKING_FILTER_OCCLUSION_MODEL_H_
#define PEOPLE_PEOPLE_TRACKING_FILTER_INCLUDE_PEOPLE_TRACKING_FILTER_OCCLUSION_MODEL_H_

// ROS includes
#include <ros/console.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

// ROS Messages
#include <sensor_msgs/LaserScan.h>

#define DEBUG_OCCLUSION_MODEL 0

class OcclusionModel{
  public:
    /**
     * The currently used laserscan
     */
    sensor_msgs::LaserScan scan_;

    /**
     * The used transformation listener
     */
    tf::TransformListener& tfl_;

    /**
     * Constructor
     * @param tf
     */
    OcclusionModel(tf::TransformListener& tfl);

    /**
     * Set the scan to use
     * @param scan
     */
    void updateScan(const sensor_msgs::LaserScan& scan);

    double getOcclusionProbability(tf::Vector3 loc);

    double getOcclusionProbability(tf::Stamped<tf::Point>);
};

typedef boost::shared_ptr<OcclusionModel> OcclusionModelPtr;

#endif /* PEOPLE_PEOPLE_TRACKING_FILTER_INCLUDE_PEOPLE_TRACKING_FILTER_OCCLUSION_MODEL_H_ */
