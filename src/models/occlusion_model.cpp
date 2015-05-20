#include <dual_people_leg_tracker/models/occlusion_model.h>
#include <cmath>

OcclusionModel::OcclusionModel( tf::TransformListener& tfl)
:tfl_(tfl)
{

}

void OcclusionModel::updateScan(const sensor_msgs::LaserScan& scan){
  ROS_DEBUG_COND(DEBUG_OCCLUSION_MODEL,"OcclusionModel::%s",__func__);
  scan_ = scan;
}

double OcclusionModel::getOcclusionProbability(tf::Stamped<tf::Point> point){
  ROS_DEBUG_COND(DEBUG_OCCLUSION_MODEL,"OcclusionModel::%s",__func__);

  // Transform the point into the coordinate system of the laserscan
  try
  {
    tfl_.transformPoint(scan_.header.frame_id, point, point); //Transform using odometry information into the fixed frame
    ROS_DEBUG_COND(DEBUG_OCCLUSION_MODEL,"OcclusionModel::%s - Transforming point from %s to %s",__func__, point.frame_id_.c_str(), scan_.header.frame_id.c_str());
  }
  catch (...)
  {
    ROS_WARN("TF exception spot 3.");
  }

  double angle  = atan2(point[1],point[0]);
  double r      = point.length();

  if(angle > scan_.angle_min && angle < scan_.angle_max){
    double distance_to_angle_min = angle - scan_.angle_min;
    int range_idx = distance_to_angle_min/scan_.angle_increment;

    // The ranges left and right of the scanpoint
    double r0 = scan_.ranges[range_idx];
    double r1 = scan_.ranges[range_idx+1];


    if(r0 < r && r1 < r){
      return 1.0; // Total shadow
    }

    if(r0 > r && r1 > r){
      return 0.0; // should be observable
    }

    //In other cases return 0.5
    return 0.5;

  }

}

