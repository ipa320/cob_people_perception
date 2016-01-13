// Own includes
#include <people_fusion_node/visualization/visualization_helper.h>
#include <people_fusion_node/fusion_node.h>

// Visualization includes
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


VisualizationHelper::VisualizationHelper(NodeHandle nh):
    nh_(nh)
{
  // Set the publisher for the visualization
  visualization_pub_= nh_.advertise<visualization_msgs::MarkerArray>("fusion_tracker_visualization", 0);
};

void VisualizationHelper::publishTracker(std::vector<TrackerPtr> &trackerList){

	// Create a marker Array
	visualization_msgs::MarkerArray markerArray;

	int counter = 0;
    for (std::vector<TrackerPtr>::iterator trackerIt = trackerList.begin();
    	trackerIt != trackerList.end();
    	trackerIt++)
    {
        //// Add a cylinder from the line center to the label
        visualization_msgs::Marker marker;
        marker.header.frame_id = fixed_frame;
        marker.header.stamp = (*trackerIt)->getCurrentTime();
        marker.ns = "tracker";
        marker.id = counter;
        marker.type = visualization_msgs::Marker::CYLINDER;


        // Set the position as center of the line
        marker.pose.position.x = (*trackerIt)->getCurrentState().x_;
        marker.pose.position.y = (*trackerIt)->getCurrentState().y_;
        marker.pose.position.z = 0;
        marker.scale.x = 1;
        marker.scale.y = 1;
        marker.scale.z = 1;

        marker.color.r = 1;
        marker.color.g = 0;
        marker.color.b = 0;
        marker.color.a = 0.5;

        markerArray.markers.push_back(marker);

        counter++;
    }

    visualization_pub_.publish(markerArray);
}
