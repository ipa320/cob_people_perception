// Own includes
#include <people_fusion_node/visualization/visualization_helper.h>
#include <people_fusion_node/visualization/color_gradient.hpp>
#include <people_fusion_node/fusion_node.h>

// Visualization includes
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


VisualizationHelper::VisualizationHelper(NodeHandle nh):
    nh_(nh)
{
  // Set the topic names

  std::cout << "Created VisualizationHelper" << std::endl;

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
        marker.type = visualization_msgs::Marker::CUBE;


        // Set the position as center of the line
        marker.pose.position.x = (*trackerIt)->getCurrentState().getPos().getX();
        marker.pose.position.y = (*trackerIt)->getCurrentState().getPos().getY();
        marker.pose.position.z = 0;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 1.2;

        marker.color.r = 1;
        marker.color.g = 0;
        marker.color.b = 0;
        marker.color.a = 0.5;

        markerArray.markers.push_back(marker);


        //// Add a cylinder from the line center to the label
        visualization_msgs::Marker markerLabel;
        markerLabel.header.frame_id = fixed_frame;
        markerLabel.header.stamp = (*trackerIt)->getCurrentTime();
        markerLabel.ns = "tracker_label";
        markerLabel.id = counter;
        markerLabel.type = visualization_msgs::Marker::TEXT_VIEW_FACING;;


        // Set the position as center of the line
        markerLabel.pose.position.x = (*trackerIt)->getCurrentState().getPos().getX();
        markerLabel.pose.position.y = (*trackerIt)->getCurrentState().getPos().getY();
        markerLabel.pose.position.z = 0;
        markerLabel.scale.z = 0.2;

        markerLabel.color.r = 0;
        markerLabel.color.g = 0;
        markerLabel.color.b = 0;
        markerLabel.color.a = 0.5;


        char buf[100];
        sprintf(buf, "Tracker[%d]", (*trackerIt)->getId());
        markerLabel.text = buf;

        markerArray.markers.push_back(markerLabel);


        counter++;
    }

    visualization_pub_.publish(markerArray);
}

void VisualizationHelper::publishDetectionArray(const cob_perception_msgs::DetectionArray::ConstPtr& detectionArray, int id){

  // Create a marker Array
  visualization_msgs::MarkerArray markerArray;


  for(int i = 0; i < detectionArray->detections.size(); i++){
    //// Add a cylinder from the line center to the label
    visualization_msgs::Marker marker;
    marker.header = detectionArray->header;
    std::stringstream sstm;
    sstm << "tracker" << id;
    marker.ns = sstm.str();
    marker.id = i;
    marker.type = visualization_msgs::Marker::CYLINDER;


    // Set the position as center of the line
    marker.pose.position.x = detectionArray->detections[i].pose.pose.position.x;
    marker.pose.position.y = detectionArray->detections[i].pose.pose.position.y;
    marker.pose.position.z = 0;

    int r,g,b;
    redGreenGradient(id/3.0, r, g, b);

    std::cout << "r: " << r << " g: " << g << " b: " << b << std::endl;

    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;

    marker.color.r = r/255.0;
    marker.color.g = g/255.0;
    marker.color.b = b/255.0;
    marker.color.a = 0.5;

    markerArray.markers.push_back(marker);

  }
  visualization_pub_.publish(markerArray);

}
