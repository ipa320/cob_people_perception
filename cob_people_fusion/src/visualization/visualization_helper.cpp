// Own includes
#include <cob_people_fusion/visualization/visualization_helper.h>
#include <cob_people_fusion/visualization/color_gradient.hpp>
#include <cob_people_fusion/fusion_node.h>

// Visualization includes
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


VisualizationHelper::VisualizationHelper(NodeHandle nh, size_t totalNumberDetectors):
    totalNumberDetectors_(totalNumberDetectors),
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
        marker.color.a = 0.2;

        marker.lifetime = ros::Duration(1);

        if((*trackerIt)->getDiversity() > 1) marker.color.a = 0.8;

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
        markerLabel.scale.z = 0.3;

        markerLabel.color.r = 0;
        markerLabel.color.g = 0;
        markerLabel.color.b = 0;
        markerLabel.color.a = 0.5;

        markerLabel.lifetime = ros::Duration(1);


        std::stringstream label_stream;
        label_stream.precision(2);
        label_stream << **trackerIt;

        std::map<std::string, size_t> updateCounts = (*trackerIt)->getUpdateCounts();

        std::map<std::string, double> updateFrequencies = (*trackerIt)->getUpdateFrequencies();

        for(std::map<std::string, size_t>::iterator mapIt = updateCounts.begin(); mapIt != updateCounts.end(); mapIt++) {
          label_stream << "|c:" << mapIt->first << ": " << mapIt->second << " f:" << updateFrequencies[mapIt->first] << "|";
        }

        markerLabel.text = label_stream.str();
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


    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;

    marker.lifetime = ros::Duration(1);

    int r,g,b;
    double value = id/((double) totalNumberDetectors_);
    redGreenGradient(value, r, g, b);

    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;


    marker.color.a = 0.25;

    markerArray.markers.push_back(marker);

  }
  visualization_pub_.publish(markerArray);

}
