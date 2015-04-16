/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
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
#include <ros/ros.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>
#include <boost/program_options.hpp>
#include <boost/regex.hpp>
#include <boost/filesystem/operations.hpp>

#include <leg_detector/TrainingSetCreatorConfig.h>
#include <leg_detector/laser_processor.h>
#include <leg_detector/calc_leg_features.h>
#include <leg_detector/ClusterMsg.h>
#include <leg_detector/LabeledRangeScanMsg.h>

#include <people_msgs/PositionMeasurement.h>
#include <people_msgs/PositionMeasurementArray.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <rosgraph_msgs/Clock.h>
#include <tf2_msgs/TFMessage.h>

#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

#include <people_tracking_filter/tracker_kalman.h>
#include <people_tracking_filter/state_pos_vel.h>
#include <people_tracking_filter/rgb.h>
#include <visualization_msgs/Marker.h>
#include <dynamic_reconfigure/server.h>

#include <algorithm>

#include <iostream>
#include <string>

#define foreach BOOST_FOREACH

#define USE_BASH_COLORS
#include <leg_detector/color_definitions.h>

//#include <leg_detector/debug.h>

using namespace std;
using namespace laser_processor;
using namespace ros;
using namespace tf;
using namespace estimation;
using namespace BFL;
using namespace MatrixWrapper;

namespace po = boost::program_options;

int g_argc;
char** g_argv;

bool sample_x_comp(laser_processor::Sample* i, laser_processor::Sample* j) {
    return i->x < j->x;
}
bool sample_y_comp(laser_processor::Sample* i, laser_processor::Sample* j) {
    return i->y < j->y;
}

/** Mapping time -> list<SampleSet*>* (Pointer to a list of pointers to SampleSets(Clusters)) */
typedef std::map<ros::Time, list<SampleSet*>*> timeSampleSetMap;
typedef std::map<ros::Time, list<SampleSet*>*>::iterator timeSampleSetMapIt;

/** Mapping time -> list<SampleSet*>* (Pointer to a list of pointers to visual markers) */
typedef std::map<ros::Time, list<visualization_msgs::Marker*>*> timeVisualMarkerMap;
typedef std::map<ros::Time, list<visualization_msgs::Marker*>*>::iterator timeVisualMarkerMapIt;

/**
* The TrainingSetCreator Class helps creating bagfiles with annotated and visualized labels.
*/
class TrainingSetCreator {
public:
    NodeHandle nh_;

    TransformListener tfl_;

    /** The mapping between time and clusters */
    timeSampleSetMap timeClusterMap_;

    /** The mapping between time and visual markers */
    timeVisualMarkerMap timeVisualMarkerMap_;

    float connected_thresh_;

    int feat_count_;

    char save_[100];

    boost::mutex saved_mutex_;

    int feature_id_;

    bool publish_clusters_;
    int next_p_id_;
    int min_points_per_group_;

    // The publishers
    ros::Publisher markers_pub_;
    ros::Publisher clusters_pub_;
    ros::Publisher scan_pub_;
    ros::Publisher clock_pub_;
    ros::Publisher tf_pub_;

    /** The reconfiguration Server */
    dynamic_reconfigure::Server<leg_detector::TrainingSetCreatorConfig> server_;

    /** Constructor */
    TrainingSetCreator(ros::NodeHandle nh) :
        nh_(nh), feat_count_(0), next_p_id_(0) {

        // advertise topics
        markers_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 20);
        clusters_pub_ = nh_.advertise<sensor_msgs::PointCloud>("clusters", 20);
        scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 20);
        clock_pub_ = nh_.advertise<rosgraph_msgs::Clock>("clock", 20);
        tf_pub_ = nh_.advertise<tf2_msgs::TFMessage>("tf", 20);

        dynamic_reconfigure::Server<leg_detector::TrainingSetCreatorConfig>::CallbackType f;
        f = boost::bind(&TrainingSetCreator::configure, this, _1, _2);
        server_.setCallback(f);

        feature_id_ = 0;
    }

    /** Deconstructor */
    ~TrainingSetCreator() {
    }

    /** configure callback - Reacts to changes of the configuration by rqt_reconfigure */
    void configure(leg_detector::TrainingSetCreatorConfig &config, uint32_t level) {
        connected_thresh_ = config.connection_threshold;
        min_points_per_group_ = config.min_points_per_group;
    }

    /**
    * creates the trainingSet bagfile.
    * @param file Input bagfile. Should contain the scan-data.
    */
    void createTrainingSet(const char* input_file, const char* output_file) {
        // Copy the input bagfile
        boost::filesystem::copy_file(input_file, output_file, boost::filesystem::copy_option::overwrite_if_exists);
        cout << "Creating copy" << endl;

        printf("Input file: %s\n", input_file);

        // Check if file exists
        if (!std::ifstream(input_file)) {
            std::cout << "File does not exist!" << std::endl;
            return;
        } else {
            std::cout << "File exists!" << std::endl;
        }

        // Open the bagfile
        rosbag::Bag bag(input_file, rosbag::bagmode::Read);

        // Read the available topics
        rosbag::View topicView(bag);
        std::vector<const rosbag::ConnectionInfo *> connection_infos = topicView.getConnections();
        std::set<std::string> topics_strings;

        cout << "Topics: " << endl;
        BOOST_FOREACH(const rosbag::ConnectionInfo *info, connection_infos) {
          if(topics_strings.find(info->topic) == topics_strings.end()) {
             topics_strings.insert(info->topic);
           cout << "\t" << info->topic << "\t" << info->datatype << endl;
         }
        }

        // TODO Check if allready a topic label topic there

        // TODO should be later all the topics
        std::vector<std::string> topics;
        topics.push_back(std::string("/scan_front"));
        //topics.push_back(std::string("/scan_rear"));
        topics.push_back(std::string("/tf"));
        std::cout << "Pushed topics" << std::endl;

        // Create the view
        rosbag::View view(bag, rosbag::TopicQuery(topics));
        //rosbag::View view(bag, rosbag::TopicQuery(topics));

        rosbag::View::iterator viewIt = view.begin();
        rosbag::MessageInstance message = (*viewIt);

        // Iterate through the message -> generate the clusters
        foreach(rosbag::MessageInstance const m, view)
        {
            // If scan is a LaserScan (to avoid stopping at every tf)
            sensor_msgs::LaserScan::Ptr s = m.instantiate<sensor_msgs::LaserScan>();
            if (s != NULL) {
                // Process the scan, generate the clusters
                ScanProcessor* pProcessor = new ScanProcessor(*s);
                pProcessor->splitConnected(connected_thresh_);
                pProcessor->removeLessThan(min_points_per_group_);

                // Create Labeled Clusters if necessary
                list<SampleSet*>* pClusters = &(pProcessor->getClusters());
                vector<SampleSet*>* pClustersVec = new std::vector<SampleSet*>(pClusters->begin(), pClusters->end());

                // Assign ids to the clusters
                int count = 0;
                for (list<SampleSet*>::iterator clusterIt = pClusters->begin(); clusterIt != pClusters->end(); clusterIt++) {
                      (*clusterIt)->id_ = count;
                      count++;
                 }

                // Insert this clusters with their labels into the map. Important: These are only pointers. Change is still possible! Change is always possible ;-)
                timeClusterMap_.insert(std::pair<ros::Time, list<SampleSet*>*>(m.getTime(),pClusters));
                cout << m.getTime() << "clustered and saved!" << endl;
            }
        }

        bool run=false;
        viewIt = view.begin();

        // Store the first seq for later
        //rosbag::MessageInstance m = (*viewIt);

        while (viewIt != view.end() && !run) {

            rosbag::MessageInstance m = (*viewIt);

            // Before anything publish the current Time
            rosgraph_msgs::Clock::Ptr simTime(new rosgraph_msgs::Clock());
            simTime->clock = m.getTime();
            clock_pub_.publish(simTime);
            cout << "Publishing time" << simTime <<  endl;


            // Check if tf Message -> these should be 'forwarded'
            tf2_msgs::TFMessagePtr tfMsgPtr = m.instantiate<tf2_msgs::TFMessage>();
            if (tfMsgPtr != NULL) {
                cout << "Publish tf: " << tfMsgPtr->transforms[0].header.stamp << endl;
                tf_pub_.publish(tfMsgPtr);
                ++viewIt;
            }

            // If scan is a LaserScan (to avoid stopping at every tf)
            sensor_msgs::LaserScan::Ptr s = m.instantiate<sensor_msgs::LaserScan>();
            if (s != NULL) {

                // Get the clusters (as list and vec)
                list<SampleSet*>* pClusters;
                vector<SampleSet*>* pClustersVec;

                timeSampleSetMapIt search = timeClusterMap_.find(m.getTime());

                if (search != timeClusterMap_.end() ) {
                    pClusters = search->second;
                    pClustersVec = new std::vector<SampleSet*>(pClusters->begin(), pClusters->end());
                }else{
                    cout << "Error! Could not find associated clusters!" << endl;
                    return;
                }


                scan_pub_.publish(s);
                // TODO seems to be needed for proper label update inside rviz, yet very ugly
                ros::spinOnce();
                for(int i=0; i<10;i++){
                    publishClusters(*pClusters, s, m.getTime());
                    ros::spinOnce();
                }

                cout << "publish cluster" << endl;



                // Print the current cluster
                // cout << CLEAR << endl;
                cout << GREEN << "Time: " << m.getTime() << RESET << " Seq: " << YELLOW << s->header.seq << RESET << std::endl;
                for (list<SampleSet*>::iterator clusterIt = pClusters->begin(); clusterIt != pClusters->end(); clusterIt++){
                    cout << RED << "[" << (*clusterIt)->id_ << "]" << (*clusterIt)->size() << (*clusterIt)->label << RESET << endl;
                }

                // Inform user about the usage
                cout << "Usage:" << endl;
                cout << "\t(n)ext - Jump to next laser scan message" << endl;
                cout << "\t(p)revious - Jump to previous laser scan message" << endl;
                cout << "\t[#n] \'label\' - Assign label \'label\' to cluster #n" << endl;
                cout << "\t(s)ave - Save and exit" << endl;
                cout << "\t(e)xit without saving" << endl;

                // Get User input
                std::string userinput;
                std::getline(std::cin, userinput);

                //regex
                boost::regex expr_label("#?([0-9]+) ([a-zA-z]+) *"); //User enters label for a cluster
                boost::regex expr_next("next|n"); //User wants to see the next frame
                boost::regex expr_prev("prev|p"); //User wants to see the previous frame
                boost::regex expr_exit("exit|e"); //User wants to exit the program without saving
                boost::regex expr_run("s|save"); //User wants to store

                boost::cmatch what;

                // Set label
                if (boost::regex_match(userinput.c_str(), what, expr_label))
                    {
                    int clusterNumber = boost::lexical_cast<int>(what[1]);
                    std::string label = boost::lexical_cast<string>(what[2]);

                    // Check that this Index is within reach
                    if (clusterNumber >= pClustersVec->size()) {
                        cout << "Index out of reach" << endl;
                    } else {
                        cout << clusterNumber << " " << label << endl;
                        (*pClustersVec)[clusterNumber]->label = label;
                    }
                } else if (boost::regex_match(userinput.c_str(), what, expr_next)) { // Next
                    viewIt++;
                }
                else if (boost::regex_match(userinput.c_str(), what, expr_prev)) { // Prev
                    cout << "Prev" << endl;
                    // Store current seq
                    int currentSeq = s->header.seq;
                    // Reset iterator
                    viewIt = view.begin();
                    while(viewIt != view.end()){
                        rosbag::MessageInstance m = (*viewIt);

                        // Publish current time
                        rosgraph_msgs::Clock::Ptr simTime(new rosgraph_msgs::Clock());
                        simTime->clock = m.getTime();
                        clock_pub_.publish(simTime);

                        sensor_msgs::LaserScan::Ptr s = m.instantiate<sensor_msgs::LaserScan>();
                        if(s != NULL){
                            cout << s->header.seq << endl;
                            if(currentSeq < s->header.seq + 2){
                                break;
                            }
                        }

                        // Check if tf Message -> these should be 'forwarded'
                        tf2_msgs::TFMessagePtr tfMsgPtr = m.instantiate<tf2_msgs::TFMessage>();
                        if (tfMsgPtr != NULL) {
                            cout << "TF-Message: " << tfMsgPtr->transforms[0].header.stamp << endl;
                            tf_pub_.publish(tfMsgPtr);
                        }

                        ++viewIt;
                    }
                }else if (boost::regex_match(userinput.c_str(), what, expr_exit)) { // Exit
                    cout << "exit!" << endl;
                    return;
                } else if (boost::regex_match(userinput.c_str(), what, expr_run)) { // Run
                    run = true;
                } else { // Unknown input
                    cout << "unkown input!" << endl;
                }

            }
            ros::spinOnce();
        } //foreach


        // Write labels to the output file
        cout << "Writing labels to " << output_file << endl;

        //The output file
        rosbag::Bag output_bag;
        output_bag.open(output_file, rosbag::bagmode::Write);

        rosbag::View view_all(bag);

        // Iterate through the messages
        foreach(rosbag::MessageInstance const m, view_all) {
            //cout << "topic: " << m.getTopic() << " Time. " << m.getTime() << std::endl;

            // Write the message to the output file
            output_bag.write(m.getTopic(), m.getTime(), m);

            // Check if that message is inside timeClusterMap_
            sensor_msgs::LaserScan::Ptr s = m.instantiate<sensor_msgs::LaserScan>();

            // If message is a laserscan
            if (s != NULL) {

                // Check if there are labels associated
                timeSampleSetMapIt search = timeClusterMap_.find(m.getTime());
                if (search != timeClusterMap_.end() ) {

                    // The Labeled Range Scan Message
                    leg_detector::LabeledRangeScanMsg rangeScanLabelMsg;

                    // Use the same header as
                    rangeScanLabelMsg.header = s->header;

                    // Generate Pointcloud
                    sensor_msgs::PointCloudPtr pPcl = generatePointCloudMessageFromClusters(*(search->second),s->header);
                    output_bag.write("/clusters", m.getTime(), *pPcl);

                    // Generate Label Messages
                    list<visualization_msgs::Marker>* pLabelMarkers = generateLabelMarkerMsgsFromClusters(*(search->second),s->header);
                    for(list<visualization_msgs::Marker>::iterator markerPtrIt = pLabelMarkers->begin(); markerPtrIt != pLabelMarkers->end(); markerPtrIt++){
                        output_bag.write("/visualization_marker", m.getTime(), *markerPtrIt);
                    }

                    // Iterate the clusters
                    for(list<SampleSet*>::iterator clusterIt = search->second->begin(); clusterIt != search->second->end(); clusterIt++){

                        // A single ClusterMsg
                        leg_detector::ClusterMsg clusterMsg;

                        // If this cluster is labeled
                        if(!(*clusterIt)->label.empty()){
                            clusterMsg.label = (*clusterIt)->label;
                            cout << GREEN << s->header.stamp << RESET << endl;
                            cout << "\tLabel:[" << clusterMsg.label << "]" << endl;
                            // Iterate through the samples of this cluster
                            for(std::set<laser_processor::Sample*>::iterator sampleIt = (*clusterIt)->begin(); sampleIt != (*clusterIt)->end(); sampleIt++){
                                // Add the indice to the ClusterMsg
                                clusterMsg.indices.push_back((int16_t)(*sampleIt)->index);
                                int16_t index = (int16_t)(*sampleIt)->index;
                                //cout << BLUE << "\t" << (int)((uint8_t)(*sampleIt)->index) << RESET << endl;

                                laser_processor::Sample* s = *sampleIt;
                                cout << "\t\t" << YELLOW << static_cast<int16_t>(index) <<  " " << RED << "x: " << s->x << " y: " << s->y << BLUE << " range: " << s->range << RESET << endl;


                            }
                            // Add the cluster message to the rangeScanLabelMsg
                            rangeScanLabelMsg.clusters.push_back(clusterMsg);
                        }
                    }

                    // write the message
                    output_bag.write("/labels", m.getTime(), rangeScanLabelMsg);
                }
            }

        }
        // Close the files
        bag.close();

        output_bag.close();

    }

    visualization_msgs::Marker getRectangleMarkerForCluster(SampleSet* cluster,
            int id, float r = 0, float g = 0, float b = 0) {
        float x_min_value = (*std::min_element(cluster->begin(), cluster->end(),
                sample_x_comp))->x;
        float x_max_value = (*std::max_element(cluster->begin(), cluster->end(),
                sample_x_comp))->x;
        float y_min_value = (*std::min_element(cluster->begin(), cluster->end(),
                sample_y_comp))->y;
        float y_max_value = (*std::max_element(cluster->begin(), cluster->end(),
                sample_y_comp))->y;

        //  ^
        //  |    p0 ------------- p1
        //  |     |               |
        //  y    p3 ------------- p2
        //    x----->

        geometry_msgs::Point p0;
        p0.x = (double) x_min_value;
        p0.y = (double) y_max_value;
        p0.z = (double) 0;
        geometry_msgs::Point p1;
        p1.x = (double) x_max_value;
        p1.y = (double) y_max_value;
        p1.z = (double) 0;

        geometry_msgs::Point p2;
        p2.x = (double) x_max_value;
        p2.y = (double) y_min_value;
        p2.z = (double) 0;

        geometry_msgs::Point p3;
        p3.x = (double) x_min_value;
        p3.y = (double) y_min_value;
        p3.z = (double) 0;

        visualization_msgs::Marker rectangle;
        rectangle.points.push_back(p0);
        rectangle.points.push_back(p1);
        rectangle.points.push_back(p2);
        rectangle.points.push_back(p3);
        rectangle.points.push_back(p0);

        rectangle.header.frame_id = "map";
        rectangle.header.stamp = ros::Time::now();
        rectangle.ns = "SELECTION";
        rectangle.id = id;
        rectangle.type = rectangle.LINE_STRIP;
        rectangle.color.a = 1;
        rectangle.scale.x = .03;
        rectangle.color.r = r;
        rectangle.color.g = g;
        rectangle.color.b = b;

        return rectangle;

    }

    sensor_msgs::PointCloudPtr generatePointCloudMessageFromClusters(list<SampleSet*>& clusters, std_msgs::Header header){
        sensor_msgs::PointCloudPtr pClusters_pcl(new sensor_msgs::PointCloud);
        sensor_msgs::ChannelFloat32 rgb_channel;
        rgb_channel.name = "rgb";
        pClusters_pcl->channels.push_back(rgb_channel);
        pClusters_pcl->header = header;

        for (list<SampleSet*>::iterator i = clusters.begin(); i != clusters.end(); i++) {

            int r[3] = { 50, 180, 255 };
            int g[3] = { 50, 180, 255 };
            int b[3] = { 50, 180, 255 };

            int r_ind = (*i)->id_ % 3;
            int g_ind = ((*i)->id_ / 3) % 3;
            int b_ind = ((*i)->id_ / 9) % 3;

            (*i)->appendToCloud(*pClusters_pcl, r[r_ind], g[g_ind], b[b_ind]);
        }
        return pClusters_pcl;
    }


    list<visualization_msgs::Marker>* generateLabelMarkerMsgsFromClusters(list<SampleSet*>& clusters, std_msgs::Header header){

        list<visualization_msgs::Marker>* pMarkerMsgPtrList = new list<visualization_msgs::Marker>;

        sensor_msgs::PointCloudPtr pClusters_pcl(new sensor_msgs::PointCloud);
        sensor_msgs::ChannelFloat32 rgb_channel;
        rgb_channel.name = "rgb";
        pClusters_pcl->channels.push_back(rgb_channel);
        pClusters_pcl->header = header;

        int r[3] = { 50, 180, 255 };
        int g[3] = { 50, 180, 255 };
        int b[3] = { 50, 180, 255 };

        for (list<SampleSet*>::iterator i = clusters.begin(); i != clusters.end(); i++) {

            visualization_msgs::Marker m_text;
            m_text.header = header;
            m_text.ns = "CLUSTERS_LABEL";
            m_text.id = (*i)->id_;
            m_text.type = m_text.TEXT_VIEW_FACING;
            m_text.pose.position.x = (*i)->center()[0] + 0.15;
            m_text.pose.position.y = (*i)->center()[1] + 0.15;
            m_text.pose.position.z = (*i)->center()[2];
            m_text.scale.z = .15;
            m_text.color.a = 1;
            m_text.lifetime = ros::Duration(0.00001);

            // Add text
            char buf[100];
            int r_ind = (*i)->id_ % 3;
            int g_ind = ((*i)->id_ / 3) % 3;
            int b_ind = ((*i)->id_ / 9) % 3;
            m_text.color.r = r[r_ind] / 255.0;
            m_text.color.g = g[g_ind] / 255.0;
            m_text.color.b = b[b_ind] / 255.0;
            if((*i)->label.empty()){
                sprintf(buf, "#%d-p%lu", (*i)->id_, (*i)->size());
            }
            else{
                sprintf(buf, "#%d-p%lu-%s", (*i)->id_, (*i)->size(), (*i)->label.c_str());
            }
            m_text.text = buf;

            pMarkerMsgPtrList->push_back(m_text);
        }
        // Clearing markers
        for (int i = 30; clusters.size() <= i; i--) {
            visualization_msgs::Marker clearingMarker;
            clearingMarker.header = header;
            clearingMarker.ns = "CLUSTERS_LABEL";
            clearingMarker.id = i;
            clearingMarker.type = clearingMarker.TEXT_VIEW_FACING;
            clearingMarker.scale.x = 1;
            clearingMarker.scale.y = 1;
            clearingMarker.scale.z = 1;
            clearingMarker.color.a = 0;
            clearingMarker.text = "a";

            pMarkerMsgPtrList->push_back(clearingMarker);
        }

        return pMarkerMsgPtrList;
    }

    void publishClusters(list<SampleSet*> clusters, const sensor_msgs::LaserScan::ConstPtr& scan, ros::Time time) {

        sensor_msgs::PointCloud clusters_pcl;
        sensor_msgs::ChannelFloat32 rgb_channel;
        rgb_channel.name = "rgb";
        clusters_pcl.channels.push_back(rgb_channel);
        clusters_pcl.header = scan->header;
        clusters_pcl.header.stamp = time;
        clusters_pcl.header.frame_id = scan->header.frame_id;

        for (list<SampleSet*>::iterator i = clusters.begin(); i != clusters.end(); i++) {

            int r[3] = { 50, 180, 255 };
            int g[3] = { 50, 180, 255 };
            int b[3] = { 50, 180, 255 };

            int r_ind = (*i)->id_ % 3;
            int g_ind = ((*i)->id_ / 3) % 3;
            int b_ind = ((*i)->id_ / 9) % 3;

            //cout << "r_ind " << r_ind << " g_ind " << g_ind << " b_ind " << b_ind << endl;

            (*i)->appendToCloud(clusters_pcl, r[r_ind], g[g_ind], b[b_ind]);

            //cout << "Hash: " << scan->header.stamp << "-" << (*i)->center()[0] << "-" << (*i)->center()[1] << "-" << (*i)->center()[3] << endl;

            visualization_msgs::Marker m_text;
            m_text.header = clusters_pcl.header;
            m_text.header.frame_id = scan->header.frame_id;
            m_text.header.stamp = time;

            m_text.ns = "CLUSTERS_LABEL";
            m_text.id = (*i)->id_;
            m_text.type = m_text.TEXT_VIEW_FACING;
            m_text.pose.position.x = (*i)->center()[0] + 0.15;
            m_text.pose.position.y = (*i)->center()[1] + 0.15;
            m_text.pose.position.z = (*i)->center()[2];
            m_text.scale.z = .15;
            m_text.color.a = 1;

            // Add text
            char buf[100];
            m_text.color.r = r[r_ind] / 255.0;
            m_text.color.g = g[g_ind] / 255.0;
            m_text.color.b = b[b_ind] / 255.0;
            sprintf(buf, "#%d-p%lu-%s", (*i)->id_, (*i)->size(), (*i)->label.c_str());

            m_text.text = buf;

            markers_pub_.publish(m_text);

            //markers_pub_.publish(getRectangleMarkerForCluster(*i,count,r[r_ind]/255.0,g[g_ind]/255.0,b[b_ind]/255.0));

        }
        // Clearing markers
        for (int i = 30; clusters.size() <= i; i--) {
            visualization_msgs::Marker clearingMarker;
            clearingMarker.header = clusters_pcl.header;
            clearingMarker.header.stamp = time;
            clearingMarker.ns = "CLUSTERS_LABEL";
            clearingMarker.id = i;
            clearingMarker.type = clearingMarker.TEXT_VIEW_FACING;
            clearingMarker.action = visualization_msgs::Marker::DELETE;

            markers_pub_.publish(clearingMarker);
        }

        cout << "Published markers with time:" << time << endl;

        clusters_pub_.publish(clusters_pcl);

    }

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "training_set_creator");
    g_argc = argc;
    g_argv = argv;
    ros::NodeHandle nh;
    TrainingSetCreator trainingSetCreator(nh);

    po::options_description description("Training Set Creator Usage");

    description.add_options()("help,h", "Display this help message")(
            "input-file", po::value<std::string>(), "Input file")("output-file",
            po::value<std::string>(), "Output file")("version,v",
            "Display the version number");

    po::positional_options_description p;
    p.add("input-file", 1);
    p.add("output-file", 1);

    po::variables_map vm;
    po::store(
            po::command_line_parser(argc, argv).options(description).positional(
                    p).run(), vm);
    po::notify(vm);

    if (vm.count("help")) {
        std::cout << description;

        return 0;
    }

    if (vm.count("version")) {
        std::cout << "Training Set Creator Version 0.1" << std::endl;

        return 0;
    }

    if (vm.count("input-file")) {
        cout << "Input files are: " << vm["input-file"].as<std::string>()
                << "\n";
    }

    if (vm.count("output-file")) {
        cout << "output files are: " << vm["output-file"].as<std::string>() << "\n";
    }

    const char* input_file = (vm["input-file"].as<std::string>()).c_str();
    const char* output_file = (vm["output-file"].as<std::string>()).c_str();

    trainingSetCreator.createTrainingSet(input_file, output_file);

    return 0;
}

