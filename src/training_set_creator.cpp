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

#define RESET   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */
#define BOLDBLACK   "\033[1m\033[30m"      /* Bold Black */
#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
#define BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
#define BOLDBLUE    "\033[1m\033[34m"      /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m"      /* Bold Magenta */
#define BOLDCYAN    "\033[1m\033[36m"      /* Bold Cyan */
#define BOLDWHITE   "\033[1m\033[37m"      /* Bold White */

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

typedef std::map<ros::Time, list<SampleSet*>*> timeSampleSetMap;
typedef std::map<ros::Time, list<SampleSet*>*>::iterator timeSampleSetMapIt;

// actual legdetector node
class TrainingSetCreator {
public:
    NodeHandle nh_;

    TransformListener tfl_;

    timeSampleSetMap clusterTimeMap_;

    ScanMask mask_;

    int mask_count_;

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
    ros::Publisher clock_pub_;
    ros::Publisher tf_pub_;


    dynamic_reconfigure::Server<leg_detector::TrainingSetCreatorConfig> server_;

    TrainingSetCreator(ros::NodeHandle nh) :
        nh_(nh), mask_count_(0), feat_count_(0), next_p_id_(0) {

        // advertise topics
        markers_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 20);
        clusters_pub_ = nh_.advertise<sensor_msgs::PointCloud>("clusters", 20);
        clock_pub_ = nh_.advertise<rosgraph_msgs::Clock>("clock", 20);
        tf_pub_ = nh_.advertise<tf2_msgs::TFMessage>("tf", 20);

        dynamic_reconfigure::Server<leg_detector::TrainingSetCreatorConfig>::CallbackType f;
        f = boost::bind(&TrainingSetCreator::configure, this, _1, _2);
        server_.setCallback(f);

        feature_id_ = 0;
    }

    ~TrainingSetCreator() {
    }

    void configure(leg_detector::TrainingSetCreatorConfig &config, uint32_t level) {
        connected_thresh_ = config.connection_threshold;
        min_points_per_group_ = config.min_points_per_group;
    }

    void createTrainingSet(const char* file) {
        // Copy the input bagfile
        boost::filesystem::copy_file(file,"output.bag",boost::filesystem::copy_option::fail_if_exists);
        cout << "Creating copy" << endl;

        printf("Input file: %s\n", file);

        // Check if file exists
        if (!std::ifstream(file)) {
            std::cout << "File does not exist!" << std::endl;
            return;
        } else {
            std::cout << "File exists!" << std::endl;
        }

        rosbag::Bag bag;
        bag.open(file, rosbag::bagmode::Read);

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

        // Iterate through the message
        // foreach(rosbag::MessageInstance const m, view)
        // {
        bool run=false;

        while (viewIt != view.end()) {
            if(run){
                break;
            }

            rosbag::MessageInstance m = (*viewIt);

            // Before anything publish the current Time
            rosgraph_msgs::Clock::Ptr simTime(new rosgraph_msgs::Clock());
            simTime->clock = m.getTime();
            clock_pub_.publish(simTime);

            // Check if tf Message -> these should be 'forwarded'
            tf2_msgs::TFMessagePtr tfMsgPtr = m.instantiate<tf2_msgs::TFMessage>();
            if (tfMsgPtr != NULL) {
                //cout << "TF-Message: " << tfMsgPtr->transforms[0].header.stamp << endl;
                tf_pub_.publish(tfMsgPtr);
                ++viewIt;
            }

            // If scan is a LaserScan (to avoid stopping at every tf)
            sensor_msgs::LaserScan::Ptr s = m.instantiate<sensor_msgs::LaserScan>();
            if (s != NULL) {

                std::cout << "topic : " << m.getTopic() << " " << m.getTime()<< endl;

                // Process the scan, generate the clusters
                ScanProcessor* pProcessor = new ScanProcessor(*s, mask_);
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

                //User interaction loop
                while (true) {

                    // TODO seems to be needed for proper label update inside rviz, yet very ugly
                    ros::spinOnce();
                    publishClusters(*pClusters, s);
                    ros::spinOnce();
                    ros::spinOnce();
                    ros::spinOnce();

                    clusterTimeMap_.insert(std::pair<ros::Time, list<SampleSet*>*>(m.getTime(),pClusters));
                    cout << "Inserted into map!" << endl;

                    for (timeSampleSetMapIt iterator = clusterTimeMap_.begin(); iterator != clusterTimeMap_.end(); iterator++) {
                        cout << GREEN << "Time: " << iterator->first << RESET << std::endl;

                        // Print out the clusters with their assigned labels
                        for (list<SampleSet*>::iterator clusterIt = iterator->second->begin(); clusterIt != iterator->second->end(); clusterIt++) {
                            cout << RED << "[" << (*clusterIt)->id_ << "]" << (*clusterIt)->size() << (*clusterIt)->label << RESET << endl;
                        }
                    }

                    // Inform user about the usage
                    cout << "Usage:" << endl;
                    cout << "\t(n)ext - Jump to next laser scan message" << endl;
                    cout << "\t[#n] \'label\' - Assign label \'label\' to cluster #n" << endl;
                    cout << "\t(e)xit" << endl;

                    // Get User input
                    std::string userinput;
                    if(!run){
                        std::getline(std::cin, userinput);
                    }else{
                        viewIt++;
                        break;
                    }

                    //regex
                    boost::regex expr_label("#?([0-9]+) ([a-zA-z]+) *"); //User enters label for a cluster
                    boost::regex expr_next("next|n"); //User enters label for a cluster
                    boost::regex expr_exit("exit|e"); //User enters label for a cluster
                    boost::regex expr_run("r|run"); //User enters label for a cluster

                    boost::cmatch what;

                    if (boost::regex_match(userinput.c_str(), what, expr_label)) // Set label
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
                        continue;

                    } else if (boost::regex_match(userinput.c_str(), what, expr_next)) { // Next
                        viewIt++;
                        break;
                    } else if (boost::regex_match(userinput.c_str(), what, expr_exit)) { // Exit
                        cout << "exit!" << endl;
                        return;
                    } else if (boost::regex_match(userinput.c_str(), what, expr_run)) { // Exit
                        run = true;
                    } else { // Unknown input
                        cout << "unkown input!" << endl;
                    }
                }// User interaction loop
            }
            ros::spinOnce();
        } //foreach

        // Write labels to the output file
        cout << "Writing labels to the output" << endl;

        //The output file
        rosbag::Bag output_bag;
        output_bag.open("output.bag", rosbag::bagmode::Write);

        rosbag::View view_all(bag);

        // Iterate through the messages
        foreach(rosbag::MessageInstance const m, view_all) {
            //cout << "topic: " << m.getTopic() << " Time. " << m.getTime() << std::endl;

            // Write the message to the output file
            output_bag.write(m.getTopic(), m.getTime(), m);

            // Check if that message is inside clusterTimeMap_
            sensor_msgs::LaserScan::Ptr s = m.instantiate<sensor_msgs::LaserScan>();
            if (s != NULL) {
                timeSampleSetMapIt search = clusterTimeMap_.find(m.getTime());
                if (search == clusterTimeMap_.end() ) {

                }
                else
                {
                    cout << "Found labels at time: " << search->first << endl;
                    // The Labeled Range Scan Message
                    leg_detector::LabeledRangeScanMsg rangeScanLabelMsg;

                    // Use the same header as
                    rangeScanLabelMsg.header = s->header;

                    // Iterate the clusters
                    for(list<SampleSet*>::iterator clusterIt = search->second->begin(); clusterIt != search->second->end(); clusterIt++){
                        // A single ClusterMsg
                        leg_detector::ClusterMsg clusterMsg;

                        // If this cluster is labeled
                        std::string test  = "test";
                        if(!(*clusterIt)->label.empty()){
                            clusterMsg.label = (*clusterIt)->label;

                            cout << "Adding indices to message!" << endl;
                            // Iterate through the samples of this cluster
                            for(std::set<laser_processor::Sample*>::iterator sampleIt = (*clusterIt)->begin(); sampleIt != (*clusterIt)->end(); sampleIt++){
                                // Add the indice to the ClusterMsg
                                clusterMsg.indices.push_back((unsigned int)(*sampleIt)->index);
                                cout << BLUE << "\t" << (unsigned int)(*sampleIt)->index << RESET << endl;
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

    void publishClusters(list<SampleSet*> clusters, const sensor_msgs::LaserScan::ConstPtr& scan) {

        sensor_msgs::PointCloud clusters_pcl;
        sensor_msgs::ChannelFloat32 rgb_channel;
        rgb_channel.name = "rgb";
        clusters_pcl.channels.push_back(rgb_channel);
        clusters_pcl.header = scan->header;
        clusters_pcl.header.stamp = scan->header.stamp;
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
            m_text.header.stamp = scan->header.stamp;
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
            clearingMarker.ns = "CLUSTERS_LABEL";
            clearingMarker.id = i;
            clearingMarker.type = clearingMarker.TEXT_VIEW_FACING;
            clearingMarker.scale.x = 1;
            clearingMarker.scale.y = 1;
            clearingMarker.scale.z = 1;
            clearingMarker.color.a = 0;
            clearingMarker.text = "a";

            markers_pub_.publish(clearingMarker);
        }

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
        cout << "Input files are: " << vm["input-file"].as<std::string>()
                << "\n";
    }

    const char* filename = (vm["input-file"].as<std::string>()).c_str();

    trainingSetCreator.createTrainingSet(filename);

    return 0;
}

