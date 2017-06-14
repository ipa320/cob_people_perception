/*
 *****************************************************************
 * Copyright (c) 2015 \n
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 * Project name: Care-O-bot
 * \note
 * ROS stack name: cob_people_perception
 * \note
 * ROS package name: cob_leg_detection
 *
 * \author
 * Author: Olha Meyer
 * \author
 * Supervised by: Richard Bormann
 *
 * \date Date of creation: 01.11.2014
 *
 * \brief
 * functions for detecting people within a scan data cloud
 * current approach: read the current scan data.
 * Assign each valid pair of detected legs to a person.
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer. \n
 * - Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution. \n
 * - Neither the name of the Fraunhofer Institute for Manufacturing
 * Engineering and Automation (IPA) nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

 /*
  * Other Copyrights :
  *
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

#include <cob_leg_detection/LegDetectionConfig.h>
#include <cob_leg_detection/laser_processor.h>
#include <cob_leg_detection/calc_leg_features.h>

#include <opencv2/opencv.hpp>

#include <cob_perception_msgs/PositionMeasurement.h>
#include <cob_perception_msgs/PositionMeasurementArray.h>
#include <cob_perception_msgs/Person.h>
#include <cob_perception_msgs/People.h>
#include <sensor_msgs/LaserScan.h>

#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

#include <cob_people_tracking_filter/tracker_kalman.h>
#include <cob_people_tracking_filter/state_pos_vel.h>
#include <cob_people_tracking_filter/rgb.h>
#include <visualization_msgs/Marker.h>
#include <dynamic_reconfigure/server.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <algorithm>
#include <vector>

using namespace std;
using namespace laser_processor;
using namespace ros;
using namespace tf;
using namespace estimation;
using namespace BFL;
using namespace MatrixWrapper;


static double no_observation_timeout_s = 0.5;
static double max_second_leg_age_s     = 2.0 ;  //2.0;
static double max_track_jump_m         = 1.0; 
static double max_meas_jump_m          = 0.75; // 1.0
static double leg_pair_separation_m    = 0.50;  //1.0;
static string fixed_frame              = "base_link";
static double cov_meas_legs_m          = 0.025;
static double cov_meas_people_m        = 0.025;

static double kal_p = 4, kal_q = .002, kal_r = 10;
static bool use_filter = true;

static string detector_ = "laser_scaner";



/*
 * INFO TO KF:
 *
 * the Kalman filter is a recursive optimization algorithm
 * that generates an estimate based upon potentially noisy
 * observation data.
 *
 * The estimation of the Kalman filter operates in two primary cycles, propagation and correction.
 * During the propagation cycle, the filter propagates the state of the system, using a system model
 * to predict the state of the system one time step in the future.
 *
 * The correction cycle inputs measurements of the system state and utilizes these observations to correct
 *  for differences between the state propagated from the system model and the measured satellite state.
 *  However, the correction cycle encounters particular difficulty due to the fact that some amount of noise
 *  and imprecision is embodied in the measurements themselves.
 *
 *  Additional INFO on:
 *  [http://www.usafa.edu/df/dfas/Papers/20042005/
 * Kalman%20Filtering%20and%20the%20Attitude%20Determination%20and%20Control%20Task%20-%20Hale.pdf]
 */

class SavedPersonFeature
{
public:

	string id_;
	string object_id;
	string person_name;

	Stamped<Point> position_;
	Stamped<Vector3> velocity_;

	//static int nextid;
	//TransformListener& tfl_;
	BFL::StatePosVel sys_sigma_;
	TrackerKalman filter_;
	ros::Time time_;
	ros::Time meas_time_;

	double reliability, p, probability;

	// person tracker
	SavedPersonFeature(Stamped<Point> loc, std::string& id, std::string& name)
	: sys_sigma_(Vector3(0.05, 0.05, 0.05), Vector3(1.0, 1.0, 1.0)),
	  filter_("tracker_people",sys_sigma_),
	  reliability(-1.), p(4)
	{
		object_id = id;
		person_name = name;
		time_ = loc.stamp_;

		//P-Matrix = covariance matrix
		//Q-Matrix = process noise covariance matrix
		//F-Matrix = state matrix



		StatePosVel prior_sigma(Vector3(0.1,0.1,0.1), Vector3(0.0000001, 0.0000001, 0.0000001));
		filter_.initialize(loc, prior_sigma, time_.toSec());
		StatePosVel est;
		filter_.getEstimate(est);
		updatePosition();
	}
	/*
	 * predicts parameter values ahead of current
	 * measurements
	 */
	void propagate(ros::Time time)
	{
		time_ = time;
		filter_.updatePrediction(time.toSec());
		updatePosition();
	}

	/*
	 * updates current values with correction:
	 * estimates parameter values using future, current
	 * and previous measurements
	 */
	void update(Stamped<Point> loc, double probability)
	{
		//float cov_meas = 0.05;
		//float cov_meas = 0.0025;
		meas_time_ = loc.stamp_;
		time_ = meas_time_;

		//R-Matrix
		SymmetricMatrix cov(3);
		cov = 0.0;
		cov(1,1) = cov_meas_people_m;
		cov(2,2) = cov_meas_people_m;
		cov(3,3) = cov_meas_people_m;

		filter_.updateCorrection(loc, cov);
		updatePosition();

	}
	/*
	 * time between prediction and correction:
	 * lifetime of a tracker
	 */
	double getLifetime()
	{
		return filter_.getLifetime();
	}

	double getReliability()
	{
		return reliability;
	}

private:
	/*
	 * estimates parameter values using current and
	 * previous measurements
	 */
	void updatePosition()
	{
		StatePosVel est;
		filter_.getEstimate(est);

		position_[0] = est.pos_[0];
		position_[1] = est.pos_[1];
		position_[2] = est.pos_[2];

		velocity_[0] = est.vel_[0];
		velocity_[1] = est.vel_[1];
		velocity_[2] = est.vel_[2];

		position_.stamp_ = time_;
		position_.frame_id_ = fixed_frame;
		velocity_.stamp_ = time_;
		velocity_.frame_id_ = fixed_frame;
	}
};

class SavedFeature
{
public:
	static int nextid;
	TransformListener& tfl_;

	BFL::StatePosVel sys_sigma_;
	TrackerKalman filter_;

	string id_;
	string object_id;
	ros::Time time_;
	ros::Time meas_time_;

	double reliability, p, probability;

	Stamped<Point> position_;
	Stamped<Vector3> velocity_;
	SavedFeature* other;
	float dist_to_person_;


	// one leg tracker
	SavedFeature(Stamped<Point> loc, TransformListener& tfl)
	: tfl_(tfl),
	  sys_sigma_(Vector3(0.05, 0.05, 0.05), Vector3(1.0, 1.0, 1.0)),
	  filter_("tracker_legs",sys_sigma_),
	  reliability(-1.), p(4)
	{
		char id[100];
		snprintf(id,100,"legtrack %d", nextid++);
		id_ = std::string(id);

		object_id = "";
		time_ = loc.stamp_;
		meas_time_ = loc.stamp_;
		other = NULL;

		try {
			tfl_.transformPoint(fixed_frame, loc, loc);
		} catch(...) {
			ROS_WARN("TF exception spot 6.");
		}
		StampedTransform pose( Pose(Quaternion(0.0, 0.0, 0.0, 1.0), loc), loc.stamp_, id_, loc.frame_id_);
		tfl_.setTransform(pose);

		StatePosVel prior_sigma(Vector3(0.1,0.1,0.1), Vector3(0.0000001, 0.0000001, 0.0000001));
		filter_.initialize(loc, prior_sigma, time_.toSec());
		StatePosVel est;
		filter_.getEstimate(est);
		updatePosition();

	}

	void propagate(ros::Time time)
	{
		time_ = time;
		filter_.updatePrediction(time.toSec());
		updatePosition();
	}

	void update(Stamped<Point> loc, double probability)
	{
		StampedTransform pose( Pose(Quaternion(0.0, 0.0, 0.0, 1.0), loc), loc.stamp_, id_, loc.frame_id_);
		tfl_.setTransform(pose);

		meas_time_ = loc.stamp_;
		time_ = meas_time_;

		SymmetricMatrix cov(3);
		cov = 0.0;
		cov(1,1) = cov_meas_legs_m;
		cov(2,2) = cov_meas_legs_m;
		cov(3,3) = cov_meas_legs_m;

		filter_.updateCorrection(loc, cov);
		updatePosition();

		if(reliability<0 || !use_filter){
			reliability = probability;
			p = kal_p;
		}
		else{
			p += kal_q;
			double k = p / (p+kal_r);
			reliability += k * (probability - reliability);
			p *= (1 - k);
		}
	}

	double getLifetime()
	{
		return filter_.getLifetime();
	}

	double getReliability()
	{
		return reliability;
	}

private:
	void updatePosition()
	{
		StatePosVel est;
		filter_.getEstimate(est);

		position_[0] = est.pos_[0];
		position_[1] = est.pos_[1];
		position_[2] = est.pos_[2];

		velocity_[0] = est.vel_[0];
		velocity_[1] = est.vel_[1];
		velocity_[2] = est.vel_[2];

		position_.stamp_ = time_;
		position_.frame_id_ = fixed_frame;
		velocity_.stamp_ = time_;
		velocity_.frame_id_ = fixed_frame;

		double nreliability = fmin(1.0, fmax(0.1, est.vel_.length() / 0.5));
	}

};

int SavedFeature::nextid = 0;


class MatchedFeature
{
public:
	SampleSet* candidate_;
	SavedFeature* closest_;
	float distance_;
	double probability_;

	MatchedFeature(SampleSet* candidate, SavedFeature* closest, float distance, double probability)
	: candidate_(candidate)
	, closest_(closest)
	, distance_(distance)
	, probability_(probability)
	{}

	inline bool operator< (const MatchedFeature& b) const
	{
		return (distance_ <  b.distance_);
	}
};

int g_argc;
char** g_argv;




// actual legdetector node
class LegDetector
{
public:
	NodeHandle nh_;
	TransformListener tfl_;
	//TransformListener tflp_;
	ScanMask mask_;
	int mask_count_;
#if CV_MAJOR_VERSION == 2
	CvRTrees forest;
#else
	// OpenCV 3
	cv::Ptr<cv::ml::RTrees> forest;
#endif
	float connected_thresh_;
	int feat_count_;
	char save_[100];
	list<SavedFeature*> saved_features_;
	boost::mutex saved_mutex_;
	int feature_id_;
	bool use_seeds_;
	bool publish_legs_, publish_people_, publish_leg_markers_, publish_people_markers_, publish_vel_markers_;
	int next_p_id_;
	double leg_reliability_limit_;
	int min_points_per_group;

	ros::Publisher people_measurements_pub_;
	ros::Publisher leg_measurements_pub_;
	ros::Publisher people_pub_;
	ros::Publisher markers_pub_;

	dynamic_reconfigure::Server<cob_leg_detection::LegDetectionConfig> server_;

	message_filters::Subscriber<cob_perception_msgs::PositionMeasurementArray> people_sub_;
	message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
	//tf::MessageFilter<cob_perception_msgs::PositionMeasurementArray> people_notifier_;
	tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;

	//list<cob_perception_msgs::PositionMeasurement*>saved_people_;
	list<SavedPersonFeature*>saved_people_;

	tf::TransformBroadcaster br_;

	LegDetector(ros::NodeHandle nh) :
		nh_(nh),
		mask_count_(0),
		feat_count_(0),
		next_p_id_(0),
		laser_sub_(nh_,"scan",10),
		laser_notifier_(laser_sub_,tfl_,fixed_frame,10)
	{

		if (g_argc > 1) {
#if CV_MAJOR_VERSION == 2
			forest.load(g_argv[1]);
			feat_count_ = forest.get_active_var_mask()->cols;
#else
			// OpenCV 3
			forest = cv::ml::RTrees::create();
			cv::String feature_file = cv::String(g_argv[1]);
			forest = cv::ml::StatModel::load<cv::ml::RTrees>(feature_file);
			feat_count_ = forest->getVarCount();
#endif
			printf("Loaded forest with %d features: %s\n", feat_count_, g_argv[1]);
		} else {
			printf("Please provide a trained random forests classifier as an input.\n");
			shutdown();
		}

		// advertise topics
		leg_measurements_pub_ = nh_.advertise<cob_perception_msgs::PositionMeasurementArray>("leg_tracker_measurements",0);
		people_measurements_pub_ = nh_.advertise<cob_perception_msgs::PositionMeasurementArray>("people_tracker_measurements", 0);
		//people_pub_ = nh_.advertise<cob_perception_msgs::PositionMeasurementArray>("people",0);
		people_pub_ = nh_.advertise<cob_perception_msgs::People>("people",0);
		markers_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 20);

		laser_notifier_.registerCallback(boost::bind(&LegDetector::laserCallback, this, _1));
		laser_notifier_.setTolerance(ros::Duration(0.01));

		people_sub_.subscribe(nh_,"people_tracker_measurements",10);
		people_sub_.registerCallback(boost::bind(&LegDetector::peopleCallback, this, _1));

		dynamic_reconfigure::Server<cob_leg_detection::LegDetectionConfig>::CallbackType f;
		f = boost::bind(&LegDetector::configure, this, _1, _2);
		server_.setCallback(f);

		feature_id_ = 0;
	}

	~LegDetector()
	{
	}

	void configure(cob_leg_detection::LegDetectionConfig &config, uint32_t level)
	{
		connected_thresh_       = config.connection_threshold;
		min_points_per_group    = config.min_points_per_group;
		leg_reliability_limit_  = config.leg_reliability_limit;
		publish_legs_           = config.publish_legs;
		publish_people_         = config.publish_people;
		publish_leg_markers_    = config.publish_leg_markers;
		publish_vel_markers_    = config.publish_vel_markers;
		publish_people_markers_ = config.publish_people_markers;

		no_observation_timeout_s = config.no_observation_timeout;
		max_second_leg_age_s     = config.max_second_leg_age;
		max_track_jump_m         = config.max_track_jump;
		max_meas_jump_m          = config.max_meas_jump;
		leg_pair_separation_m    = config.leg_pair_separation;
		cov_meas_legs_m			 = config.cov_meas_legs;
		cov_meas_people_m        = config.cov_meas_people;

		if(fixed_frame.compare(config.fixed_frame) != 0){
			fixed_frame              = config.fixed_frame;
			laser_notifier_.setTargetFrame(fixed_frame);
			//people_notifier_.setTargetFrame(fixed_frame);
		}
		kal_p                    = config.kalman_p;
		kal_q                    = config.kalman_q;
		kal_r                    = config.kalman_r;
		use_filter               = config.kalman_on == 1;
	}


	/*
	 * keeps tracking the same person after their name
	 * with the help of a Kalman Filter. Determins the
	 * current position and velocity and speed of a tracked person.
	 */
	void peopleCallback(const cob_perception_msgs::PositionMeasurementArray::ConstPtr& people_meas)
	{
		//ROS_INFO("start people callback");
		if (people_meas->people.empty())
			return;

		boost::mutex::scoped_lock lock(saved_mutex_);
		list<SavedPersonFeature*>::iterator it;
		cob_perception_msgs::PositionMeasurement ppl;
		list<SavedPersonFeature*> saved_people;

		//if there are some people in memory list
		if(saved_people_.size() != 0){
			//predict distribution of error measurement values over the next time
			// by a known correct state from the previous time ->
			for(list<SavedPersonFeature*>::iterator iter_ = saved_people_.begin();
					iter_ != saved_people_.end(); ++iter_){
				(*iter_)->propagate(people_meas->header.stamp);
			}
		}

		//extracts the data from the comming message and saves new people in the list
		for(int i = 0; i < people_meas->people.size(); i++){
			ppl = people_meas->people.at(i);
			Stamped<Point> person_loc(Vector3(ppl.pos.x, ppl.pos.y, ppl.pos.z), people_meas->header.stamp, ppl.header.frame_id);
			//if the list is empty, add the person to the list
			if(saved_people.size() == 0){
				saved_people.insert(saved_people.end(), new SavedPersonFeature(person_loc, ppl.object_id, ppl.name));
			}else{
				bool found = false;
				//if the list is not empty, check for a person name in the list
				for (it = saved_people.begin(); it != saved_people.end(); ++it){
					if(ppl.name.compare((*it)->person_name) == 0 )
						found = true;
				}
				//if there is no same name in the list, add the person to the list
				if(!found){
					found = false;
					saved_people.insert(saved_people.end(), new SavedPersonFeature(person_loc, ppl.object_id, ppl.name));
				}
			}
		}

		// Compare two lists:
		// saved_people_ - global list over the time with current detected people
		// saved_people  - temporary list within this message with the current detected people

		//if the memory list of people is empty, put all new people on the list
		// same people are excluded
		if(saved_people_.size() == 0){
			for(list<SavedPersonFeature*>::iterator iter = saved_people.begin(); iter != saved_people.end(); ++iter){
				saved_people_.push_back((*iter));
			}
		}else{ // if the memory list is not empty, check the list for the same people to update there values
			for(list<SavedPersonFeature*>::iterator iter = saved_people.begin(); iter != saved_people.end(); ++iter)
			{
				bool found_temp = false;
				for(list<SavedPersonFeature*>::iterator iter_ = saved_people_.begin(); iter_ != saved_people_.end(); ++iter_){
					if((*iter)->person_name.compare((*iter_)->person_name) == 0){
						found_temp = true;
						// update distribution over current state of values
						//by known prediction of state and next measurements:
						(*iter_)->update((*iter)->position_, 1.0);
					}
				}

				if(found_temp == false ){
					saved_people_.push_back((*iter));
				}
			}
		}

		//erase unnecessary tracks on people if they are called the same
		for(list<SavedPersonFeature*>::iterator iter = saved_people_.begin(); iter != saved_people_.end(); ++iter){
			bool found_temp = false;
			for(list<SavedPersonFeature*>::iterator iter_ = saved_people.begin(); iter_ != saved_people.end(); ++iter_){
				if((*iter)->person_name.compare((*iter_)->person_name) == 0){
					found_temp = true;
				}
			}
			if(found_temp == false ){
				delete (*iter);
				saved_people_.erase(iter++);
			}
		}


		PRINT_LIST(saved_people_, "LIST saved_people ::  ");
		//publish data

		int i = 0;

		vector<cob_perception_msgs::Person> people;

		for (list<SavedPersonFeature*>::iterator sp_iter = saved_people_.begin();
				sp_iter != saved_people_.end(); sp_iter++,i++){
			//ROS_INFO("Velocity [%f, %f, %f]}: ", (*sp_iter)->velocity_[0], (*sp_iter)->velocity_[1], (*sp_iter)->velocity_[2]);
			cob_perception_msgs::Person person;
			person.detector = detector_;
			person.name = (*sp_iter)->person_name; // name of the person
			person.position.position.x = (*sp_iter)->position_[0];
			person.position.position.y = (*sp_iter)->position_[1];
			person.position.position.z = (*sp_iter)->position_[2];

			person.velocity.x = (*sp_iter)->velocity_[0];
			person.velocity.y = (*sp_iter)->velocity_[1];
			person.velocity.z = (*sp_iter)->velocity_[2];

			people.push_back(person);

			double dx = (*sp_iter)->velocity_[0], dy = (*sp_iter)->velocity_[1];
			visualization_msgs::Marker m;
			m.header.stamp = people_meas->header.stamp;
			m.header.frame_id = fixed_frame;
			m.ns = "SPEED";
			m.type = m.ARROW;
			m.pose.position.x = (*sp_iter)->position_[0];
			m.pose.position.y = (*sp_iter)->position_[1];
			m.pose.position.z = (*sp_iter)->position_[2];
			m.pose.orientation.x = (*sp_iter)->velocity_[0];
			m.pose.orientation.y = (*sp_iter)->velocity_[1];
			m.pose.orientation.z = 0.0;
			m.scale.x = sqrt(dx*dx+dy*dy);
			ROS_INFO("speed %f", m.scale.x);
			//m.scale.x = .4;
			m.scale.y = .05;
			m.scale.z = .05;
			m.color.a = 1;
			m.color.r = 1;
			m.lifetime = ros::Duration(0.5);

			markers_pub_.publish(m);
		}
		//cob_perception_msgs::PositionMeasurementArray array;
		cob_perception_msgs::People array;
		array.header.stamp = ros::Time::now();
		array.people = people;
		people_pub_.publish(array);
	}


	//callback for laser data

	void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
	{
		ScanProcessor processor(*scan, mask_);
		processor.splitConnected(connected_thresh_);
		processor.removeLessThan(5);
#if CV_MAJOR_VERSION == 2
		CvMat* tmp_mat = cvCreateMat(1,feat_count_,CV_32FC1);
#else
// OpenCV 3
		cv::Mat tmp_mat = cv::Mat(1, feat_count_, CV_32FC1);
#endif

		// if no measurement matches to a tracker in the last <no_observation_timeout>  seconds: erase tracker
		ros::Time purge = scan->header.stamp + ros::Duration().fromSec(-no_observation_timeout_s);
		list<SavedFeature*>::iterator sf_iter = saved_features_.begin();
		while (sf_iter != saved_features_.end()){
			if ((*sf_iter)->meas_time_ < purge){
				if( (*sf_iter)->other )
					(*sf_iter)->other->other = NULL;
				delete (*sf_iter);
				saved_features_.erase(sf_iter++);
			}else
				++sf_iter;
		}


		// System update of trackers, and copy updated ones in propagate list
		list<SavedFeature*> propagated;
		for (list<SavedFeature*>::iterator sf_iter = saved_features_.begin();
				sf_iter != saved_features_.end();
				sf_iter++){
			(*sf_iter)->propagate(scan->header.stamp);
			propagated.push_back(*sf_iter);
		}

		// Detection step: build up the set of "candidate" clusters
		// For each candidate, find the closest tracker (within threshold) and add to the match list
		// If no tracker is found, start a new one
		multiset<MatchedFeature> matches;
		for (list<SampleSet*>::iterator i = processor.getClusters().begin();
				i != processor.getClusters().end();i++){
			vector<float> f = calcLegFeatures(*i, *scan);

			for (int k = 0; k < feat_count_; k++)
#if CV_MAJOR_VERSION == 2
				tmp_mat->data.fl[k] = (float)(f[k]);
#else
// OpenCV 3
				tmp_mat.data[k] = (float)(f[k]);
#endif

#if CV_MAJOR_VERSION == 2
			float probability = forest.predict_prob( tmp_mat );
#else
// OpenCV 3
			// Probability is the fuzzy measure of the probability that the second element should be chosen,
			// in opencv2 RTrees had a method predict_prob, but that disapeared in opencv3, this is the
			// substitute.
			float probability = 0.5 -
				forest->predict(tmp_mat, cv::noArray(), cv::ml::RTrees::PREDICT_SUM) /
				forest->getRoots().size();
#endif
			Stamped<Point> loc((*i)->center(), scan->header.stamp, scan->header.frame_id);
			try {
				tfl_.transformPoint(fixed_frame, loc, loc);
			} catch(...) {
				ROS_WARN("TF exception spot 3.");
			}

			list<SavedFeature*>::iterator closest = propagated.end();
			float closest_dist = max_track_jump_m;

			for (list<SavedFeature*>::iterator pf_iter = propagated.begin();
					pf_iter != propagated.end();pf_iter++){
				// find the closest distance between candidate and trackers
				float dist = loc.distance((*pf_iter)->position_);
				if ( dist < closest_dist ){
					closest = pf_iter;
					closest_dist = dist;
				}
			}
			// Nothing close to it, start a new track
			if (closest == propagated.end()){
				list<SavedFeature*>::iterator new_saved = saved_features_.insert(saved_features_.end(), new SavedFeature(loc, tfl_));
			}else // Add the candidate, the tracker and the distance to a match list
				matches.insert(MatchedFeature(*i,*closest,closest_dist,probability));
		}

		// loop through _sorted_ matches list
		// find the match with the shortest distance for each tracker
		while (matches.size() > 0){
			multiset<MatchedFeature>::iterator matched_iter = matches.begin();
			bool found = false;
			list<SavedFeature*>::iterator pf_iter = propagated.begin();
			while (pf_iter != propagated.end()){
				// update the tracker with this candidate
				if (matched_iter->closest_ == *pf_iter){
					// Transform candidate to fixed frame
					Stamped<Point> loc(matched_iter->candidate_->center(), scan->header.stamp, scan->header.frame_id);
					try {
						tfl_.transformPoint(fixed_frame, loc, loc);
					} catch(...) {
						ROS_WARN("TF exception spot 4.");
					}

					// Update the tracker with the candidate location
					matched_iter->closest_->update(loc, matched_iter->probability_);

					// remove this match and
					matches.erase(matched_iter);
					propagated.erase(pf_iter++);
					found = true;
					break;
				}else{  // still looking for the tracker to update
					pf_iter++;
				}
			}

			// didn't find tracker to update, because it was deleted above
			// try to assign the candidate to another tracker
			if (!found){
				Stamped<Point> loc(matched_iter->candidate_->center(), scan->header.stamp, scan->header.frame_id);
				try {
					tfl_.transformPoint(fixed_frame, loc, loc);
				} catch(...) {
					ROS_WARN("TF exception spot 5.");
				}
				list<SavedFeature*>::iterator closest = propagated.end();
				float closest_dist = max_track_jump_m;

				for (list<SavedFeature*>::iterator remain_iter = propagated.begin();
						remain_iter != propagated.end();remain_iter++){
					float dist = loc.distance((*remain_iter)->position_);
					if ( dist < closest_dist ){
						closest = remain_iter;
						closest_dist = dist;
					}
				}

				// no tracker is within a threshold of this candidate
				// so create a new tracker for this candidate
				if (closest == propagated.end())
					list<SavedFeature*>::iterator new_saved = saved_features_.insert(saved_features_.end(), new SavedFeature(loc, tfl_));
				else
					matches.insert(MatchedFeature(matched_iter->candidate_,*closest,closest_dist, matched_iter->probability_));
				matches.erase(matched_iter);
			}
		}

#if CV_MAJOR_VERSION == 2
		cvReleaseMat(&tmp_mat); tmp_mat = 0;
#endif
		// if(!use_seeds_)
		pairLegs();

		// Publish Data!

		int i = 0;
		vector<cob_perception_msgs::PositionMeasurement> people;
		vector<cob_perception_msgs::PositionMeasurement> legs;

		for (list<SavedFeature*>::iterator sf_iter = saved_features_.begin();
				sf_iter != saved_features_.end(); sf_iter++,i++){
			// reliability
			double reliability = (*sf_iter)->getReliability();
			//  ROS_INFO("reliability %f", reliability);

			if ((*sf_iter)->getReliability() > leg_reliability_limit_
					&& publish_legs_){

				cob_perception_msgs::PositionMeasurement pos;
				pos.header.stamp = scan->header.stamp;
				pos.header.frame_id = fixed_frame;
				pos.name = "leg_detection";
				pos.object_id = (*sf_iter)->id_;
				pos.pos.x = (*sf_iter)->position_[0];
				pos.pos.y = (*sf_iter)->position_[1];
				pos.pos.z = (*sf_iter)->position_[2];
				pos.vel.x = (*sf_iter)->velocity_[0];
				pos.vel.y = (*sf_iter)->velocity_[1];
				pos.vel.z = (*sf_iter)->velocity_[2];
				pos.reliability = reliability;
				pos.covariance[0] = pow(0.3 / reliability,2.0);
				pos.covariance[1] = 0.0;
				pos.covariance[2] = 0.0;
				pos.covariance[3] = 0.0;
				pos.covariance[4] = pow(0.3 / reliability,2.0);
				pos.covariance[5] = 0.0;
				pos.covariance[6] = 0.0;
				pos.covariance[7] = 0.0;
				pos.covariance[8] = 10000.0;
				pos.initialization = 0;
				legs.push_back(pos);
			}

			if (publish_leg_markers_){
				visualization_msgs::Marker m;
				m.header.stamp = (*sf_iter)->time_;
				m.header.frame_id = fixed_frame;
				m.ns = "LEGS";
				m.id = i;
				m.type = m.SPHERE;
				m.pose.position.x = (*sf_iter)->position_[0];
				m.pose.position.y = (*sf_iter)->position_[1];
				m.pose.position.z = (*sf_iter)->position_[2];

				m.scale.x = .1;
				m.scale.y = .1;
				m.scale.z = .1;
				m.color.a = 1;
				m.lifetime = ros::Duration(0.5);
				if((*sf_iter)->object_id != ""){
					m.color.r = 1;
				}else{
					m.color.b = (*sf_iter)->getReliability();
				}
				markers_pub_.publish(m);
			}

			if (publish_people_ || publish_people_markers_ ){
				SavedFeature* other = (*sf_iter)->other;
				if(other!=NULL && other<(*sf_iter)){

					Stamped<Point> one = (*sf_iter)->position_, two = (other)->position_;
					double ddx = one[0]-two[0], ddy = one[1]-two[1], ddz = one[2]-two[2];
					double d =  sqrt(ddx*ddx + ddy*ddy + ddz*ddz);
					//ROS_INFO("Person %s with distance %f",  (*sf_iter)->object_id.c_str() , d);

					double dx = ((*sf_iter)->position_[0] + other->position_[0])/2,
							dy = ((*sf_iter)->position_[1] + other->position_[1])/2,
							dz = ((*sf_iter)->position_[2] + other->position_[2])/2;

					double vx = ((*sf_iter)->velocity_[0]- ((*sf_iter)->velocity_[0] - other->velocity_[0])/2),
							vy = ((*sf_iter)->velocity_[1]- ((*sf_iter)->velocity_[1] - other->velocity_[1])/2),
							vz = ((*sf_iter)->velocity_[2]- ((*sf_iter)->velocity_[2] - other->velocity_[2])/2);

					double speed = sqrt(vx*vx + vy*vy + vz*vz);
					//ROS_INFO("speed %f: ", speed );

					if (publish_people_ ){

						reliability = reliability * other->reliability;
						cob_perception_msgs::PositionMeasurement pos;
						pos.header.stamp = (*sf_iter)->time_;
						pos.header.frame_id = fixed_frame;
						pos.name = (*sf_iter)->object_id;
						pos.object_id = (*sf_iter)->id_ + "|" + other->id_;
						pos.pos.x = dx;
						pos.pos.y = dy;
						pos.pos.z = dz;
						pos.vel.x = vx;
						pos.vel.y = vy;
						pos.vel.z = vz;
						pos.reliability = reliability;
						pos.covariance[0] = pow(0.3 / reliability,2.0);
						pos.covariance[1] = 0.0;
						pos.covariance[2] = 0.0;
						pos.covariance[3] = 0.0;
						pos.covariance[4] = pow(0.3 / reliability,2.0);
						pos.covariance[5] = 0.0;
						pos.covariance[6] = 0.0;
						pos.covariance[7] = 0.0;
						pos.covariance[8] = 10000.0;
						pos.initialization = 0;

						people.push_back(pos);

						ros::Time time = ros::Time::now();
						tf::Transform person(tf::Quaternion(0,0,0,1), tf::Vector3(dx, dy, dz));
						try
						{
							br_.sendTransform(tf::StampedTransform(person, time,
									"/base_link" , pos.name.c_str()));
						}catch (tf::TransformException ex){
							ROS_ERROR("Broadcaster unavailable %s", ex.what());
						}
					}

					if (publish_people_markers_ ){
						visualization_msgs::Marker m;
						m.header.stamp = (*sf_iter)->time_;
						m.header.frame_id = fixed_frame;
						m.ns = "PEOPLE";
						m.id = i;
						m.type = m.SPHERE;
						m.pose.position.x = dx;
						m.pose.position.y = dy;
						m.pose.position.z = dz;
						m.scale.x = .2;
						m.scale.y = .2;
						m.scale.z = .2;
						m.color.a = 1;
						m.color.g = 1;
						m.lifetime = ros::Duration(0.5);
						markers_pub_.publish(m);
					}

					if(publish_people_markers_ ){
						visualization_msgs::Marker m;
						m.header.stamp = (*sf_iter)->time_;
						m.header.frame_id = fixed_frame;
						m.ns = "TEXT";
						m.id = i;
						m.type = m.TEXT_VIEW_FACING;
						m.pose.position.x = dx;
						m.pose.position.y = dy;
						m.pose.position.z = dz;
						m.pose.orientation.w = 1.0;
						m.text = (*sf_iter)->object_id.c_str();
						m.scale.z = 0.3;
						m.color.a = 1;
						m.color.r = 0.5;
						m.color.g = 0.5;
						m.lifetime = ros::Duration(0.5);
						markers_pub_.publish(m);
					}
					/*
						if (publish_people_markers_ ){
							visualization_msgs::Marker m;
							m.header.stamp = (*sf_iter)->time_;
							m.header.frame_id = fixed_frame;
							m.ns = "SPEED";
							m.id = i;
							m.type = m.ARROW;
							m.pose.position.x = dx;
							m.pose.position.y = dy;
							m.pose.position.z = dz;
							m.pose.orientation.x = vx;
							m.pose.orientation.y = vy;
							m.pose.orientation.z = vz;

							m.scale.x = .4;
							m.scale.y = .05;
							m.scale.z = .05;
							m.color.a = 1;
							m.color.r = 1;
							m.lifetime = ros::Duration(0.5);

							markers_pub_.publish(m);
						}
					 */
				}
			}
		}
		cob_perception_msgs::PositionMeasurementArray array;
		array.header.stamp = ros::Time::now();
		array.header.frame_id = fixed_frame;

		if(publish_legs_){
			array.people = legs;
			leg_measurements_pub_.publish(array);
		}

		if(publish_people_){
			array.people = people;
			people_measurements_pub_.publish(array);
		}
	}


	double distance( list<SavedFeature*>::iterator it1,  list<SavedFeature*>::iterator it2)
	{
		Stamped<Point> one = (*it1)->position_, two = (*it2)->position_;
		double dx = one[0]-two[0], dy = one[1]-two[1], dz = one[2]-two[2];
		return sqrt(dx*dx+dy*dy+dz*dz);
	}


	inline void PRINT_LIST	(list<SavedPersonFeature*>& coll, const char* optcstr="")
	{
		std::cout << optcstr;
		for (	list<SavedPersonFeature*>::iterator it =coll.begin();it !=coll.end(); ++it) {
			std::cout << (*it)->person_name << ' ';
		}
		std::cout << std::endl;
	}


	void pairLegs()
	{
		// Deal With legs that already have ids
		list<SavedFeature*>::iterator begin = saved_features_.begin();
		list<SavedFeature*>::iterator end = saved_features_.end();
		list<SavedFeature*>::iterator leg1, leg2, best, it;

		for (leg1 = begin; leg1 != end; ++leg1){
			// If this leg has no id, skip
			if ((*leg1)->object_id == "")
				continue;

			leg2 = end;
			best = end;
			// ROS_INFO("leg pair separation %f", leg_pair_separation_m);

			double closest_dist = leg_pair_separation_m;
			for ( it = begin; it != end; ++it){
				if(it==leg1) continue;

				if ( (*it)->object_id == (*leg1)->object_id ) {
					leg2 = it;
					break;
				}

				if ((*it)->object_id != "")
					continue;

				double d = distance(it, leg1);
				if (((*it)->getLifetime() <= max_second_leg_age_s)
						&& (d < closest_dist)){
					closest_dist = d;
					best = it;
				}
			}

			if(leg2 != end){
				double dist_between_legs = distance(leg1, leg2);
				if (dist_between_legs > leg_pair_separation_m){
					(*leg1)->object_id = "";
					(*leg1)->other = NULL;
					(*leg2)->object_id = "";
					(*leg2)->other = NULL;
				}else{
					(*leg1)->other = *leg2;
					(*leg2)->other = *leg1;
				}
			}else if(best != end){
				(*best)->object_id = (*leg1)->object_id;
				(*leg1)->other = *best;
				(*best)->other = *leg1;

			}
		}

		// Attempt to pair up legs with no id
		for(;;){
			list<SavedFeature*>::iterator best1 = end, best2 = end;
			double closest_dist = leg_pair_separation_m;

			for (leg1 = begin; leg1 != end; ++leg1){
				// If this leg has an id or low reliability, skip
				if ((*leg1)->object_id != ""
						|| (*leg1)->getReliability() < leg_reliability_limit_)
					continue;

				for ( leg2 = begin; leg2 != end; ++leg2){
					if(((*leg2)->object_id != "")
							|| ((*leg2)->getReliability() < leg_reliability_limit_)
							|| (leg1==leg2)) continue;

					double d = distance(leg1, leg2);

					if(d < closest_dist){
						best1 = leg1;
						best2 = leg2;
					}
				}
			}

			if(best1 != end){
				char id[100];
				float number = next_p_id_;
				snprintf(id,100,"Person%d", next_p_id_++);
				(*best1)->object_id = std::string(id);
				(*best2)->object_id = std::string(id);
				(*best1)->other = *best2;
				(*best2)->other = *best1;
			}else{
				break;
			}
		}
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv,"leg_detection");
	g_argc = argc;
	g_argv = argv;
	ros::NodeHandle nh;
	LegDetector ld(nh);
	ROS_INFO("Execute main");
	ros::spin();

	return 0;
}
