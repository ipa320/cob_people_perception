#ifndef PEOPLE_DUAL_PEOPLE_LEG_TRACKER_SRC_MHT_KALMANFILTER_H_
#define PEOPLE_DUAL_PEOPLE_LEG_TRACKER_SRC_MHT_KALMANFILTER_H_

// Eigen includes
#include <Eigen/Dense>

// ROS includes
#include <ros/ros.h>

namespace filter{

class KalmanFilter {
public:
	Eigen::Matrix<double,4,4> A_; // Transition Matrix

	Eigen::Matrix<double,4,4> A_dt; // Changes with dt

	Eigen::Matrix<double,4,4> H_; // Measurement matrix

	Eigen::Matrix<double,4,4> P_prior_; // Predicted (a priori) estimate covariance

	Eigen::Matrix<double,4,4> P_post_; // Updated (a posteriori) estimate covariance

	Eigen::Matrix<double,4,4> Q_; // Process Covariance

	Eigen::Matrix<double,4,4> R_; // Measurement Covariance

	Eigen::Matrix<double,4,4> S_k_; // Residual Covariance

	Eigen::Matrix<double,4,4> S_k_temp_; // For the measurement likelihood

	Eigen::Matrix<double,4,1> initial_state_; // The current State

	Eigen::Matrix<double,4,1> state_predicted_; // The current State

	Eigen::Matrix<double,4,1> state_estimated_; // The state estimation

	Eigen::Matrix<double,4,1> measurement_prediction_; // Current measurement prediction

	ros::Time filterTime_;

public:
	KalmanFilter(Eigen::Matrix<double,4,1> initialState, ros::Time time);
	//KalmanFilter(const KalmanFilter &obj);
	virtual ~KalmanFilter();

	// Do the prediction
	void predict(double dt);

  // Do the prediction
  void predict(ros::Time time);

	// Get the prediction
	Eigen::Matrix<double,4,1> getPrediction();

	// Get the prediction
	Eigen::Matrix<double,4,1> getEstimation();

	// Get the measurement prediction
	Eigen::Matrix<double,4,1> getMeasurementPrediction();

	// Do a update
	void update(Eigen::Matrix<double,4,1>);

	// Get measurement likelihood
  // double getMeasurementLikelihood(Eigen::Vector2d meas);

};
}
#endif /* PEOPLE_DUAL_PEOPLE_LEG_TRACKER_SRC_MHT_KALMANFILTER_H_ */
