#include <iostream>
#include <iomanip>
#include <Eigen/Dense>

#include <dual_people_leg_tracker/mht/KalmanFilter.h>

int main(int argc, char **argv)
{

	// Time setttings
	double dt = 0.08; // Timestep
	double duration = 1;

	// System settings
	Eigen::Matrix<double,4,4> A_; // Transition Matrix

	Eigen::Matrix<double,2,4> H_; // Measurement
	A_ = Eigen::Matrix<double,-1,-1>::Identity(4,4);
	A_(0,2) = dt;
	A_(1,3) = dt;

	std::cout << "A_" << A_ << std::endl;

	Eigen::Matrix<double,4,1> x; // Transition Matrix
	x << 0,0,0.1,0;

	Eigen::Matrix<double,2,1> z; // Measurement

	// Define the measurement Matrix
	H_ << 1, 0, 0, 0,
		  0, 1, 0,  0;
	z = H_*x;

	// The Kalman Filter
	mht::KalmanFilter kalmanFilter(z);

	for(double t=0; t<duration; t=t+dt){
		std::cout << "Time       " << t << std::endl;
		std::cout << "State      " << std::setw(3) << x.transpose() << std::endl;
		std::cout << "Measurement" << std::setw(3) << z.transpose() << std::endl;
		std::cout << "Kalman Prediction " << kalmanFilter.getPrediction().transpose() << std::endl;
		std::cout << "Kalman Estimation " << kalmanFilter.getEstimation().transpose() << std::endl;

		std::cout << "Real state " << x.transpose() << std::endl;
		// Calculate the Mahalanobis distance
		Eigen::Matrix<double,2,1> estimated_pos;
		Eigen::Matrix<double,2,1> diff;
		estimated_pos = kalmanFilter.getEstimation().block(0,0,2,1);

		diff = estimated_pos - z;
		double value = diff.transpose()*kalmanFilter.S_k_.inverse()*diff;

		std::cout << "Mahalanobis Distance " << sqrt(value) << std::endl;

		std::cout << "Residual " << (kalmanFilter.getEstimation() - x).norm() << std::endl << "#########################" << std::endl;

		x = A_*x;
		z = H_*x;


		kalmanFilter.predict(dt);
		kalmanFilter.update(z);

	}



}
