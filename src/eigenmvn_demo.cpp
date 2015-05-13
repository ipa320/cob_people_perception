#include <fstream>
#include <iostream>
#include <dual_people_leg_tracker/eigenmvn/eigenmvn.h>
#ifndef M_PI
#define M_PI REAL(3.1415926535897932384626433832795029)
#endif

/**
  Take a pair of un-correlated variances.
  Create a covariance matrix by correlating
  them, sandwiching them in a rotation matrix.
*/
Eigen::Matrix2d genCovar(double v0,double v1,double theta)
{
  Eigen::Matrix2d rot = Eigen::Rotation2Dd(theta).matrix();
  return rot*Eigen::DiagonalMatrix<double,2,2>(v0,v1)*rot.transpose();
}

int main()
{
  Eigen::Vector2d mean;
  Eigen::Matrix2d covar;

  Eigen::Matrix2d q;

  Eigen::Vector2d v1;
  Eigen::Vector2d v2;

  Eigen::Matrix2d diag;

  v1 << 10.0, 30.0;
  v2 << 1, -1.0/3;

  double d1 = v1.norm();
  double d2 = v2.norm();

  v1.normalize();
  v2.normalize();

  mean << 0,0; // Set the mean
  // Create a covariance matrix
  // Much wider than it is tall
  // and rotated clockwise by a bit
  //covar = genCovar(3,0.1,M_PI/5.0);
  q.col(0) = v1;
  q.col(1) = v2;

  diag << d1, 0, 0, d2;

  covar = q * diag * q.transpose();

  // Create a bivariate gaussian distribution of doubles.
  // with our chosen mean and covariance
  const int dim = 2;
  Eigen::EigenMultivariateNormal<double> normX_solver(mean,covar);
  std::ofstream file_solver("samples_solver.txt");

  // Generate some samples and write them out to file
  // for plotting
  file_solver << normX_solver.samples(5000).transpose() << std::endl;

  // same for Cholesky decomposition.
  //covar = genCovar(3,0.1,M_PI/5.0);
  Eigen::EigenMultivariateNormal<double> normX_cholesk(mean,covar,true);
  std::ofstream file_cholesky("samples_cholesky.txt");
  file_cholesky << normX_cholesk.samples(5000).transpose() << std::endl;
}
