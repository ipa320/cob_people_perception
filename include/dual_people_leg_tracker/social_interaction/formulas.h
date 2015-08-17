#ifndef PEOPLE_DUAL_PEOPLE_LEG_TRACKER_SOCIAL_INTERACTION_FORMULAS_H_
#define PEOPLE_DUAL_PEOPLE_LEG_TRACKER_SOCIAL_INTERACTION_FORMULAS_H_


/***
 * Interaction Energy between two persons
 * @param p_i       Own position
 * @param p_j       Position of other person
 * @param v_tilde_i Next desired velocity
 * @param v_j other Persons velocity
 * @param sigma_d   Controls the distance to the other person to be avoided
 * @return          Energy
 */
inline double E_ij(Eigen::Vector2d p_i, Eigen::Vector2d p_j, Eigen::Vector2d v_tilde_i, Eigen::Vector2d v_j, double sigma_d){

  Eigen::Vector2d k,q;
  k = p_i - p_j;
  q = v_tilde_i - v_j;

  double d_square = (k-((k.dot(q))/q.squaredNorm()) * q).squaredNorm();

  double energy = exp(- (d_square / (2 * pow(sigma_d,2))));

  return energy;
}

inline Eigen::Vector2d E_ij_gradient(Eigen::Vector2d p_i, Eigen::Vector2d p_j, Eigen::Vector2d v_tilde_i, Eigen::Vector2d v_j, double sigma_d){

  Eigen::Vector2d k,q;
  k = p_i - p_j;
  q = v_tilde_i - v_j;

  double k_x = k[0];
  double k_y = k[1];
  double q_x = q[0];
  double q_y = q[1];

  double nominator = pow(q.squaredNorm(),2);

  double beta = (k_x * q_x + k_y * q_y)/q.squaredNorm();

  double beta_dx = (3 * k_x * pow(q_x,2) + k_x * pow(q_y,2) + 2*k_y*q_x*q_y)/nominator;
  double beta_dy = (3 * k_y * pow(q_y,2) + k_y * pow(q_x,2) + 2*k_x*q_x*q_y)/nominator;

  double d_dx = 2 * (k_x - beta * q_x) * ( -beta_dx * q_x - beta) + 2 * (k_y - beta * q_y) * (-beta_dx) * q_y;
  double d_dy = 2 * (k_x - beta * q_x) * (-beta_dy) * q_x         + 2 * (k_y - beta * q_y) * (-beta_dy * q_y - beta);

  double energyHere = E_ij(p_i, p_j, v_tilde_i, v_j, sigma_d);

  double E_dx = (-(1.0/(2 * pow(sigma_d,2)))) * energyHere * d_dx;
  double E_dy = (-(1.0/(2 * pow(sigma_d,2)))) * energyHere * d_dy;

  Eigen::Vector2d e_ij_gradient;
  e_ij_gradient[0] = E_dx;
  e_ij_gradient[1] = E_dy;

  return e_ij_gradient;
}

inline double S_i(double const u_i, Eigen::Vector2d v_tilde_i){
  double S_i = 0;

  double v_i_tilde_norm = v_tilde_i.norm();

  S_i = pow(u_i - v_i_tilde_norm,2);

  return S_i;
}

inline Eigen::Vector2d S_i_gradient(double u_i, Eigen::Vector2d v_tilde_i){

  Eigen::Vector2d s_i_gradient;

  double v_i_tilde_norm = v_tilde_i.norm();

  s_i_gradient(0) = 2*v_tilde_i(0) - (2*u_i*v_tilde_i(0))/v_i_tilde_norm;
  s_i_gradient(1) = 2*v_tilde_i(1) - (2*u_i*v_tilde_i(1))/v_i_tilde_norm;

  return s_i_gradient;
}

inline double D_i(Eigen::Vector2d z_i, Eigen::Vector2d p_i, Eigen::Vector2d v_tilde_i){
  double D_i = 0;

  double nominator   = (z_i - p_i).dot(v_tilde_i);
  double denominator = (z_i - p_i).norm() * v_tilde_i.norm();

  D_i = -nominator / denominator;

  return D_i;
}

inline Eigen::Vector2d D_i_gradient(Eigen::Vector2d z_i, Eigen::Vector2d p_i, Eigen::Vector2d v_tilde_i){

  Eigen::Vector2d d_i_gradient;

  double factor = 1.0 / ( (z_i - p_i).norm() * pow(v_tilde_i.norm(),3) );

  double v_i_tilde_norm = v_tilde_i.norm();

  d_i_gradient(0) = factor * v_tilde_i(1) * (( (z_i(1) - p_i(1)) * v_tilde_i(0) ) - (z_i(0) - p_i(0)) * v_tilde_i(1));
  d_i_gradient(1) = factor * v_tilde_i(0) * (( (z_i(0) - p_i(0)) * v_tilde_i(1) ) - (z_i(1) - p_i(1)) * v_tilde_i(0));

  return d_i_gradient;
}

// Weighting factors
inline double w_r_d(Eigen::Vector2d p_i, Eigen::Vector2d p_j, double sigma_w){

  double w_r_d;
  Eigen::Vector2d k;
  k = p_i - p_j;

  w_r_d = exp(-(k.squaredNorm())/(2 * pow(sigma_w,2)));

  assert(w_r_d >= 0);
  assert(!isnan(w_r_d));
  return w_r_d;
}

// Weighting factors
inline double w_r_phi(Eigen::Vector2d p_i, // Own position
                      Eigen::Vector2d p_j, // Other persons position
                      Eigen::Vector2d v_i, // My current velocity
                      double beta          // Peakiness of the function
                      )
{

  double w_r_phi;
  Eigen::Vector2d vecToOther = p_j - p_i; // Vector from self to other person

  // Calculate the angular displacement
  double cos_phi = v_i.dot(vecToOther)/(v_i.norm() * vecToOther.norm());
  w_r_phi = pow(((1.0 + cos_phi)/2),beta);

  if(abs(acos(cos_phi)) > M_PI/2.0){
    w_r_phi = 0;
  }


  assert(w_r_phi >= 0);
  assert(!isnan(w_r_phi));
  return w_r_phi;
}

inline double w_r(Eigen::Vector2d p_i, Eigen::Vector2d p_j, Eigen::Vector2d v_i, double sigma_w, double beta){

  double w_r;
  w_r = w_r_d(p_i, p_j, sigma_w) * w_r_phi(p_i,p_j, v_i, beta);

  //std::cout << "w_r: " << w_r << std::endl;
  return w_r;
}

// Overall interaction energy (eq. 10)
inline double I_i(std::vector<Eigen::Vector2d> positionsOthers,
                  std::vector<Eigen::Vector2d> velocityOthers,
                  Eigen::Vector2d v_tilde_i,
                  Eigen::Vector2d p_i,
                  Eigen::Vector2d v_i,
                  double sigma_d,
                  double sigma_w,
                  double beta){

  double i_i = 0;


  size_t nOther = positionsOthers.size();
  //std::cout << "nOther: " << nOther << std::endl;

  for(size_t i = 0; i < nOther; i++){
    Eigen::Vector2d p_j, v_j;
    p_j = positionsOthers[i];
    v_j = velocityOthers[i];
    //std::cout << "p_i: " << p_i.transpose() << std::endl;
    //std::cout << "v_i: " << v_i.transpose() << std::endl;
    //std::cout << "p_j: " << p_j.transpose() << std::endl;
    //std::cout << "v_j: " << v_j.transpose() << std::endl;
    //std::cout << "sigma_w: " << sigma_w << std::endl;
    //std::cout << "beta: " << beta << std::endl;

    double w_r_temp = w_r(p_i, p_j, v_i, sigma_w, beta);
    double e_temp = E_ij(p_i, p_j, v_tilde_i, v_j, sigma_d);

    i_i += w_r_temp * e_temp;
    //std::cout << "w_r_temp " << w_r_temp << std::endl;
    //std::cout << "e_temp " << e_temp << std::endl;
    //std::cout << "i_i " << i_i << std::endl;
  }

  return i_i;


}

inline Eigen::Vector2d I_i_gradient(std::vector<Eigen::Vector2d> positionsOthers,
                                    std::vector<Eigen::Vector2d> velocityOthers,
                                    Eigen::Vector2d v_tilde_i,
                                    Eigen::Vector2d p_i,
                                    Eigen::Vector2d v_i,
                                    double sigma_d,
                                    double sigma_w,
                                    double beta){

  Eigen::Vector2d i_i_gradient;
  i_i_gradient.setZero();

  size_t nOther = positionsOthers.size();

  for(size_t i = 0; i < nOther; i++){
    Eigen::Vector2d p_j, v_j;
    p_j = positionsOthers[i];
    v_j = velocityOthers[i];

    // Calculating the weighting factors
    i_i_gradient += w_r(p_i, p_j, v_i, sigma_w, beta) * E_ij_gradient(p_i, p_j, v_tilde_i, v_j, sigma_d);
  }

  return i_i_gradient;

}

// Total energy (eg 13)
double E_i(std::vector<Eigen::Vector2d> positionsOthers,
           std::vector<Eigen::Vector2d> velocityOthers,
           Eigen::Vector2d v_tilde_i,
           Eigen::Vector2d p_i,
           Eigen::Vector2d v_i,
           double u_i, // Desired speed
           Eigen::Vector2d z_i, // Goal
           double sigma_d,
           double sigma_w,
           double beta,
           double lambda_0,
           double lambda_1,
           double lambda_2)
{

  double energy_I_i = lambda_0 * I_i(positionsOthers, velocityOthers, v_tilde_i, p_i, v_i, sigma_d, sigma_w, beta);
  double energy_S_i = lambda_1 * S_i(u_i, v_tilde_i);
  double energy_D_i = lambda_2 * D_i(z_i, p_i, v_tilde_i); // Energy towards goal

  double energy = energy_I_i + energy_S_i + energy_D_i;

  return energy;
}

// Total energy gradient
Eigen::Vector2d E_i_gradient(std::vector<Eigen::Vector2d> positionsOthers,
                             std::vector<Eigen::Vector2d> velocityOthers,
                             Eigen::Vector2d v_tilde_i,
                             Eigen::Vector2d p_i,
                             Eigen::Vector2d v_i,
                             double u_i, // Desired speed
                             Eigen::Vector2d z_i, // Goal
                             double sigma_d,
                             double sigma_w,
                             double beta,
                             double lambda_0,
                             double lambda_1,
                             double lambda_2)
{

  Eigen::Vector2d energyGradient;
  Eigen::Vector2d energyGradient_I_i;
  Eigen::Vector2d energyGradient_S_i;
  Eigen::Vector2d energyGradient_D_i;

  energyGradient_I_i = lambda_0 * I_i_gradient(positionsOthers, velocityOthers, v_tilde_i, p_i, v_i, sigma_d, sigma_w, beta);
  energyGradient_S_i = lambda_1 * S_i_gradient(u_i, v_tilde_i);
  energyGradient_D_i = lambda_2 * D_i_gradient(z_i, p_i, v_tilde_i);

  energyGradient = energyGradient_I_i + energyGradient_S_i + energyGradient_D_i;

  double energyTotalNorm =  energyGradient_I_i.norm() + energyGradient_S_i.norm() + energyGradient_D_i.norm();

  //std::cout << "EnergyDist: OtherObjects: " << energyGradient_I_i.norm()/energyTotalNorm << "  keepingSpeed: " << energyGradient_S_i.norm()/energyTotalNorm << "%  keepingGoal" << energyGradient_D_i.norm()/energyTotalNorm << "%" << std::endl;

  return energyGradient;
}


#endif /* PEOPLE_DUAL_PEOPLE_LEG_TRACKER_SOCIAL_INTERACTION_FORMULAS_H_ */
