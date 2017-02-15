#ifndef PEOPLE_cob_dual_leg_tracker_INCLUDE_cob_dual_leg_tracker_VISUALIZATION_MATRIX_COUT_HELPER_H_
#define PEOPLE_cob_dual_leg_tracker_INCLUDE_cob_dual_leg_tracker_VISUALIZATION_MATRIX_COUT_HELPER_H_

class CoutMatrixHelper{
  public:
    static void cout_probability_matrix(std::string title, // Title of the Matrix
                                        Eigen::Matrix<double , -1, -1> probabilityMatrix,
                                        Eigen::Matrix<int, -1, -1> assignmentMatrix,
                                        Eigen::VectorXi indicesMAP,
                                        size_t nLegsTracked,
                                        size_t nMeasurementsReal,
                                        size_t nMeasurementsFake){

      std::cout << title << std::endl;
      std::cout << "      ";
      for(int j = 0; j < nMeasurementsReal; j++)
        std::cout << BOLDGREEN << "LM" << std::setw(2) << j<< "    |";
      std::cout << RESET;

      for(int j = 0; j < nMeasurementsFake; j++)
        std::cout << BOLDYELLOW << "LMF" << std::setw(2) << j<< "    |";
      std::cout << RESET << std::endl;

      for(int i = 0; i < nLegsTracked; i++){
        std::cout << BOLDMAGENTA << "LT" << std::setw(3) <<  indicesMAP(i) << RESET <<"|";
        for(int j = 0; j < nMeasurementsReal + nMeasurementsFake; j++){
            if(assignmentMatrix(i,j) == 1)
              std::cout << YELLOW;

            std::cout << std::setw(6) << std::fixed << std::setprecision(6) << probabilityMatrix(i,j) << RESET "|";
        }
        std::cout << std::endl;
      }

      std::cout << std::endl;

    }

    static void cout_cost_matrix(std::string title, // Title of the Matrix
                                        Eigen::Matrix<int, -1, -1> costMatrix,
                                        Eigen::Matrix<int, -1, -1> assignmentMatrix,
                                        Eigen::VectorXi indicesMAP,
                                        size_t nLegsTracked,
                                        size_t nMeasurementsReal,
                                        size_t nMeasurementsFake){

      // Print the probability matrix //TODO move this to a visualization
      std::cout << title << std::endl;
      std::cout << "      ";
      for(int j = 0; j < nMeasurementsReal; j++)
        std::cout << BOLDGREEN << "LM" << std::setw(2) << j<< "  |";
      std::cout << RESET;

      for(int j = 0; j < nMeasurementsFake; j++)
        std::cout << BOLDYELLOW << "LMF" << std::setw(2) << j<< "  |";
      std::cout << RESET << std::endl;

      for(int i = 0; i < nLegsTracked; i++){
        std::cout << BOLDMAGENTA << "LT" << std::setw(3) <<  indicesMAP(i) << RESET <<"|";
        for(int j = 0; j < nMeasurementsReal + nMeasurementsFake; j++){
          if(assignmentMatrix(i,j) == 1)
            std::cout << YELLOW;

            std::cout << std::setw(6) << std::fixed << std::setprecision(5) << costMatrix(i,j) << RESET "|";
        }
        std::cout << std::endl;
      }
      std::cout << std::endl;

    }
};



#endif /* PEOPLE_cob_dual_leg_tracker_INCLUDE_cob_dual_leg_tracker_VISUALIZATION_MATRIX_COUT_HELPER_H_ */
