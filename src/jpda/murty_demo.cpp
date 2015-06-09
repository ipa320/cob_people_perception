// System includes
#include <iostream>
#include <climits>

// Own includes
#include <dual_people_leg_tracker/jpda/murty.h>
#include <dual_people_leg_tracker/benchmarking/timer.h>

int main(int argc, char **argv)
{
  // Variables
  int nRows = 30;
  int nCols = 30;
  int k     = 5; //k-th best solutions


  srand((unsigned int) time(0)); // Generate random
  Eigen::Matrix<int, -1, -1> problem = (Eigen::Matrix<float, -1, -1>::Random(nRows,nCols)*100).cast<int> ();;

  // Make entries positive
  problem = (problem.array().abs()).matrix();

  std::cout << "CostMatrix" << std::endl << problem << std::endl;

  std::vector<Solution> solutions;

  benchmarking::Timer murtyTimer; murtyTimer.start();
  solutions = murty(problem,k);
  murtyTimer.stop();

  std::cout << "Solutions are:" << std::endl;
  for(std::vector<Solution>::iterator solIt = solutions.begin(); solIt != solutions.end(); solIt++){
    color_print_solution(problem,solIt->assignmentMatrix);
    std::cout << "Costs "<< "\033[1m\033[31m" << solIt->cost_total << "\033[0m" << std::endl;
  }

  std::cout << "Solving took \033[1m\033[34m" << murtyTimer.getElapsedTimeMs() << " ms\033[0m" << std::endl;

}


