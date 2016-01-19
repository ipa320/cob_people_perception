#ifndef PEOPLE_DUAL_PEOPLE_LEG_TRACKER_INCLUDE_DUAL_PEOPLE_LEG_TRACKER_ASSOCIATION_ASSOCIATORGNN_H_
#define PEOPLE_DUAL_PEOPLE_LEG_TRACKER_INCLUDE_DUAL_PEOPLE_LEG_TRACKER_ASSOCIATION_ASSOCIATORGNN_H_

// Own includes
#include <people_fusion_node/detection/detection.h>
#include <people_fusion_node/association/association.h>
#include <people_fusion_node/tracker.h>


// Eigen includes
#include <Eigen/Dense>

#include <libhungarian/hungarian.h>

#define DEBUG_ASSOCIATOR_GNN 1


//#define PSEUDO_INF
#define INF 100000


// Murty Algorithm, basic header only implementation
void color_print_solution(Eigen::Matrix<int, -1, -1> costMatrix, Eigen::Matrix<int, -1, -1> solution);

// TODO only one argument, the matrix should be sufficient
int** eigenmatrix_to_cmatrix(Eigen::Matrix<int, -1, -1> m, int rows, int cols);

Eigen::Matrix<int, -1, -1> cmatrix_to_eigenmatrix(int** C, int rows, int cols);


class AssociatorGNN{

  private:
    double euclideanDistanceCost(TrackerPtr tracker, DetectionPtr detection);

  public:
    std::vector<AssociationPtr> associate(std::vector<DetectionPtr>& detections, std::vector<TrackerPtr>& trackers, std::vector<DetectionPtr>& notAssociatedDetections);

    std::vector<AssociationPtr> associateGNN(std::vector<DetectionPtr>& detections, std::vector<TrackerPtr>& trackers, std::vector<DetectionPtr>& notAssociatedDetections);

};



#endif /* PEOPLE_DUAL_PEOPLE_LEG_TRACKER_INCLUDE_DUAL_PEOPLE_LEG_TRACKER_ASSOCIATION_ASSOCIATORGNN_H_ */
