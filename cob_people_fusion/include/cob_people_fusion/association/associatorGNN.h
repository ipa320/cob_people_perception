#ifndef PEOPLE_DUAL_PEOPLE_LEG_TRACKER_INCLUDE_DUAL_PEOPLE_LEG_TRACKER_ASSOCIATION_ASSOCIATORGNN_H_
#define PEOPLE_DUAL_PEOPLE_LEG_TRACKER_INCLUDE_DUAL_PEOPLE_LEG_TRACKER_ASSOCIATION_ASSOCIATORGNN_H_

// Own includes
#include <cob_people_fusion/detection/detection.h>
#include <cob_people_fusion/association/association.h>
#include <cob_people_fusion/tracker.h>


// Eigen includes
#include <Eigen/Dense>

#include <libhungarian/hungarian.h>

#define DEBUG_ASSOCIATOR_GNN 1

//#define PSEUDO_INF
#define INF 100000


// Murty Algorithm, basic header only implementation
void color_print_solution(Eigen::Matrix<int, -1, -1> costMatrix, Eigen::Matrix<int, -1, -1> solution);

// Eigen <-> C conversions
int** eigenmatrix_to_cmatrix(Eigen::Matrix<int, -1, -1> m, int rows, int cols);
Eigen::Matrix<int, -1, -1> cmatrix_to_eigenmatrix(int** C, int rows, int cols);

class AssociatorGNN{

  private:
    /**
     * Calculates the euclidean distance between a diven tracker and detection
     * @param tracker The tracker
     * @param detection The detection
     * @return euclidean distance
     */
    double euclideanDistanceCost(TrackerPtr tracker, DetectionPtr detection);

  public:
    /**
     * Nearest neighbour association
     * @param detections All detections
     * @param trackers All trackers
     * @param notAssociatedDetections detections which where not associated
     * @return associations
     */
    std::vector<AssociationPtr> associate(std::vector<DetectionPtr>& detections, std::vector<TrackerPtr>& trackers, std::vector<DetectionPtr>& notAssociatedDetections);

    /**
     * Global nearest neighbour association
     * @param detections All detections
     * @param trackers All trackers
     * @param notAssociatedDetections detections which where not associated
     * @return associations
     */
    std::vector<AssociationPtr> associateGNN(std::vector<DetectionPtr>& detections, std::vector<TrackerPtr>& trackers, std::vector<DetectionPtr>& notAssociatedDetections);

};



#endif /* PEOPLE_DUAL_PEOPLE_LEG_TRACKER_INCLUDE_DUAL_PEOPLE_LEG_TRACKER_ASSOCIATION_ASSOCIATORGNN_H_ */
