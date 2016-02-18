#include <cob_people_fusion/association/association.h>
#include <cob_people_fusion/association/associatorGNN.h>



// Murty Algorithm, basic header only implementation
void color_print_solution(Eigen::Matrix<int, -1, -1> costMatrix, Eigen::Matrix<int, -1, -1> solution) {
  int i,j;
  fprintf(stderr , "\n");
  for(i=0; i<costMatrix.rows(); i++) {
    fprintf(stderr, " [");
    for(j=0; j<costMatrix.cols(); j++) {
      if(solution(i,j) == 1)
        fprintf(stderr,"\033[1m\033[33m");
      fprintf(stderr, "%5d ",costMatrix(i,j));
      fprintf(stderr,"\033[0m");

    }
    fprintf(stderr, "]\n");
  }
}

// TODO only one argument, the matrix should be sufficient
int** eigenmatrix_to_cmatrix(Eigen::Matrix<int, -1, -1> m, int rows, int cols) {
  int i,j;
  int** r;
  r = (int**)calloc(rows,sizeof(int*));
  for(i=0;i<rows;i++)
  {
    r[i] = (int*)calloc(cols,sizeof(int));
    for(j=0;j<cols;j++)
      r[i][j] = m(i,j);
  }
  return r;
}

Eigen::Matrix<int, -1, -1> cmatrix_to_eigenmatrix(int** C, int rows, int cols) {

  Eigen::Matrix<int, -1, -1> result;
  result = Eigen::Matrix<int, -1, -1>::Zero(rows,cols);

  int i,j;
  for(i=0; i<rows; i++) {
    for(j=0; j<cols; j++) {
      result(i,j) = C[i][j];
    }
  }

  return result;
}

std::vector<AssociationPtr>  AssociatorGNN::associate(std::vector<DetectionPtr>& detections, std::vector<TrackerPtr>& trackers, std::vector<DetectionPtr>& notAssociatedDetections){
  ROS_DEBUG_COND(DEBUG_ASSOCIATOR_GNN, "AssociatorGNN::%s - Number of detections: %i Number of trackers %i", __func__, (int) detections.size(), (int) trackers.size());

  std::vector<AssociationPtr> associations;

  // Calculate the costs
  for(std::vector<DetectionPtr>::iterator detectionIt = detections.begin();
      detectionIt < detections.end();
      detectionIt++)
  {
    bool associated = false;

    // Calculate the costs
    for(std::vector<TrackerPtr>::iterator trackerIt = trackers.begin();
        trackerIt < trackers.end();
        trackerIt++)
    {

      double distance = euclideanDistanceCost(*trackerIt, *detectionIt);

      if(distance < 0.4){
        associated = true;
        associations.push_back(AssociationPtr(new Association(*trackerIt, *detectionIt, distance)));
      }
    }

    if(!associated){
      notAssociatedDetections.push_back(*detectionIt);
    }

  }
  return associations;
}

std::vector<AssociationPtr>  AssociatorGNN::associateGNN(std::vector<DetectionPtr>& detections, std::vector<TrackerPtr>& trackers, std::vector<DetectionPtr>& notAssociatedDetections){
  ROS_DEBUG_COND(DEBUG_ASSOCIATOR_GNN, "AssociatorGNN::%s - Number of detections: %i Number of trackers %i", __func__, (int) detections.size(), (int) trackers.size());

  std::vector<AssociationPtr> associations;
  Eigen::Matrix<double, -1, -1> costMatrix = Eigen::Matrix<double,-1,-1>::Zero(trackers.size(), detections.size());
  Eigen::Matrix<int, -1, -1> costMatrixInt = Eigen::Matrix<int,-1,-1>::Zero(trackers.size(), detections.size());
  bool usedDetection[detections.size()]; //Like this.
  for(size_t i = 0; i < detections.size(); i++){
    usedDetection[i] = false;
  }

  // Calculate the costs
  size_t col = 0;
  for(std::vector<DetectionPtr>::iterator detectionIt = detections.begin();
      detectionIt < detections.end();
      detectionIt++)
  {
    bool associated = false;

    // Calculate the costs
    size_t row = 0;
    for(std::vector<TrackerPtr>::iterator trackerIt = trackers.begin();
        trackerIt < trackers.end();
        trackerIt++)
    {

      double distance = euclideanDistanceCost(*trackerIt, *detectionIt);

      costMatrix(row, col) = distance;
      costMatrixInt(row, col) = (int) (costMatrix(row, col) * 10000);

      row++;
    }

    col++;
  }

  ROS_DEBUG_STREAM_COND(DEBUG_ASSOCIATOR_GNN, "COSTMATRIX:" << std::endl << costMatrix << std::endl);


  //Solution solution;

  // Turn eigenmatrix into a c matrix
  int** m = eigenmatrix_to_cmatrix(costMatrixInt,costMatrixInt.rows(),costMatrixInt.cols());


  unsigned int nRows = costMatrix.rows();
  unsigned int nCols = costMatrix.cols();

  // Solve Hungarian
  hungarian_problem_t p;

  /* initialize the gungarian_problem using the cost matrix*/
  int matrix_size = hungarian_init(&p, m , nRows, nCols, HUNGARIAN_MODE_MINIMIZE_COST) ;

  /* solve the assignement problem */
  hungarian_solve(&p);

  //solution.cost_total = p.cost_total;
  //solution.assignmentMatrix =
  Eigen::Matrix<int,-1,-1> assignmentMatrix = cmatrix_to_eigenmatrix(p.assignment,nRows,nCols);

  ROS_DEBUG_STREAM_COND(DEBUG_ASSOCIATOR_GNN, "AssignmentMatrix:" << std::endl << assignmentMatrix << std::endl);

  /* free used memory */
  hungarian_free(&p);
  free(m);

  // Create associations
  for(size_t row = 0; row < nRows; row++){
    for(size_t col = 0; col < nCols; col++){
      if(assignmentMatrix(row, col) > 0){
        usedDetection[col] = true;
        double distance = costMatrix(row, col);
        associations.push_back(AssociationPtr(new Association(trackers[row], detections[col], distance)));
      }
    }
  }

  // Filter the not used detections
  for(size_t i = 0; i < detections.size(); i++){
    if(!usedDetection[i]){
      notAssociatedDetections.push_back(detections[i]);
    }
  }

  // Now iterate the association and filter them.
  std::vector<AssociationPtr> filteredAssociations;
  std::vector<AssociationPtr> removedAssociations;
  for(std::vector<AssociationPtr>::iterator assoIt = associations.begin();
      assoIt < associations.end();
      assoIt++){

    // If the distance is not far // TODO make this better into a filter class
    if((*assoIt)->getDistance() < 0.4){
      filteredAssociations.push_back(*assoIt);
    }else{
      removedAssociations.push_back(*assoIt);
      notAssociatedDetections.push_back((*assoIt)->getDetection());
    }
  }


  // Debug output
  ROS_DEBUG_STREAM_COND(DEBUG_ASSOCIATOR_GNN, "Used associations:" << std::endl);
  for(std::vector<AssociationPtr>::iterator assoIt = filteredAssociations.begin();
      assoIt < filteredAssociations.end();
      assoIt++){
    ROS_DEBUG_STREAM_COND(DEBUG_ASSOCIATOR_GNN, "\t" << (**assoIt) << std::endl);
  }

  // Debug output
  ROS_DEBUG_STREAM_COND(DEBUG_ASSOCIATOR_GNN, "Removed associations:" << std::endl);
  for(std::vector<AssociationPtr>::iterator assoIt = removedAssociations.begin();
      assoIt < removedAssociations.end();
      assoIt++){
    ROS_DEBUG_STREAM_COND(DEBUG_ASSOCIATOR_GNN, "\t" << (**assoIt) << std::endl);
  }

  return filteredAssociations;
}




double AssociatorGNN::euclideanDistanceCost(TrackerPtr tracker, DetectionPtr detection){
  return (tracker->getCurrentState().getPos() - detection->getState().getPos()).length();
}

//ROS_DEBUG_COND(FUSION_NODE_DEBUG, "FusionNode::%s - Number of detections: %i", __func__, (int) detectionArray->detections.size());
