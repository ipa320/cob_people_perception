/*
 * jpda.h
 *
 *  Created on: Jun 10, 2015
 *      Author: frm-ag
 */

#ifndef PEOPLE_DUAL_PEOPLE_LEG_TRACKER_INCLUDE_DUAL_PEOPLE_LEG_TRACKER_JPDA_JPDA_H_
#define PEOPLE_DUAL_PEOPLE_LEG_TRACKER_INCLUDE_DUAL_PEOPLE_LEG_TRACKER_JPDA_JPDA_H_

namespace jpda{

double calculateAssociationProbability(Eigen::Matrix< int,-1,-1> assignmentMatrix,
                                     double falseAlarmProbability,
                                     Eigen::Matrix< double, -1, -1> measProbabilities){

  return 1.0;

  double product = 0.0;
  int numberOfMeasurements = assignmentMatrix.cols();
  int numberOfAssignments = assignmentMatrix.sum();

  // Build the product of the single
  for(int j = 0; j < assignmentMatrix.cols(); j++)        // The measurements
    for(int i = 0; i < assignmentMatrix.rows(); i++){   // The objects

      if(assignmentMatrix(i,j)==1)
        product *= measProbabilities(i,j);

    }

  product *= pow(falseAlarmProbability,numberOfMeasurements-numberOfAssignments);

  return product;
}

} // namespace



#endif /* PEOPLE_DUAL_PEOPLE_LEG_TRACKER_INCLUDE_DUAL_PEOPLE_LEG_TRACKER_JPDA_JPDA_H_ */
