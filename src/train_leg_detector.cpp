/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include "leg_detector/laser_processor.h"
#include "leg_detector/calc_leg_features.h"
#include "leg_detector/ClusterMsg.h"
#include "leg_detector/LabeledRangeScanMsg.h"

#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/ml.h"

#include "people_msgs/PositionMeasurement.h"
#include "sensor_msgs/LaserScan.h"

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>

#define USE_BASH_COLORS
#include <leg_detector/color_definitions.h>
#include <stdlib.h>     /* exit, EXIT_FAILURE */

using namespace std;
using namespace laser_processor;
using namespace ros;

enum LoadType {LOADING_NONE, LOADING_TRAIN, LOADING_TEST};

enum ClassificatorType {CVBOOST, CVRTREES};

struct classificator {
  ClassificatorType type;
  string desc;
  CvStatModel* pClassificator;
};

class TrainLegDetector
{
public:
  ScanMask mask_;
  int mask_count_;

  vector< vector<float> > pos_data_;
  vector< vector<float> > neg_data_;
  vector< vector<float> > test_data_;

  vector< vector<float> > test_pos_data_;  //Positiv test data
  vector< vector<float> > test_neg_data_;  //Negative test data

  vector<classificator> classificators;

  //CvRTrees forest;
  //CvBoost adaBoost;

  float connected_thresh_;

  int feat_count_;

  TrainLegDetector() : mask_count_(0), connected_thresh_(0.06), feat_count_(0)
  {
  }

  void loadData(LoadType load, char* file)
  {
    if (load != LOADING_NONE)
    {
        switch (load)
        {
        case LOADING_TRAIN:
            printf("Loading training data from file: %s\n",file); break;
        case LOADING_TEST:
            printf("Loading test data from file: %s\n",file); break;
        default:
            break;
        }

        // Open the bagfile
        rosbag::Bag bag;
        try {
          bag.open(file, rosbag::bagmode::Read);
          cout << "File open" << endl;
        } catch (const rosbag::BagException& e) {
          cout << RED << "Error opening the file: " << file << RESET << endl;
          exit (EXIT_FAILURE);
        }

        sensor_msgs::LaserScan::Ptr pLaserScan;
        leg_detector::LabeledRangeScanMsg::Ptr pLabeledRangeScanMsg;

        // Iterate the messages
        std::vector<std::string> topics;
        topics.push_back(std::string("/scan_front"));   // TODO make this more general
        topics.push_back(std::string("/labels"));       // TODO make this more general
        //topics.push_back(std::string("/tf"));

        // Create the view
        rosbag::View view(bag, rosbag::TopicQuery(topics));
        BOOST_FOREACH(rosbag::MessageInstance const m, view) // Iterate through the messages
        {

          // Initiate as
          sensor_msgs::LaserScan::Ptr pLaserScanTemp = m.instantiate<sensor_msgs::LaserScan>();
          leg_detector::LabeledRangeScanMsg::Ptr pLabeledRangeScanMsgTemp = m.instantiate<leg_detector::LabeledRangeScanMsg>();

          if (pLaserScanTemp != NULL) {
              pLaserScan = pLaserScanTemp;
          }
          if (pLabeledRangeScanMsgTemp != NULL) {
              pLabeledRangeScanMsg = pLabeledRangeScanMsgTemp;
          }

          // Check if both messages are defined and have the 'same' timestamp -> At this point the laserscan and its associated labels
          if (pLaserScan != NULL && pLabeledRangeScanMsg != NULL &&
              abs(pLaserScan->header.stamp.nsec - pLabeledRangeScanMsg->header.stamp.nsec) < (uint32_t) 1){

              if(pLabeledRangeScanMsg->clusters.size() > 0){ //If there are clusters
                  this->loadCb(pLaserScan,pLabeledRangeScanMsg,load);
              }
              pLaserScan == NULL;
              pLabeledRangeScanMsg == NULL;

          }
        }//end BOOST_FOREACH

    }
  }

  //void loadCb(string name, sensor_msgs::LaserScan* scan, ros::Time t, ros::Time t_no_use, vector< vector<float> >& data) //TODO Remove first parameter
  void loadCb(sensor_msgs::LaserScan::Ptr pLaserScan, leg_detector::LabeledRangeScanMsg::Ptr pLabeledRangeScanMsg, LoadType load)
  {
      // TODO extract the clusters

      // Iterate the clusters inside the message
      for(leg_detector::LabeledRangeScanMsg::_clusters_type::iterator clusterIt = pLabeledRangeScanMsg->clusters.begin(); clusterIt != pLabeledRangeScanMsg->clusters.end(); clusterIt++){

          SampleSet* pCluster = new SampleSet();
          cout << GREEN << pLaserScan->header.stamp << RESET << endl;
          cout << "\tLabel:[" << clusterIt->label << "] " << endl;
          pCluster->label = clusterIt->label;

          // Iterate the indices
          for(leg_detector::ClusterMsg::_indices_type::iterator indexIt = clusterIt->indices.begin(); indexIt != clusterIt->indices.end(); indexIt++){
              int16_t index = ((int16_t)(*indexIt));
              //cout << "Extracting index " << index << endl;

              // TODO Generate SampleSet here
              Sample* s = Sample::Extract(index, *pLaserScan);
              pCluster->insert(s);

              //cout << "\t\t" << YELLOW << index <<  " " << RED << "x: " << s->x << " y: " << s->y << BLUE << " range: " << s->range << RESET << endl;
          }

          switch (load)
          {
          case LOADING_TRAIN:
              // Input into the relevant data set
              if(pCluster->label == "LEG"){
                  cout << "Pushing pos data into pos_data_" << endl;
                  pos_data_.push_back( calcLegFeatures(pCluster, *pLaserScan));
              }else if(pCluster->label == "NOLEG"){
                  cout << "Pushing neg data into neg_data_" << endl;
                  neg_data_.push_back( calcLegFeatures(pCluster, *pLaserScan));
              }
              break;
          case LOADING_TEST:
              if(pCluster->label == "LEG"){
                  cout << "Pushing test data into test_pos_data_" << endl;
                  test_pos_data_.push_back( calcLegFeatures(pCluster, *pLaserScan));
              }else if(pCluster->label == "NOLEG"){
                  cout << "Pushing neg data into test_neg_data_" << endl;
                  test_neg_data_.push_back( calcLegFeatures(pCluster, *pLaserScan));
              }
              break;
          default:
              break;
          }
      }
  }

  void train()
  {
    int sample_size = pos_data_.size() + neg_data_.size();
    feat_count_ = pos_data_[0].size();

    CvMat* cv_data = cvCreateMat( sample_size, feat_count_, CV_32FC1);
    CvMat* cv_resp = cvCreateMat( sample_size, 1, CV_32S);

    // Put positive data in opencv format.
    int j = 0;
    for (vector< vector<float> >::iterator i = pos_data_.begin();
         i != pos_data_.end();
         i++)
    {
      float* data_row = (float*)(cv_data->data.ptr + cv_data->step*j);
      for (int k = 0; k < feat_count_; k++)
        data_row[k] = (*i)[k];
      
      cv_resp->data.i[j] = 1;
      j++;
    }

    // Put negative data in opencv format.
    for (vector< vector<float> >::iterator i = neg_data_.begin();
         i != neg_data_.end();
         i++)
    {
      float* data_row = (float*)(cv_data->data.ptr + cv_data->step*j);
      for (int k = 0; k < feat_count_; k++)
        data_row[k] = (*i)[k];
      
      cv_resp->data.i[j] = -1;
      j++;
    }

    CvMat* var_type = cvCreateMat( 1, feat_count_ + 1, CV_8U );
    cvSet( var_type, cvScalarAll(CV_VAR_ORDERED));
    cvSetReal1D( var_type, feat_count_, CV_VAR_CATEGORICAL );
    
    float priors[] = {1.0, 1.0};
    
    CvRTParams fparam(8, // Maximal tree depth
                      20,
                      0,
                      false,
                      10,
                      priors,
                      false,
                      5,
                      50,
                      0.001f,
                      CV_TERMCRIT_ITER);
    fparam.term_crit = cvTermCriteria(CV_TERMCRIT_ITER, 100, 0.1);
    
    CvRTrees* forest = new CvRTrees();
    forest->train(cv_data, //Training Data
                 CV_ROW_SAMPLE, // each row is a sample
                 cv_resp, // The response that is expected
                 0, // Can be used to choose as subset of the features
                 0, // Can be used to choose as subset of the samples
                 var_type, // ???
                 0, // Can be used to mark missing values
                 fparam);

    CvBoostParams boostParams;
        boostParams.boost_type = CvBoost::REAL;
        boostParams.weak_count = feat_count_;
        boostParams.weight_trim_rate = 0.50; //A threshold between 0 and 1 used to save computational time. Samples with summary weight \leq 1 - weight\_trim\_rate do not participate in the next iteration of training. Set this parameter to 0 to turn off this functionality.
        boostParams.max_depth = 1;
        boostParams.cv_folds = 0;
        //boostParams.priors = priors;

    CvBoost* adaBoost = new CvBoost();
    adaBoost->train(cv_data, //Training Data
                   CV_ROW_SAMPLE, // each row is a sample
                   cv_resp, // The response that is expected
                   0, // Can be used to choose as subset of the features
                   0, // Can be used to choose as subset of the samples
                   var_type, // ???
                   0, // Can be used to mark missing values
                   boostParams);

    // Store the forestClassificator
    classificator forestClassificator;
    forestClassificator.desc = "forestClassificator";
    forestClassificator.pClassificator = forest;
    forestClassificator.type = CVRTREES;
    classificators.push_back(forestClassificator);

    classificator adaBoostClassificator;
    adaBoostClassificator.desc = "adaBoostClassificator";
    adaBoostClassificator.pClassificator = adaBoost;
    adaBoostClassificator.type = CVBOOST;
    classificators.push_back(adaBoostClassificator);

    cvReleaseMat(&cv_data);
    cvReleaseMat(&cv_resp);
    cvReleaseMat(&var_type);
  }

  void test()
  {
    // Test every iterator there is
    for(vector<classificator>::iterator classificatorIt = classificators.begin(); classificatorIt != classificators.end(); classificatorIt++){
        cout << BOLDMAGENTA << classificatorIt->desc << RESET << endl;

        CvMat* tmp_mat = cvCreateMat(1,feat_count_,CV_32FC1);

        int pos_right = 0;
        int pos_total = 0;
        for (vector< vector<float> >::iterator i = pos_data_.begin();
             i != pos_data_.end();
             i++)
        {
          for (int k = 0; k < feat_count_; k++)
            tmp_mat->data.fl[k] = (float)((*i)[k]);

          float prediction = 0;
          switch(classificatorIt->type){
              case CVBOOST:
                  prediction = ((CvBoost *)classificatorIt->pClassificator)->predict(tmp_mat);
                  break;
              case CVRTREES:
                  prediction = ((CvRTrees *)classificatorIt->pClassificator)->predict(tmp_mat);
                  break;
          }
          if (prediction > 0)
            pos_right++;
          pos_total++;
        }

        int neg_right = 0;
        int neg_total = 0;
        for (vector< vector<float> >::iterator i = neg_data_.begin();
             i != neg_data_.end();
             i++)
        {
          for (int k = 0; k < feat_count_; k++)
            tmp_mat->data.fl[k] = (float)((*i)[k]);
          float prediction = 0;
          switch(classificatorIt->type){
              case CVBOOST:
                  prediction = ((CvBoost *)classificatorIt->pClassificator)->predict(tmp_mat);
                  break;
              case CVRTREES:
                  prediction = ((CvRTrees *)classificatorIt->pClassificator)->predict(tmp_mat);
                  break;
          }
          if (prediction < 0)
            neg_right++;
          neg_total++;
        }

        // Test for positive data (no legs)
        int test_pos_right = 0;
        int test_pos_total = 0;
        for (vector< vector<float> >::iterator i = test_pos_data_.begin();
             i != test_pos_data_.end();
             i++)
        {
          for (int k = 0; k < feat_count_; k++)
            tmp_mat->data.fl[k] = (float)((*i)[k]);
          float prediction = 0;
          switch(classificatorIt->type){
              case CVBOOST:
                  prediction = ((CvBoost *)classificatorIt->pClassificator)->predict(tmp_mat);
                  break;
              case CVRTREES:
                  prediction = ((CvRTrees *)classificatorIt->pClassificator)->predict(tmp_mat);
                  break;
          }
          if (prediction > 0)
            test_pos_right++;
          test_pos_total++;
        }

        // Test for negative data (no legs)
        int test_neg_right = 0;
        int test_neg_total = 0;
        for (vector< vector<float> >::iterator i = test_neg_data_.begin();
             i != test_neg_data_.end();
             i++)
        {
          for (int k = 0; k < feat_count_; k++)
            tmp_mat->data.fl[k] = (float)((*i)[k]);
          float prediction = 0;
          switch(classificatorIt->type){
              case CVBOOST:
                  prediction = ((CvBoost *)classificatorIt->pClassificator)->predict(tmp_mat);
                  break;
              case CVRTREES:
                  prediction = ((CvRTrees *)classificatorIt->pClassificator)->predict(tmp_mat);
                  break;
          }
          if (prediction > 0)
              test_neg_right++;
          test_neg_total++;
        }


        // Test for negative data

        printf("\t----- Training Set -----(N=%lu)\n", neg_data_.size()+pos_data_.size());
        printf("\t True positives: %d\n",pos_right);
        printf("\t False positives: %d\n",pos_total-pos_right);

        printf("\t True negatives: %d\n",neg_right);
        printf("\t False negatives: %d\n\n",neg_total-neg_right);

        printf("\t True positives Rate(Sensitivity): %g %%\n",(float)(pos_right)/pos_total * 100);
        printf("\t True negatives Rate(Specificity): %g %%\n\n",(float) (neg_right)/neg_total * 100);

        printf("\t-----   Test Set   -----(N=%lu)\n", test_neg_data_.size()+test_pos_data_.size());

        printf("\t True positives: %d\n",test_pos_right);
        printf("\t False positives: %d\n",test_pos_total-test_pos_right);

        printf("\t True negatives: %d\n",test_neg_right);
        printf("\t False negatives: %d\n\n",test_neg_total-test_neg_right);

        printf("\t True positives Rate(Sensitivity): %g %% \n",(float) (test_pos_right)/test_pos_total * 100);
        printf("\t True negatives Rate(Specificity): %g %% \n\n",(float) (test_neg_right)/test_neg_total * 100);

        cvReleaseMat(&tmp_mat);
    }

  }

  void save(char* outputfile_prefix)
  {
      cout << "Save called!" << endl;
      for(vector<classificator>::iterator classificatorIt = classificators.begin(); classificatorIt != classificators.end(); classificatorIt++){
          char outputfile[1000];
          outputfile[0] = 0;
          strcpy(outputfile,outputfile_prefix);
          strncat(outputfile,(("_" + classificatorIt->desc + ".yaml").c_str()),100);
          cout << "Saving: " << outputfile << endl;

          //classificatorIt->pClassificator->save(outputfile);
      }
    //forest.save(file);
  }
};

int main(int argc, char **argv)
{
  TrainLegDetector tld;

  LoadType loading = LOADING_NONE;

  char outputfile_prefix[100];
  outputfile_prefix[0] = 0;

  printf("Loading data...\n");
  for (int i = 1; i < argc; i++)
  {
    if (!strcmp(argv[i],"--train"))
      loading = LOADING_TRAIN;
    else if (!strcmp(argv[i],"--test"))
      loading = LOADING_TEST;
    else if (!strcmp(argv[i],"--outputfileprefix"))
    {
      if (++i < argc)
        strncpy(outputfile_prefix,argv[i],100);

        // TODO Check existenz of the file

      continue;
    }
    else
      tld.loadData(loading, argv[i]);
  }
  printf("Positive Dataset %lu entries\n",tld.pos_data_.size());
  printf("Negativ Dataset %lu entries\n",tld.neg_data_.size());
  printf("Test Dataset %lu entries\n",tld.test_data_.size());

  printf("Training classifier...\n");
  tld.train();

  printf("Evaluating classifier...\n");
  tld.test();
  
  if (strlen(outputfile_prefix) > 0)
  {
    //printf("Saving classifier as: %s\n", outputfile_prefix);
    tld.save(outputfile_prefix);
  }
}
