#ifndef SSA_H_
#define SSA_H_


#include<opencv/cv.h>

// Base class for SubSpace Analysis(SSA)
//
//

class SSA
{

  public:
    SSA(){};
    SSA(std::vector<cv::Mat>& input_data,int& ss_dim);
    virtual ~SSA(){};
    void calcDataMat(std::vector<cv::Mat>& input_data,cv::Mat& data_mat);
    void calcDataMatMean(cv::Mat& data,cv::Mat& mean);
    void decomposeModel();

    cv::Mat data;
    cv::Mat model;
    cv::Mat eigenvals;
    std::vector<cv::Mat> eigenvecs;
    cv::Mat mean;
    int dimension;
};


class LDA:public SSA
{

  public:
    LDA(){};
    LDA(std::vector<cv::Mat>& input_data,std::vector<int>& input_labels,int& ss_dim);
    virtual ~LDA(){};

    void calcClassMean(cv::Mat& data_mat,std::vector<int>& label_vec,std::vector<cv::Mat>&  mean_vec);
    void calcModelMatrix(std::vector<int>& label_vec,cv::Mat& M);

    int num_classes;
    std::vector<cv::Mat> class_means;

};


class PCA:public SSA
{
  PCA(std::vector<cv::Mat>& input_data,int& ss_dim);
  virtual ~PCA(){};
  void calcModelMatrix(cv::Mat& M);
};


#endif
