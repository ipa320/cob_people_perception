#ifndef SSA_H_
#define SSA_H_


#include<opencv/cv.h>
#include"thirdparty/decomposition.hpp"

// Base class for SubSpace Analysis(SSA)
//
//
namespace SubspaceAnalysis{

  void project(cv::Mat& data_mat,cv::Mat& coeffs);

  class Eigenfaces
  {
    public:
    Eigenfaces(std::vector<cv::mat>& img_vec,std::vector<int>label_vec,int& red_dim)
    virtual ~Eigenfaces(){};


    protected:
    SubspaceAnalysis::PCA pca_;
    cv::Mat proj_;
    cv::Mat avg_;
    cv::Mat model_data_;
    cv::Mat proj_model_data_;





  }


  class SSA
  {

    public:
      SSA(){};
      SSA(std::vector<cv::Mat>& input_data,int& ss_dim);
      virtual ~SSA(){};
      void calcDataMat(std::vector<cv::Mat>& input_data,cv::Mat& data_mat);
      void calcDataMatMean(cv::Mat& data,cv::Mat& mean);
      void decompose();

      cv::Mat data;
      cv::Mat model;
      cv::Mat eigenvals;
      cv::Mat eigenvecs;
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
    public:
    PCA(cv::Mat& input_data,int& ss_dim);
    virtual ~PCA(){};
    void calcProjMatrix();
  };


};
#endif
