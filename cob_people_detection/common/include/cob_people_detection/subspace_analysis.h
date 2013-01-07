#ifndef SSA_H_
#define SSA_H_


#include<opencv/cv.h>
#include"thirdparty/decomposition.hpp"

// Base class for SubSpace Analysis(SSA)
//
//
namespace SubspaceAnalysis{

  void project(cv::Mat& src_mat,cv::Mat& proj_mat,cv::Mat& avg_mat,cv::Mat& coeff_mat);
  void reconstruct(cv::Mat& coeffs,cv::Mat& proj_mat,cv::Mat& avg,cv::Mat& rec_im);
  void calcDataMat(std::vector<cv::Mat>& input_data,cv::Mat& data_mat);
  void DFFS(cv::Mat& orig_mat,cv::Mat& recon_mat,cv::Mat& avg,std::vector<double>& DFFS);




  class SSA
  {

    public:
      SSA(){};
      SSA(cv::Mat& input_data,int& ss_dim);
      virtual ~SSA(){};
      void calcDataMat(std::vector<cv::Mat>& input_data,cv::Mat& data_mat);
      void calcDataMatMean(cv::Mat& data,cv::Mat& mean);
      void decompose();
      void decompose(cv::Mat& data_mat);
      void decompose2(cv::Mat& data_mat);

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
      LDA(Mat& input_data,std::vector<int>& input_labels,int& ss_dim);
      virtual ~LDA(){};

      void calcClassMean(cv::Mat& data_mat,std::vector<int>& label_vec,std::vector<cv::Mat>&  mean_vec);
      void calcModelMatrix(std::vector<int>& label_vec,cv::Mat& M);

      int num_classes;
      std::vector<cv::Mat> class_means;

  };


  class PCA:public SSA
  {
    public:
    PCA(){};
    PCA(cv::Mat& input_data,int& ss_dim);
    virtual ~PCA(){};
    void calcProjMatrix(cv::Mat& data);
  };

  class Eigenfaces
  {
    public:
    Eigenfaces(std::vector<cv::Mat>& img_vec,std::vector<int>& label_vec,int& red_dim);
    virtual ~Eigenfaces(){};

    void projectToSubspace(cv::Mat& src_mat,cv::Mat& dst_mat,std::vector<double>& DFFS);
    void meanCoeffs(cv::Mat& coeffs,std::vector<int>& label_vec,cv::Mat& mean_coeffs);

    protected:
    SubspaceAnalysis::PCA pca_;
    cv::Mat proj_;
    cv::Mat avg_;
    cv::Mat model_data_;
    cv::Mat proj_model_data_;





  };


};
#endif
