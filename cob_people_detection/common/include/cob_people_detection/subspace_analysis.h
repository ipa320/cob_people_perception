#ifndef SSA_H_
#define SSA_H_


#include<opencv/cv.h>
#include<iostream>
#include<opencv/highgui.h>
#include<opencv/ml.h>
#include<fstream>
#include<ostream>
#include<limits>

// Base class for SubSpace Analysis(SSA)
//
//
namespace SubspaceAnalysis{

  void dump_matrix(cv::Mat& mat,std::string filename);
  void  mat_info(cv::Mat& mat);
  void  mat_info(cv::Mat& mat);


  enum Classifier
  {
    CLASS_MIN_DIFFS,
    CLASS_SVM,
    CLASS_KNN,
  };


  //Baseclass for Fisherfaces and Eigenfaces
  class XFaces
  {
    public:
    XFaces():svm_trained_(false),knn_trained_(false){};

    virtual ~XFaces(){};
    void retrieve(std::vector<cv::Mat>& out_eigenvectors,cv::Mat& out_eigenvalues,cv::Mat& out_avg,cv::Mat& out_proj_model_data);
    //void classify(cv::Mat& src_mat,int& class_index);
    void classify(cv::Mat& coeff_arr,Classifier method,int& class_index);
    void projectToSubspace(cv::Mat& probe_mat,cv::Mat& coeff_arr,double& DFFS);

    protected:
    void project(cv::Mat& src_mat,cv::Mat& proj_mat,cv::Mat& avg_mat,cv::Mat& coeff_mat);
    void reconstruct(cv::Mat& coeffs,cv::Mat& proj_mat,cv::Mat& avg,cv::Mat& rec_im);
    void calcDataMat(std::vector<cv::Mat>& input_data,cv::Mat& data_mat);
    void calcDFFS(cv::Mat& orig_mat,cv::Mat& recon_mat,cv::Mat& avg,std::vector<double>& DFFS);
    void calcDIFS(cv::Mat& probe_mat,std::vector<double>& DFFS);
    void mat2arr(cv::Mat& src_mat,cv::Mat& dst_mat);
    //data
    int ss_dim_;
    cv::Mat eigenvector_arr_;
    cv::Mat eigenvalue_arr_;
    cv::Mat avg_arr_;
    cv::Mat model_data_arr_;
    cv::Mat proj_model_data_arr_;
    cv::Mat model_label_arr_;


    //classification flags
    CvSVM svm_;
    bool svm_trained_;

    CvKNearest knn_;
    bool knn_trained_;
  };

  //Baseclass for PCA LDA
  class SSA
  {

    public:
      SSA(){};
      SSA(cv::Mat& input_data);
      virtual ~SSA(){};
      void calcDataMat(std::vector<cv::Mat>& input_data,cv::Mat& data_mat);
      void calcDataMatMean(cv::Mat& data,cv::Mat& mean_row);
      void decompose();
      void decompose(cv::Mat& data_mat);
      void decompose2(cv::Mat& data_mat);
      //Interface methods
      void retrieve(cv::Mat& proj,cv::Mat& avg,cv::Mat& proj_model_data);

      cv::Mat eigenvals;
      cv::Mat eigenvecs;
      cv::Mat mean;
  };


  class LDA:public SSA
  {

    public:
      LDA(){};
      LDA(cv::Mat& input_data,std::vector<int>& input_labels,int& num_classes,int& ss_dim);
      virtual ~LDA(){};

      void calcClassMean(cv::Mat& data_mat,std::vector<int>& label_vec,cv::Mat&  class_mean_arr);
      void calcProjMatrix(cv::Mat& data_arr,std::vector<int>& label_vec);

      int num_classes_;

      cv::Mat class_mean_arr;
  };


  class PCA:public SSA
  {
    public:
    PCA(){};
    PCA(cv::Mat& input_data,int& ss_dim);
    virtual ~PCA(){};
    void calcProjMatrix(cv::Mat& data);
  };

  class Eigenfaces:public XFaces
  {
    public:
    Eigenfaces(){};
    virtual ~Eigenfaces(){};

    void init(std::vector<cv::Mat>& img_vec,std::vector<int>& label_vec,int& red_dim);
    void meanCoeffs(cv::Mat& coeffs,std::vector<int>& label_vec,cv::Mat& mean_coeffs);

    protected:
    SubspaceAnalysis::PCA pca_;
  };

  class Fisherfaces:public XFaces
  {
    public:
    Fisherfaces(){};
    virtual ~Fisherfaces(){};

    void init(std::vector<cv::Mat>& img_vec,std::vector<int>& label_vec);
    //void projectToSubspace(cv::Mat& src_mat,cv::Mat& dst_mat,double& DFFS);
    //void meanCoeffs(cv::Mat& coeffs,std::vector<int>& label_vec,cv::Mat& mean_coeffs);
    //void retrieve(std::vector<cv::Mat>& out_eigenvectors,cv::Mat& out_eigenvalues,cv::Mat& out_avg,cv::Mat& out_proj_model_data);
    //void classify(cv::Mat& src_mat,int class_index);

    protected:
    //void calcDIFS(cv::Mat& probe_mat,std::vector<double>& DFFS);
    int num_classes_;
    SubspaceAnalysis::PCA pca_;
    SubspaceAnalysis::LDA lda_;
  };

};
#endif
