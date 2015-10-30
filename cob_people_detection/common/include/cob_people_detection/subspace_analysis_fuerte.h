#ifndef SSA_H_
#define SSA_H_

#include<iostream>
#include<opencv/cv.h>
#include<opencv/highgui.h>
#include<opencv/ml.h>
#include<fstream>
#include<ostream>
#include<limits>

#include<math.h>
// Base class for SubSpace Analysis(SSA)
//
//
namespace SubspaceAnalysis
{

void dump_matrix(cv::Mat& mat, std::string filename);
void condense_labels(std::vector<int>& labels);

void error_prompt(std::string fct, std::string desc);
void unique_elements(std::vector<int> & vec, int& unique_elements, std::vector<int>& distinct_vec);
void unique_elements(cv::Mat & mat, int& unique_elements, std::vector<int>& distinct_vec);

void mat_info(cv::Mat& mat);

template<class T>
void process_labels(std::vector<T> src_vec, cv::Mat& dst_labels)
{

	dst_labels = cv::Mat::zeros(1, src_vec.size(), CV_32FC1);
	dst_labels -= 1;
	bool unique;
	for (int i = 0; i < src_vec.size(); ++i)
	{
		unique = true;
		for (int j = 0; j < dst_labels.rows; j++)
		{
			if (i == 0)
			{
				break;
			}
			else if (src_vec[i] == src_vec[j])
			{
				unique = false;
				dst_labels.at<float>(i) = dst_labels.at<float>(j);
				break;
			}
			else
				continue;
		}
		if (unique == true)
			dst_labels.at<float>(i) = (float)i;
	}

}
enum Classifier
{
	CLASS_DIFS, CLASS_SVM, CLASS_KNN, CLASS_RF
};

enum Method
{
	METH_FISHER, METH_EIGEN, METH_IFLDA, METH_OCV_FISHER
};

//Baseclass for Fisherfaces and Eigenfaces
class XFaces
{
public:
	XFaces() :
		trained(false), svm_trained_(false), knn_trained_(false), rf_trained_(false)
	{
	}
	;

	virtual ~XFaces()
	{
	}
	;
	// TODO: temporary fix - use this one for sqare sized input images
	void retrieve(std::vector<cv::Mat>& out_eigenvectors, cv::Mat& out_eigenvalues, cv::Mat& out_avg, cv::Mat& out_proj_model_data);
	void retrieve(std::vector<cv::Mat>& out_eigenvectors, cv::Mat& out_eigenvalues, cv::Mat& out_avg, cv::Mat& out_proj_model_data, cv::Size output_dim);
	void getModel(cv::Mat& out_eigenvectors, cv::Mat& out_eigenvalues, cv::Mat& out_avg, cv::Mat& out_proj_model_data);
	//void classify(cv::Mat& src_mat,int& class_index);
	void classify(cv::Mat& coeff_arr, Classifier method, int& class_index);
	void projectToSubspace(cv::Mat& probe_mat, cv::Mat& coeff_arr, double& DFFS);
	void releaseModel();
	bool verifyClassification(cv::Mat& sample, int& index);
	bool saveModel(std::string path);
	bool loadModel(cv::Mat& eigenvec_arr, cv::Mat& eigenval_arr, cv::Mat& proj_model, cv::Mat& avg_arr, std::vector<int>& label_vec, bool use_unknown_thresh);
	bool loadModelFromFile(std::string path, bool use_unknown_thresh);
	bool trained;

protected:
	void project(cv::Mat& src_mat, cv::Mat& proj_mat, cv::Mat& avg_mat, cv::Mat& coeff_mat);
	void reconstruct(cv::Mat& coeffs, cv::Mat& proj_mat, cv::Mat& avg, cv::Mat& rec_im);
	void calcDataMat(std::vector<cv::Mat>& input_data, cv::Mat& data_mat);
	void calcDFFS(cv::Mat& orig_mat, cv::Mat& recon_mat, cv::Mat& avg, std::vector<double>& DFFS);
	void calcDIFS(cv::Mat& probe_mat, int& minDIFSindex, double& minDIFS, cv::Mat& minDIFScoeffs);
	void mat2arr(cv::Mat& src_mat, cv::Mat& dst_mat);
	void calc_threshold(cv::Mat& data, double& thresh);
	void calc_threshold(cv::Mat& data, std::vector<cv::Mat>& thresh);

	//data
	int ss_dim_;
	cv::Mat eigenvector_arr_;
	cv::Mat eigenvalue_arr_;
	cv::Mat avg_arr_;
	cv::Mat model_data_arr_;
	cv::Mat proj_model_data_arr_;
	cv::Mat model_label_arr_;

	int num_classes_;
	std::vector<int> unique_labels_;

	std::vector<cv::Mat> thresholds_;
	bool use_unknown_thresh_;
	double thresh_;
	cv::Mat class_centers_;

	//classification flags
	CvSVM svm_;
	bool svm_trained_;

	CvKNearest knn_;
	bool knn_trained_;

	CvRTrees rf_;
	bool rf_trained_;
};

//Baseclass for PCA LDA
class SSA
{

public:
	SSA()
	{
	}
	;

	virtual ~SSA()
	{
	}
	;
	void calcDataMat(std::vector<cv::Mat>& input_data, cv::Mat& data_mat);
	void calcDataMatMean(cv::Mat& data, cv::Mat& mean_row);
	void decompose();
	void decompose(cv::Mat& data_mat);
	void decompose2(cv::Mat& data_mat);
	//Interface methods
	void retrieve(cv::Mat& proj, cv::Mat& avg, cv::Mat& proj_model_data);

	cv::Mat eigenvals;
	cv::Mat eigenvecs;
	cv::Mat mean;
	int ss_dim_;
};

class LDA: public SSA
{

public:
	LDA()
	{
	}
	;
	LDA(cv::Mat& input_data, std::vector<int>& input_labels, int& num_classes, int& ss_dim);
	virtual ~LDA()
	{
	}
	;

	void calcClassMean(cv::Mat& data_mat, std::vector<int>& label_vec, cv::Mat& class_mean_arr, int& num_classes);
	void calcClassMean(cv::Mat& data_mat, cv::Mat& label_mat, cv::Mat& class_mean_arr, int& num_classes);
	virtual void calcProjMatrix(cv::Mat& data_arr, std::vector<int>& label_vec);

	int num_classes_;
	std::vector<int> unique_labels_;

	cv::Mat class_mean_arr;
};

class ILDA: public LDA
{
public:
	ILDA()
	{
	}
	;
	ILDA(cv::Mat& input_data, std::vector<int>& input_labels, int& num_classes, int& ss_dim);
	virtual ~ILDA()
	{
	}
	;

	virtual void calcProjMatrix(cv::Mat& data_arr, std::vector<int>& label_vec);

};

class PCA: public SSA
{
public:
	PCA()
	{
	}
	;
	PCA(cv::Mat& input_data, int& ss_dim);
	virtual ~PCA()
	{
	}
	;
	virtual void calcProjMatrix(cv::Mat& data);
};

class Eigenfaces: public XFaces
{
public:
	Eigenfaces()
	{
	}
	;
	virtual ~Eigenfaces()
	{
	}
	;

	bool init(std::vector<cv::Mat>& img_vec, std::vector<int>& label_vec, int& red_dim);
	void meanCoeffs(cv::Mat& coeffs, std::vector<int>& label_vec, cv::Mat& mean_coeffs);

protected:
	SubspaceAnalysis::PCA pca_;
};

class Fisherfaces: public XFaces
{
public:
	Fisherfaces()
	{
	}
	;
	virtual ~Fisherfaces()
	{
	}
	;

	bool init(std::vector<cv::Mat>& img_vec, std::vector<int>& label_vec);

protected:
	//void calcDIFS(cv::Mat& probe_mat,std::vector<double>& DFFS);
	SubspaceAnalysis::PCA pca_;
	SubspaceAnalysis::LDA lda_;
};
class FishEigFaces: public XFaces
{
public:
	FishEigFaces();
	virtual ~FishEigFaces()
	{
	}
	;
	//old interface TODO change to new
	bool init(std::vector<cv::Mat>& img_vec, std::vector<int>& label_vec, int& red_dim);
	bool init(std::vector<cv::Mat>& img_vec, std::vector<int>& label_vec, int& red_dim, Method method);
	bool init(std::vector<cv::Mat>& img_vec, std::vector<int>& label_vec, int& red_dim, Method method, bool fallback, bool use_unknown_thresh);

	bool trainModel(std::vector<cv::Mat>& img_vec, std::vector<int>& label_vec, int& red_dim);
	bool trainModel(std::vector<cv::Mat>& img_vec, std::vector<int>& label_vec, int& red_dim, Method method);
	bool trainModel(std::vector<cv::Mat>& img_vec, std::vector<int>& label_vec, int& red_dim, Method method, bool fallback, bool use_unknown_thresh);

protected:
	SubspaceAnalysis::PCA pca_;
	SubspaceAnalysis::LDA lda_;
	bool fallback_;
};

}
;
#endif
