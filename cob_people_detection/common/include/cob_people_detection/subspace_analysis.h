#ifndef SSA_H_
#define SSA_H_

#include<opencv/cv.h>
#include<iostream>
#include<opencv/highgui.h>
#include<opencv/ml.h>
#include<fstream>
#include<ostream>
#include<limits>

#include<math.h>
namespace SubspaceAnalysis
{

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
	void decomposeSVD(cv::Mat& data_mat);
	void decomposeSymmetricMatrix(cv::Mat& data_mat);
	void decomposeAsymmetricMatrix(cv::Mat& data_mat);
	//Interface methods
	void retrieve(cv::Mat& proj, cv::Mat& avg, cv::Mat& proj_model_data);

	cv::Mat eigenvals;
	cv::Mat eigenvecs;
	cv::Mat mean;
	int ss_dim_;
};

class PCA2D: public SSA
{

public:
	PCA2D()
	{
	}
	;
	PCA2D(std::vector<cv::Mat>& input_data, std::vector<int>& input_labels, int& num_classes, int& ss_dim);
	virtual ~PCA2D()
	{
	}
	;

	int num_classes_;
	std::vector<int> unique_labels_;

	cv::Mat class_mean_arr;
};
class LDA2D: public SSA
{

public:
	LDA2D()
	{
	}
	;
	LDA2D(std::vector<cv::Mat>& input_data, std::vector<int>& input_labels, int& num_classes, int& ss_dim);
	virtual ~LDA2D()
	{
	}
	;

	int num_classes_;
	std::vector<int> unique_labels_;

	cv::Mat class_mean_arr;
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
	void PCA_OpenCv(cv::Mat& input_data, int& ss_dim);
};

// Base class for SubSpace Analysis(SSA)
//
//
//
//
//
//
void dump_matrix(cv::Mat& mat, std::string filename);
void condense_labels(std::vector<int>& labels);

void error_prompt(std::string fct, std::string desc);
void unique_elements(std::vector<int> & vec, int& unique_elements, std::vector<int>& distinct_vec);
void unique_elements(cv::Mat & mat, int& unique_elements, std::vector<int>& distinct_vec);

void mat_info(cv::Mat& mat);
void mat2arr(cv::Mat& src_mat, cv::Mat& dst_mat);

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

}
;
#endif
