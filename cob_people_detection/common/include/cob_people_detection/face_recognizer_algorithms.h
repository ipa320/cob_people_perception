#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <ostream>
#include <limits>

#include <cob_people_detection/subspace_analysis.h>

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

namespace ipa_PeopleDetector
{

enum Classifier
{
	CLASS_DIFS, CLASS_SVM, CLASS_KNN, CLASS_RF
};

enum Method
{
	NONE, METH_FISHER, METH_EIGEN, METH_LDA2D, METH_PCA2D
};

class FaceRecognizerBaseClass
{
public:
	/// Constructor
	FaceRecognizerBaseClass() :
		use_unknown_thresh_(false), trained_(false)
	{
	}
	;

	/// Destructor
	virtual ~FaceRecognizerBaseClass()
	{
	}
	;

	/// Abstract method to train recognition model.
	/// @brief Abstract method for model training.
	/// @param[in] img_vec Vector of image matrices
	/// @param[in] label_vec Vector of labels
	/// @param[in] target_dim Subspace dimension
	/// @return True when training was successful
	virtual bool trainModel(std::vector<cv::Mat>& img_vec, std::vector<int>& label_vec, int& target_dim)=0;

	/// Abstract method to classifiy image.
	/// @brief Abstract method for classification.
	/// @param[in] src_vec Vector of image matrices
	/// @param[in] probe_mat Image that is classified
	/// @param[out] max_prob_index Index of most probable label
	virtual void classifyImage(cv::Mat& probe_mat, int& max_prob_index)=0;

	/// Abstract method to classifiy image and recieve classification
	///  probabilities for all possible classes.
	/// @brief Abstract method for classification.
	/// @param[in] probe_mat Image that is classified
	/// @param[out] max_prob_index Index of most probable label
	/// @param[out]classification_probabilities Classification probabilities
	/// for all classes in dataset
	virtual void classifyImage(cv::Mat& probe_mat, int& max_prob_index, cv::Mat& classification_probabilities)=0;

	/// Abstract method to save recognition model.
	virtual bool saveModel(boost::filesystem::path& model_file)=0;

	/// Abstract method to load recognition model.
	virtual bool loadModel(boost::filesystem::path& model_file)=0;

	///  Method to activate usage of "unknown" threshold
	inline virtual void activate_unknown_treshold()
	{
		use_unknown_thresh_ = true;
	}
	;

	bool trained_; ///< Flag indicates whether model is trained and ready for recognition.
protected:

	/// Abstract method to extract features from image matrix using a linear projection
	///  matrix.
	/// @brief Abstract method feature extraction.
	/// @param[in] src_vec Vector of image matrices
	/// @param[in] proj_mat Linear projection matrix
	/// @param[out] coeff_vec Vector with feature matrices
	virtual void extractFeatures(std::vector<cv::Mat>& src_vec, cv::Mat& proj_mat, std::vector<cv::Mat>& coeff_vec)=0;
	/// Abstract method to extract features from image matrix using a linear projection
	///  matrix.
	/// @brief Abstract method feature extraction.
	/// @param[in] src_vec Vector of image matrices
	/// @param[in] src_mat Image matrix
	/// @param[in] proj_mat Linear projection matrix
	/// @param[out] coeff_vec Feature matrix
	virtual void extractFeatures(cv::Mat& src_mat, cv::Mat& proj_mat, cv::Mat& coeff_mat)=0;

	/// Abstract method to calculate the minimal distance in face space (DIFS) for a given probe image.
	/// @brief Abstract method for DIFS calculation.
	/// @param[in] probe_mat Image which is compared to the model features
	/// @param[out] minDIFSindex Index of the minimal distance
	/// @param[out] minDIFS  Minimal distance
	/// @param[out] probabilities Classification probabilities for all classes in dataset
	virtual void calcDIFS(cv::Mat& probe_mat, int& minDIFSindex, double& minDIFS, cv::Mat& probabilities)=0;

	/// Abstract method for the calculation of the "unknown" threshold
	/// @brief Calculation of unknown threshold.
	/// @param[in] data Matrix containing model features as matrix-rows.
	/// @param[out] thresh Value for "unknown" threshold.
	virtual void calc_threshold(cv::Mat& data, double& thresh)=0;

	/// Abstract method for the calculation of the "unknown" threshold
	/// @brief Calculation of unknown threshold.
	/// @param[in] data Vector containing model features as matrices.
	/// @param[out] thresh Value for "unknown" threshold.
	virtual void calc_threshold(std::vector<cv::Mat>& data, double& thresh)=0;
	/// Method that checks input parameters for filetype and usable
	/// dimensions
	/// @return False when input parameter check detects invalid parameters
	virtual bool input_param_check(std::vector<cv::Mat>& imgs, std::vector<int>& labels, int& target_dim);

	/// Method is used to check whether the unknown threshold is exceeded.
	/// @brief Check whether the unknown threshold is exceeded
	///  @param[in] distance Distance in faces space
	///  @param[in] threshold "unknown" threshold
	/// @return Returns false, when threshold is exceeded.
	inline bool is_known(double& distance, double& threshold)
	{
		if (distance >= threshold)
			return false;
		return true;
	}
	;

	double unknown_thresh_; ///< Unknown threshold. When it is exceeded face is classified as unknown.
	cv::Size source_dim_; ///< Dimensions of the images the model is trained with.
	int target_dim_; ///< Subspace dimension that is used for the facespace.
	int num_classes_; ///< Number of classes
	cv::Mat projection_mat_;///< Linear projection matrix (Eigenvectors) for transition from image space to features space.
	cv::Mat eigenvalues_; ///< Eigenvalues from Eigenvalue decomposition of training set.
	std::vector<int> model_label_vec_; ///< Vector containing labels of training set.
	bool use_unknown_thresh_; ///< When true unknown idendities are considered.
};

class FaceRecognizer1D: public FaceRecognizerBaseClass
{

public:
	FaceRecognizer1D()
	{
	}
	;
	virtual ~FaceRecognizer1D()
	{
	}
	;

	virtual void extractFeatures(std::vector<cv::Mat>& src_vec, cv::Mat& proj_mat, std::vector<cv::Mat>& coeff_vec)
	{
	}
	;
	virtual void extractFeatures(cv::Mat& src_vec, cv::Mat& proj_mat, cv::Mat& coeff_vec);
	virtual void classifyImage(cv::Mat& probe_mat, int& max_prob_index);
	virtual void classifyImage(cv::Mat& probe_mat, int& max_prob_index, cv::Mat& classification_probabilities);
	virtual void calcDIFS(cv::Mat& probe_mat, int& minDIFSindex, double& minDIFS, cv::Mat& probabilities);
	virtual void calc_threshold(cv::Mat& data, double& thresh);
	virtual void calc_threshold(std::vector<cv::Mat>& data, double& thresh)
	{
	}
	;

	virtual bool saveModel(boost::filesystem::path& model_file);
	virtual bool loadModel(boost::filesystem::path& model_file);

	virtual void model_data_mat(std::vector<cv::Mat>& input_data, cv::Mat& data_mat);

protected:
	cv::Mat average_arr_;
	cv::Mat model_features_;

};

class FaceRecognizer2D: public FaceRecognizerBaseClass
{

public:
	virtual void extractFeatures(std::vector<cv::Mat>& src_vec, cv::Mat& proj_mat, std::vector<cv::Mat>& coeff_vec);
	virtual void extractFeatures(cv::Mat& src_vec, cv::Mat& proj_mat, cv::Mat& coeff_vec);
	virtual void classifyImage(cv::Mat& probe_mat, int& max_prob_index, cv::Mat& classification_probabilities);
	virtual void classifyImage(cv::Mat& probe_mat, int& max_prob_index);
	virtual void calcDIFS(cv::Mat& probe_mat, int& minDIFSindex, double& minDIFS, cv::Mat& probabilities);
	virtual void calc_threshold(cv::Mat& data, double& thresh)
	{
	}
	;
	virtual void calc_threshold(std::vector<cv::Mat>& data, double& thresh);

	virtual bool saveModel(boost::filesystem::path& model_file);
	virtual bool loadModel(boost::filesystem::path& model_file);

protected:
	cv::Mat average_mat_;
	std::vector<cv::Mat> model_features_;
};

class FaceRecognizer_Eigenfaces: public FaceRecognizer1D
{
public:
	FaceRecognizer_Eigenfaces()
	{
	}
	;
	virtual ~FaceRecognizer_Eigenfaces()
	{
	}
	;
	virtual bool trainModel(std::vector<cv::Mat>& img_vec, std::vector<int>& label_vec, int& target_dim);
};

class FaceRecognizer_Fisherfaces: public FaceRecognizer1D
{
public:
	FaceRecognizer_Fisherfaces()
	{
	}
	;
	virtual ~FaceRecognizer_Fisherfaces()
	{
	}
	;
	virtual bool trainModel(std::vector<cv::Mat>& img_vec, std::vector<int>& label_vec, int& target_dim);

protected:
	SubspaceAnalysis::LDA lda_;
};

class FaceRecognizer_LDA2D: public FaceRecognizer2D
{
public:
	FaceRecognizer_LDA2D()
	{
	}
	;
	virtual ~FaceRecognizer_LDA2D()
	{
	}
	;
	virtual bool trainModel(std::vector<cv::Mat>& img_vec, std::vector<int>& label_vec, int& target_dim);
};

class FaceRecognizer_PCA2D: public FaceRecognizer2D
{
public:
	FaceRecognizer_PCA2D()
	{
	}
	;
	virtual ~FaceRecognizer_PCA2D()
	{
	}
	;
	virtual bool trainModel(std::vector<cv::Mat>& img_vec, std::vector<int>& label_vec, int& target_dim);
};

}
;
