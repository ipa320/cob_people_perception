#include<cob_people_detection/face_recognizer_algorithms.h>

bool ipa_PeopleDetector::FaceRecognizerBaseClass::input_param_check(std::vector<cv::Mat>& imgs, std::vector<int>& labels, int& target_dim)
{
	if (imgs.size() != labels.size())
		return false;
	if (imgs.size() == 0)
		return false;
	if (labels.size() == 0)
		return false;
	if (target_dim == 0)
		return false;
	//if(target_dim>imgs.size()) return false;

}
void ipa_PeopleDetector::FaceRecognizer1D::calc_threshold(cv::Mat& data, double& thresh)
{
	thresh = std::numeric_limits<double>::max();
	std::vector<double> P(num_classes_, std::numeric_limits<double>::max());
	std::vector<double> D(num_classes_, std::numeric_limits<double>::min());
	std::vector<double> Phi(num_classes_, std::numeric_limits<double>::max());

	for (int i = 0; i < data.rows; i++)
	{
		cv::Mat i_row = data.row(i);
		for (int n = 0; n < data.rows; n++)
		{
			if (n == i)
				continue;
			cv::Mat n_row = data.row(n);
			double dist = cv::norm(i_row, n_row, cv::NORM_L2);
			if (model_label_vec_[n] == model_label_vec_[i])
			{
				D[model_label_vec_[i]] = std::max(dist, D[model_label_vec_[i]]);
			}
			else
			{
				P[model_label_vec_[i]] = std::min(dist, P[model_label_vec_[i]]);
			}
		}
	}

	// if only one class - P =D
	if (num_classes_ == 1)
	{
		P[0] = D[0];
	}

	for (int c = 0; c < num_classes_; c++)
	{
		thresh = std::min(thresh, (P[c] + D[c]) * 0.5);
	}
	std::cout << "THRESH for db: " << thresh << std::endl;

}

void ipa_PeopleDetector::FaceRecognizer1D::model_data_mat(std::vector<cv::Mat>& input_data, cv::Mat& data_mat)
{

	// convert input to data matrix,with images as rows

	for (int i = 0; i < input_data.size(); i++)
	{
		cv::Mat src_mat;
		src_mat = input_data[i];
		src_mat = src_mat.reshape(1, 1);
		cv::Mat dst_row = data_mat.row(i);
		src_mat.copyTo(dst_row);
	}

	return;
}

void ipa_PeopleDetector::FaceRecognizer1D::extractFeatures(cv::Mat& src_mat, cv::Mat& proj_mat, cv::Mat& coeff_mat)
{

	//projection
	cv::gemm(src_mat, proj_mat, 1.0, cv::Mat(), 0.0, coeff_mat, cv::GEMM_2_T);

}

void ipa_PeopleDetector::FaceRecognizer1D::classifyImage(cv::Mat& probe_mat, int& max_prob_index)
{
	cv::Mat classification_probabilities;
	classifyImage(probe_mat, max_prob_index, classification_probabilities);
}
void ipa_PeopleDetector::FaceRecognizer1D::classifyImage(cv::Mat& probe_mat, int& max_prob_index, cv::Mat& classification_probabilities)
{

	//project query mat to feature space
	cv::Mat feature_arr = cv::Mat(1, target_dim_, CV_64FC1);
	// conversion from matrix format to array
	cv::Mat probe_arr = cv::Mat(1, probe_mat.total(), probe_mat.type());
	SubspaceAnalysis::mat2arr(probe_mat, probe_arr);

	extractFeatures(probe_arr, projection_mat_, feature_arr);

	//calculate distance in face space DIFS
	double minDIFS;
	cv::Mat minDIFScoeffs;
	int minDIFSindex;
	calcDIFS(feature_arr, minDIFSindex, minDIFS, classification_probabilities);
	max_prob_index = (int)model_label_vec_[minDIFSindex];

	//check whether unknown threshold is exceeded
	if (use_unknown_thresh_)
	{
		if (!is_known(minDIFS, unknown_thresh_))
			max_prob_index = -1;
	}
	return;
}

void ipa_PeopleDetector::FaceRecognizer1D::calcDIFS(cv::Mat& probe_mat, int& minDIFSindex, double& minDIFS, cv::Mat& probabilities)
{

	double norm;
	minDIFS = std::numeric_limits<float>::max();
	probabilities = cv::Mat(1, num_classes_, CV_64FC1);
	probabilities *= std::numeric_limits<float>::max();
	for (int r = 0; r < model_features_.rows; r++)
	{
		cv::Mat model_mat = model_features_.row(r);

		//calculate L2 norm between probe amd all model mats
		norm = cv::norm(probe_mat, model_mat, cv::NORM_L2);

		// update minimum distance and index if required
		if (norm < minDIFS)
		{
			minDIFSindex = r;
			minDIFS = norm;
		}
		//calculate cost for classification to every class in database
	}

	//process class_cost
	double min_cost, max_cost;
	probabilities = 1 / (probabilities.mul(probabilities));
	cv::minMaxLoc(probabilities, &min_cost, &max_cost, 0, 0);
	probabilities /= max_cost;

	return;
}

void ipa_PeopleDetector::FaceRecognizer2D::classifyImage(cv::Mat& probe_mat, int& max_prob_index)
{
	cv::Mat classification_probabilities;
	classifyImage(probe_mat, max_prob_index, classification_probabilities);
}
void ipa_PeopleDetector::FaceRecognizer2D::classifyImage(cv::Mat& probe_mat, int& max_prob_index, cv::Mat& classification_probabilities)
{
	//if((int)probe_mat.rows!=(int)source_dim_.height || (int)probe_mat.cols !=(int)source_dim_.width)
	//    {
	//    std::cout<<"[FaceRecognizerAlgorithm] Invalid image dimensions for classification.Aborting."<<std::endl;
	//    }

	//project query mat to feature space

	cv::Mat feature_mat;
	extractFeatures(probe_mat, projection_mat_, feature_mat);

	//calculate distance in face space DIFS
	double minDIFS;
	cv::Mat minDIFScoeffs;
	int minDIFSindex;
	calcDIFS(feature_mat, minDIFSindex, minDIFS, classification_probabilities);
	max_prob_index = (int)model_label_vec_[minDIFSindex];

	//check whether unknown threshold is exceeded
	if (use_unknown_thresh_)
	{
		if (!is_known(minDIFS, unknown_thresh_))
			max_prob_index = -1;
	}
	return;
}
void ipa_PeopleDetector::FaceRecognizer2D::extractFeatures(std::vector<cv::Mat>& src_vec, cv::Mat& proj_mat, std::vector<cv::Mat>& coeff_mat_vec)
{
	//calculate coefficients
	for (int i = 0; i < src_vec.size(); i++)
	{
		cv::Mat src_mat = cv::Mat(src_vec[0].rows, src_vec[0].cols, CV_64FC1);
		src_vec[i].convertTo(src_mat, CV_64FC1);
		cv::Mat coeff_mat = cv::Mat(src_mat.rows, src_mat.cols, CV_64FC1);
		cv::gemm(src_mat, proj_mat, 1.0, cv::Mat(), 0.0, coeff_mat, cv::GEMM_2_T);
		coeff_mat_vec[i] = coeff_mat;
	}
}
void ipa_PeopleDetector::FaceRecognizer2D::extractFeatures(cv::Mat& src_mat, cv::Mat& proj_mat, cv::Mat& coeff_mat)
{

	coeff_mat = cv::Mat(target_dim_, target_dim_, CV_64FC1);
	cv::gemm(src_mat, proj_mat, 1.0, cv::Mat(), 0.0, coeff_mat, cv::GEMM_2_T);
}

void ipa_PeopleDetector::FaceRecognizer2D::calcDIFS(cv::Mat& probe_mat, int& minDIFSindex, double& minDIFS, cv::Mat& probabilities)
{
	minDIFS = std::numeric_limits<double>::max();
	probabilities = cv::Mat(1, num_classes_, CV_32FC1);
	probabilities *= std::numeric_limits<float>::max();
	for (int m = 0; m < model_features_.size(); m++)
	{
		// subtract matrices
		cv::Mat work_mat;
		cv::subtract(probe_mat, model_features_[m], work_mat);
		cv::pow(work_mat, 2, work_mat);
		cv::Mat tmp_vec = cv::Mat::zeros(1, probe_mat.cols, CV_64FC1);
		cv::reduce(work_mat, tmp_vec, 0, CV_REDUCE_SUM);

		cv::Scalar norm = cv::sum(tmp_vec);

		if ((double)norm[0] < minDIFS)
		{
			minDIFSindex = m;
			minDIFS = (double)norm[0];
		}
		//calculate cost for classification to every class in database
		probabilities.at<float>(model_label_vec_[m]) = std::min(probabilities.at<float>(model_label_vec_[m]), (float)norm[0]);
	}

	//process class_cost
	double min_cost, max_cost;
	probabilities = 1 / (probabilities.mul(probabilities));
	cv::minMaxLoc(probabilities, &min_cost, &max_cost, 0, 0);
	probabilities /= max_cost;

	return;
}

void ipa_PeopleDetector::FaceRecognizer2D::calc_threshold(std::vector<cv::Mat>& data, double& thresh)
{
	thresh = std::numeric_limits<double>::max();
	std::vector<double> P(num_classes_, std::numeric_limits<double>::max());
	std::vector<double> D(num_classes_, std::numeric_limits<double>::min());
	std::vector<double> Phi(num_classes_, std::numeric_limits<double>::max());

	for (int i = 0; i < data.size(); i++)
	{
		for (int n = 0; n < data.size(); n++)
		{
			if (n == i)
				continue;

			// subtract matrices
			cv::Mat work_mat = cv::Mat(data[0].rows, data[0].cols, CV_64FC1);
			cv::subtract(data[i], data[n], work_mat);
			cv::pow(work_mat, 2, work_mat);
			//cv::sqrt(work_mat,work_mat);
			cv::Mat tmp_vec = cv::Mat::zeros(1, data[i].cols, CV_64FC1);
			cv::reduce(work_mat, tmp_vec, 0, CV_REDUCE_SUM);

			cv::Scalar temp = cv::sum(tmp_vec);
			double dist = temp.val[0];
			if (model_label_vec_[n] == model_label_vec_[i])
			{
				D[model_label_vec_[i]] = std::max(dist, D[model_label_vec_[i]]);
			}
			else
			{
				P[model_label_vec_[i]] = std::min(dist, P[model_label_vec_[i]]);
			}
		}
	}

	// if only one class - P =D
	if (num_classes_ == 1)
	{
		P[0] = D[0];
	}

	for (int c = 0; c < num_classes_; c++)
	{
		thresh = std::min(thresh, (P[c] + D[c]) * 0.2);
	}
	std::cout << "THRESH for db: " << thresh << std::endl;
}

bool ipa_PeopleDetector::FaceRecognizer1D::loadModel(boost::filesystem::path& model_file)
{

	//TODO:assert file is regular file
	//
	std::cout << "FaceRecognizer1D::loadModel() from " << model_file.string() << std::endl;
	cv::FileStorage fs(model_file.string(), cv::FileStorage::READ);

	fs["projection_matrix"] >> projection_mat_;
	fs["eigenvalues"] >> eigenvalues_;
	fs["unknown_threshold"] >> unknown_thresh_;
	fs["average_image"] >> average_arr_;
	fs["model_features"] >> model_features_;

	// load model labels
	cv::FileNode fn = fs["numeric_labels"];
	cv::FileNodeIterator it = fn.begin(), it_end = fn.end();
	int idx = 0;
	model_label_vec_.resize(model_features_.rows);
	for (; it != it_end; ++it, idx++)
	{
		model_label_vec_[idx] = (int)(*it);
	}

	target_dim_ = model_features_.cols;
	trained_ = true;

}

bool ipa_PeopleDetector::FaceRecognizer1D::saveModel(boost::filesystem::path& model_file)
{

	std::cout << "FaceRecognizer2D::saveModel() to " << model_file.string() << std::endl;
	cv::FileStorage fs(model_file.string(), cv::FileStorage::WRITE);

	fs << "projection_matrix" << projection_mat_;
	fs << "eigenvalues" << eigenvalues_;
	fs << "unknown_threshold" << unknown_thresh_;
	fs << "average_image" << average_arr_;
	fs << "model_features" << model_features_;
	fs << "numeric_labels" << "[";
	for (int i = 0; i < model_label_vec_.size(); i++)
	{
		fs << model_label_vec_[i];
	}
	fs << "]";
	fs.release();

}

bool ipa_PeopleDetector::FaceRecognizer2D::loadModel(boost::filesystem::path& model_file)
{

	////TODO:assert file is regular file
	////
	std::cout << "FaceRecognizer2D::loadModel() from " << model_file.string() << std::endl;
	cv::FileStorage fs(model_file.string(), cv::FileStorage::READ);

	fs["projection_matrix"] >> projection_mat_;
	fs["eigenvalues"] >> eigenvalues_;
	fs["unknown_threshold"] >> unknown_thresh_;
	//fs["average_image"]>>average_mat_;

	//load model features
	cv::FileNode fnm = fs["model_features"];
	cv::FileNodeIterator itm = fnm.begin(), itm_end = fnm.end();
	int idm = 0;
	for (; itm != itm_end; ++itm, idm++)
	{
		cv::Mat temp;
		(*itm) >> temp;
		model_features_.push_back(temp);
	}

	// load model labels
	cv::FileNode fn = fs["numeric_labels"];
	cv::FileNodeIterator it = fn.begin(), it_end = fn.end();
	int idx = 0;
	model_label_vec_.resize(model_features_.size());
	for (; it != it_end; ++it, idx++)
	{
		model_label_vec_[idx] = (int)(*it);
	}

	target_dim_ = model_features_[0].cols;
	trained_ = true;

}

bool ipa_PeopleDetector::FaceRecognizer2D::saveModel(boost::filesystem::path& model_file)
{

	std::cout << "FaceRecognizer1D::saveModel() to " << model_file.string() << std::endl;
	cv::FileStorage fs(model_file.string(), cv::FileStorage::WRITE);

	fs << "projection_matrix" << projection_mat_;
	fs << "eigenvalues" << eigenvalues_;
	fs << "unknown_threshold" << unknown_thresh_;
	fs << "average_image" << average_mat_;

	fs << "model_features" << "[";
	for (int i = 0; i < model_features_.size(); i++)
		fs << model_features_[i];
	fs << "]";

	fs << "numeric_labels" << "[";
	for (int i = 0; i < model_label_vec_.size(); i++)
	{
		fs << model_label_vec_[i];
	}
	fs << "]";
	fs.release();

}
bool ipa_PeopleDetector::FaceRecognizer_Eigenfaces::trainModel(std::vector<cv::Mat>& img_vec, std::vector<int>& label_vec, int& target_dim)
{

	input_param_check(img_vec, label_vec, target_dim);

	std::cout << "Training Eigenfaces" << std::endl;
	std::vector<int> unique_labels;
	SubspaceAnalysis::unique_elements(label_vec, num_classes_, unique_labels);
	SubspaceAnalysis::condense_labels(label_vec);

	SubspaceAnalysis::PCA PCA;

	if (target_dim > num_classes_)
		target_dim_ = num_classes_;
	else
		target_dim_ = target_dim;

	//allocate all matrices
	cv::Mat model_data_arr = cv::Mat(img_vec.size(), img_vec[0].total(), CV_64FC1);
	model_label_vec_.resize(img_vec.size());
	average_arr_ = cv::Mat(1, img_vec[0].total(), CV_64FC1);
	projection_mat_ = cv::Mat(target_dim_, img_vec[0].total(), CV_64FC1);
	eigenvalues_ = cv::Mat(1, target_dim_ - 1, CV_64FC1);
	model_features_ = cv::Mat(model_data_arr.rows, target_dim_, CV_64FC1);

	model_label_vec_ = label_vec;
	model_data_mat(img_vec, model_data_arr);
	//initiate PCA
	PCA = SubspaceAnalysis::PCA(model_data_arr, target_dim_);

	//Assign model to member variables
	projection_mat_ = PCA.eigenvecs;
	eigenvalues_ = PCA.eigenvals;
	average_arr_ = PCA.mean;

	extractFeatures(model_data_arr, projection_mat_, model_features_);

	calc_threshold(model_features_, unknown_thresh_);

	// set FaceRecognizer to trained
	this->trained_ = true;
	return true;
}

bool ipa_PeopleDetector::FaceRecognizer_Fisherfaces::trainModel(std::vector<cv::Mat>& img_vec, std::vector<int>& label_vec, int& target_dim)
{
	input_param_check(img_vec, label_vec, target_dim);

	std::cout << "Training Fisherfaces" << std::endl;
	std::vector<int> unique_labels;
	SubspaceAnalysis::unique_elements(label_vec, num_classes_, unique_labels);
	// check if suitable number of classes is contained in training data
	if (num_classes_ < 2)
	{
		std::cout << "[FaceRecognizerAlgorithm] Fisherfaces needs more than single class in training data.Aborting." << std::endl;
		return false;
	}
	SubspaceAnalysis::condense_labels(label_vec);

	SubspaceAnalysis::PCA PCA;
	SubspaceAnalysis::LDA LDA;

	// set target dimensions for subspace methods

	target_dim_ = num_classes_ - 1;
	int target_dim_PCA = label_vec.size() - num_classes_;
	if (target_dim_PCA < 1)
		target_dim_PCA = num_classes_;

	//allocate all matrices
	cv::Mat model_data_arr = cv::Mat(img_vec.size(), img_vec[0].total(), CV_64FC1);
	model_label_vec_.resize(img_vec.size());
	average_arr_ = cv::Mat(1, img_vec[0].total(), CV_64FC1);
	projection_mat_ = cv::Mat(target_dim_, img_vec[0].total(), CV_64FC1);
	eigenvalues_ = cv::Mat(1, target_dim_ - 1, CV_64FC1);
	model_features_ = cv::Mat(model_data_arr.rows, target_dim_, CV_64FC1);

	// local matrices for PCA
	cv::Mat P_PCA = cv::Mat(target_dim_PCA, img_vec[0].total(), CV_64FC1);
	cv::Mat model_features_PCA = cv::Mat(model_data_arr.rows, target_dim_PCA, CV_64FC1);

	// local matrices for LDA
	cv::Mat P_LDA = cv::Mat(target_dim_, target_dim_PCA, CV_64FC1);

	model_label_vec_ = label_vec;
	model_data_mat(img_vec, model_data_arr);

	//initiate PCA
	PCA = SubspaceAnalysis::PCA(model_data_arr, target_dim_PCA);
	P_PCA = PCA.eigenvecs;

	extractFeatures(model_data_arr, P_PCA, model_features_PCA);

	//perform LDA
	LDA = SubspaceAnalysis::LDA(model_features_PCA, model_label_vec_, num_classes_, target_dim_);
	P_LDA = LDA.eigenvecs;

	// combine projection matrices
	cv::gemm(P_PCA.t(), P_LDA.t(), 1.0, cv::Mat(), 0.0, projection_mat_);

	//Assign model to member variables
	projection_mat_ = projection_mat_.t();
	eigenvalues_ = LDA.eigenvals;
	average_arr_ = PCA.mean;

	extractFeatures(model_data_arr, projection_mat_, model_features_);

	calc_threshold(model_features_, unknown_thresh_);

	// set FaceRecognizer to trained
	this->trained_ = true;
	return true;
}

bool ipa_PeopleDetector::FaceRecognizer_PCA2D::trainModel(std::vector<cv::Mat>& img_vec, std::vector<int>& label_vec, int& target_dim)
{

	input_param_check(img_vec, label_vec, target_dim);

	std::cout << "Training PCA2D" << std::endl;
	std::vector<int> unique_labels;
	SubspaceAnalysis::unique_elements(label_vec, num_classes_, unique_labels);
	SubspaceAnalysis::condense_labels(label_vec);

	//SubspaceAnalysis::PCA2D PCA;

	target_dim_ = target_dim;

	//allocate all matrices
	model_label_vec_.resize(img_vec.size());
	average_mat_ = cv::Mat(img_vec[0].rows, img_vec[0].cols, CV_64FC1);
	projection_mat_ = cv::Mat(target_dim_, img_vec[0].cols, CV_64FC1);
	eigenvalues_ = cv::Mat(1, target_dim_ - 1, CV_64FC1);
	model_features_.resize(img_vec.size());

	model_label_vec_ = label_vec;
	//initiate PCA
	SubspaceAnalysis::PCA2D PCA2D(img_vec, model_label_vec_, num_classes_, target_dim_);

	//Assign model to member variables
	projection_mat_ = PCA2D.eigenvecs;
	eigenvalues_ = PCA2D.eigenvals;
	average_mat_ = PCA2D.mean;

	extractFeatures(img_vec, projection_mat_, model_features_);
	calc_threshold(model_features_, unknown_thresh_);

	// set FaceRecognizer to trained
	this->trained_ = true;
	return true;

}

bool ipa_PeopleDetector::FaceRecognizer_LDA2D::trainModel(std::vector<cv::Mat>& img_vec, std::vector<int>& label_vec, int& target_dim)
{

	input_param_check(img_vec, label_vec, target_dim);

	std::cout << "Training LDA2D" << std::endl;
	std::vector<int> unique_labels;
	SubspaceAnalysis::unique_elements(label_vec, num_classes_, unique_labels);
	SubspaceAnalysis::condense_labels(label_vec);

	if (num_classes_ < 2)
	{
		std::cout << "[FaceRecognizerAlgorithm] LDA 2D needs more than single class in training data.Aborting." << std::endl;
		return false;
	}

	//SubspaceAnalysis::PCA2D PCA;

	source_dim_ = cv::Size(img_vec[0].rows, img_vec[0].cols);
	target_dim_ = target_dim;

	//allocate all matrices
	model_label_vec_.resize(img_vec.size());
	average_mat_ = cv::Mat(img_vec[0].rows, img_vec[0].cols, CV_64FC1);
	projection_mat_ = cv::Mat(target_dim_, img_vec[0].cols, CV_64FC1);
	eigenvalues_ = cv::Mat(1, target_dim_ - 1, CV_64FC1);
	model_features_.resize(img_vec.size());

	model_label_vec_ = label_vec;
	for (int i = 0; i < model_label_vec_.size(); i++)
	{
		std::cout << model_label_vec_[i];
	}
	//initiate PCA
	SubspaceAnalysis::LDA2D LDA2D(img_vec, model_label_vec_, num_classes_, target_dim_);

	//Assign model to member variables
	projection_mat_ = LDA2D.eigenvecs;
	eigenvalues_ = LDA2D.eigenvals;
	average_mat_ = LDA2D.mean;

	extractFeatures(img_vec, projection_mat_, model_features_);
	calc_threshold(model_features_, unknown_thresh_);

	// set FaceRecognizer to trained
	this->trained_ = true;
	return true;
}
