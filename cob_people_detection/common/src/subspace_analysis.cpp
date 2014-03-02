#include<cob_people_detection/subspace_analysis.h>
#include<thirdparty/decomposition.hpp>

//---------------------------------------------------------------------------------------------------------------------<
//---------------------------------------------------------------------------------------------------------------------<
//---------------------------------------------------------------------------------------------------------------------<
//---------------------------------------------------------------------------------------------------------------------<
//---------------------------------------------------------------------------------------------------------------------<
//---------------------------------------------------------------------------------------------------------------------<
//---------------------------------------------------------------------------------------------------------------------<
void SubspaceAnalysis::mat2arr(cv::Mat& src_mat, cv::Mat& dst_mat)
{

	dst_mat = src_mat.clone().reshape(1, 1);

	return;
}

void SubspaceAnalysis::condense_labels(std::vector<int>& labels)
{
	int min_val = std::numeric_limits<int>::max();
	for (int i = 0; i < labels.size(); i++)
	{
		if (labels[i] < min_val)
			min_val = labels[i];
	}
	if (min_val > 0)
	{
		for (int i = 0; i < labels.size(); i++)
		{
			labels[i] -= min_val;
		}
	}
}
void SubspaceAnalysis::unique_elements(cv::Mat & mat, int& unique_elements, std::vector<int>& distinct_vec)
{
	bool unique = true;
	for (int i = 0; i < mat.total(); ++i)
	{

		if (i != 0)
		{
			unique = true;
			for (int j = 0; j < distinct_vec.size(); j++)
			{
				if (mat.at<float>(i) == distinct_vec[j])
					unique = false;
			}
		}
		if (unique == true)
			distinct_vec.push_back(mat.at<float>(i));
	}
	unique_elements = distinct_vec.size();
}
void SubspaceAnalysis::unique_elements(std::vector<int> & vec, int& unique_elements, std::vector<int>& distinct_vec)
{
	bool unique = true;
	for (int i = 0; i < vec.size(); ++i)
	{

		if (i != 0)
		{
			unique = true;
			for (int j = 0; j < distinct_vec.size(); j++)
			{
				if (vec[i] == distinct_vec[j])
					unique = false;
			}
		}
		if (unique == true)
			distinct_vec.push_back(vec[i]);
	}
	unique_elements = distinct_vec.size();
}

//void SubspaceAnalysis::XFaces::getModel(cv::Mat& out_eigenvectors,cv::Mat& out_eigenvalues,cv::Mat& out_avg,cv::Mat& out_proj_model_data)
//{
//  avg_arr_            .copyTo( out_avg            ) ;
//  proj_model_data_arr_.copyTo( out_proj_model_data) ;
//  eigenvector_arr_    .copyTo( out_eigenvectors   ) ;
//  eigenvalue_arr_     .copyTo( out_eigenvalues    ) ;
//}
//
//
//
//void SubspaceAnalysis::XFaces::releaseModel()
//{
//      num_classes_ =-1;
//      ss_dim_ =-1;
//      svm_trained_=false;
//      knn_trained_=false;
//      eigenvector_arr_.release();
//      eigenvalue_arr_.release();
//      avg_arr_.release();
//      model_data_arr_.release();
//      proj_model_data_arr_.release();
//      model_label_arr_.release();;
//
//
//      CvSVM svm_;
//      CvKNearest knn_;
//
//}
//
//
//
//
//bool SubspaceAnalysis::XFaces::saveModel(std::string path)
//{
//
//  std::cout<<"saving model"<<std::endl;
//
//		if (boost::filesystem::is_regular_file(path.c_str()))
//		{
//			if (boost::filesystem::remove(path.c_str()) == false)
//			{
//
//      error_prompt("saveModel()","old rdata.xml can not be removed");
//				return false;
//			}
//		}
//		cv::FileStorage fileStorage(path.c_str(), cv::FileStorage::WRITE);
//		if(!fileStorage.isOpened())
//		{
//      error_prompt("saveModel()","Output path is invalid");
//			return false;
//		}
//
//    //create string describing model ( 1D or 2D)
//    std::string model_type_string;
//    if(method_==METH_FISHER ||method_==METH_EIGEN)
//    {
//      model_type_string="1D";
//    }
//    if(method_==METH_LDA2D ||method_==METH_PCA2D)
//    {
//      model_type_string="2D";
//    }
//
//    // Modeltype ( 1D or 2D)
//    fileStorage << "modeltype" << model_type_string;
//    // Eigenvectors
//    fileStorage << "eigenvectors" << eigenvector_arr_;
//		// Eigenvalue matrix
//		fileStorage << "eigenvalues" << eigenvalue_arr_;
//
//		// Average image
//		fileStorage << "average_image" << avg_arr_;
//
//		// Projection coefficients of the training faces
//		fileStorage << "projected_model_data" << proj_model_data_arr_;
//
//    fileStorage << "model_data_labels"  << model_label_arr_;
//
//		fileStorage.release();
//
//	return true;
//}
//
//bool SubspaceAnalysis::XFaces::loadModelFromFile(std::string path,bool use_unknown_thresh)
//{
//  //if(this->trained)this->releaseModel();
//
//	// secure this function with a mutex
//
//		cv::FileStorage fileStorage(path.c_str(), cv::FileStorage::READ);
//		if(!fileStorage.isOpened())
//		{
//      error_prompt("loadModelFromFile()","Invalid input file");
//      return false;
//		}
//		else
//		{
//      fileStorage["eigenvectors"] >> eigenvector_arr_;
//      fileStorage["eigenvalues"] >> eigenvalue_arr_;
//      fileStorage["average_image"] >> avg_arr_;
//      fileStorage["projected_model_data"] >> proj_model_data_arr_;
//      fileStorage["model_data_labels"]  >> model_label_arr_;
//
//		}
//		fileStorage.release();
//
//    //TODO keep only a selection instead of whole dataset depending on labels
//
//    use_unknown_thresh_= use_unknown_thresh;
//
//    SubspaceAnalysis::unique_elements(model_label_arr_,num_classes_,unique_labels_);
//
//    ss_dim_=proj_model_data_arr_.cols;
//
//    if(use_unknown_thresh_)
//    {
//    std::cout<<"calculating threshold...";
//    calc_threshold(proj_model_data_arr_,thresh_);
//    std::cout<<"done"<<std::endl;
//    }
//    this->trained=true;
//    std::cout<<"FaceRecognizer --> model loaded successfully\n";
//    return true;
//
//}
//
//bool SubspaceAnalysis::XFaces::loadModel(cv::Mat& eigenvec_arr,cv::Mat& eigenval_arr,cv::Mat& avg_arr,cv::Mat& proj_model,std::vector<int>& label_vec,bool use_unknown_thresh)
//{
//  //check if number of labels equals number of images in training set
//  if(label_vec.size()!=proj_model.rows)
//  {
//    error_prompt("loadModel()","#labels != #rows in projected model data");
//    return false;
//  }
//
//  //if(this->trained)this->releaseModel();
//  eigenvector_arr_=eigenvec_arr;
//  eigenvalue_arr_=eigenval_arr;
//  proj_model_data_arr_=proj_model;
//  avg_arr_=avg_arr;
//
//  use_unknown_thresh_= use_unknown_thresh;
//
//  SubspaceAnalysis::unique_elements(label_vec,num_classes_,unique_labels_);
//  SubspaceAnalysis::condense_labels(label_vec);
//  model_label_arr_=cv::Mat(1,label_vec.size(),CV_32FC1);
//  for(int i=0;i<label_vec.size();i++)
//  {
//    model_label_arr_.at<float>(i)=static_cast<float>(label_vec[i]);
//  }
//
//  ss_dim_=proj_model_data_arr_.cols;
//
//  if(use_unknown_thresh_)
//  {
//  std::cout<<"calculating threshold...";
//  calc_threshold(proj_model_data_arr_,thresh_);
//  std::cout<<"done"<<std::endl;
//  }
//  this->trained=true;
//  std::cout<<"FaceRecognizer --> model loaded successfully\n";
//  return true;
//}


//---------------------------------------------------------------------------------
// SSA
//---------------------------------------------------------------------------------
void SubspaceAnalysis::SSA::calcDataMatMean(cv::Mat& data, cv::Mat& mean_row)
{
	for (int i = 0; i < data.rows; ++i)
	{
		cv::Mat data_row = data.row(i);
		//calculate mean
		cv::add(mean_row, data_row, mean_row);
	}
	mean_row.convertTo(mean_row, CV_64F, 1.0 / static_cast<double>(data.rows));

}

void SubspaceAnalysis::SSA::decomposeAsymmetricMatrix(cv::Mat& data_mat)
{
	EigenvalueDecomposition es(data_mat);
	eigenvals = es.eigenvalues();
	eigenvals = eigenvals.reshape(1, 1).t();
	eigenvecs = es.eigenvectors();
	eigenvecs = eigenvecs.t();

}
void SubspaceAnalysis::SSA::decomposeSVD(cv::Mat& data_mat)
{

	data_mat.convertTo(data_mat, CV_64F, 1 / sqrt(data_mat.rows));
	cv::SVD svd(data_mat.t());
	eigenvecs = svd.u;
	//svd output is transposed
	//svd.u.copyTo(eigenvecs);
	eigenvecs = eigenvecs.t();
	//svd.w.copyTo(eigenvals);
	eigenvals = svd.w;

}

void SubspaceAnalysis::SSA::decomposeSymmetricMatrix(cv::Mat& data_mat)
{

	cv::eigen(data_mat, eigenvals, eigenvecs);

}

//---------------------------------------------------------------------------------
// LDA
//---------------------mean_arr_row--------------------------------------------
SubspaceAnalysis::PCA2D::PCA2D(std::vector<cv::Mat>& input_data, std::vector<int>& input_labels, int& num_classes, int& ss_dim)
{
	SubspaceAnalysis::unique_elements(input_labels, num_classes_, unique_labels_);

	//calculate mean of matrices
	cv::Mat mean = cv::Mat::zeros(input_data[0].rows, input_data[0].cols, CV_64FC1);

	for (int i = 0; i < input_data.size(); i++)
	{
		//add to class mean
		mean += input_data[i];
	}
	// is this a valid matrix op TODO
	mean /= input_data.size();

	// TODO TEMPORARY PCA
	cv::Mat P = cv::Mat::zeros(input_data[0].cols, input_data[0].cols, CV_64FC1);

	for (int i = 0; i < input_data.size(); i++)
	{
		cv::Mat temp;
		cv::subtract(input_data[i], mean, temp);
		cv::mulTransposed(temp, temp, true);
		cv::add(temp, P, P);
	}

	P /= input_data.size();
	decomposeSymmetricMatrix(P);
	eigenvecs = eigenvecs(cv::Rect(0, 0, input_data[0].cols, ss_dim));
	eigenvals = eigenvals(cv::Rect(0, 0, 1, ss_dim)).t();

}
SubspaceAnalysis::LDA2D::LDA2D(std::vector<cv::Mat>& input_data, std::vector<int>& input_labels, int& num_classes, int& ss_dim)
{
	SubspaceAnalysis::unique_elements(input_labels, num_classes_, unique_labels_);
	mean = cv::Mat::zeros(input_data[0].rows, input_data[0].cols, CV_64FC1);

	//calculate mean of matrices
	cv::Mat mean_mat = cv::Mat::zeros(input_data[0].rows, input_data[0].cols, CV_64FC1);
	std::vector<cv::Mat> class_means;
	std::vector<int> class_sizes;

	// resize vectors
	class_means.resize(num_classes_);
	class_sizes.resize(num_classes_);

	// initialize with zero
	for (int i = 0; i < num_classes_; i++)
	{
		class_means[i] = cv::Mat::zeros(input_data[0].rows, input_data[0].cols, CV_64FC1);
		class_sizes[i] = 0;
	}

	for (int i = 0; i < input_data.size(); i++)
	{
		//add to class mean
		class_means[input_labels[i]] += input_data[i];
		mean_mat += input_data[i];
		class_sizes[input_labels[i]]++;
	}
	// is this a valid matrix op TODO
	mean_mat /= input_data.size();

	//  // TODO TEMPORARY PCA
	//  cv::Mat P=cv::Mat::zeros(input_data[0].cols,input_data[0].cols,CV_64FC1);
	//
	//  for(int i=0;i<input_data.size();i++)
	//  {
	//    cv::Mat temp;
	//    cv::subtract(input_data[i],mean_mat,temp);
	//    cv::mulTransposed(temp,temp,true);
	//    cv::add(temp,P,P);
	//  }
	//
	//  P/=input_data.size();
	//  decompose(P);
	//  mean=mean_mat;

	for (int i = 0; i < num_classes_; i++)
	{
		class_means[i] /= class_sizes[i];
	}

	cv::Mat S_intra = cv::Mat::zeros(input_data[0].cols, input_data[0].cols, CV_64FC1);
	cv::Mat S_inter = cv::Mat::zeros(input_data[0].cols, input_data[0].cols, CV_64FC1);
	int class_index;

	for (int i = 0; i < input_data.size(); ++i)
	{
		//reduce data matrix
		class_index = input_labels[i];
		cv::Mat tmp;

		cv::subtract(input_data[i], class_means[class_index], tmp);
		cv::Mat temp;
		cv::mulTransposed(input_data[i], temp, true);
		cv::add(temp, S_intra, S_intra);
	}
	for (int c = 0; c < num_classes_; c++)
	{
		cv::Mat temp;
		class_index = unique_labels_[c];
		cv::subtract(class_means[c], mean_mat, temp);
		cv::mulTransposed(temp, temp, true);
		temp = temp * class_sizes[c];
		cv::add(S_inter, temp, S_inter);
	}

	for (int i = 0; i < input_data.size(); i++)
	{
	}
	// //Intra class scatter

	cv::Mat S_intra_inv = S_intra.inv();

	cv::Mat P;
	gemm(S_intra_inv, S_inter, 1.0, cv::Mat(), 0.0, P);

	decomposeSymmetricMatrix(P);

	eigenvecs = eigenvecs(cv::Rect(0, 0, input_data[0].cols, ss_dim));
	eigenvals = eigenvals(cv::Rect(0, 0, 1, ss_dim)).t();
	//cv::normalize(eigenvecs,eigenvecs);

}
SubspaceAnalysis::LDA::LDA(cv::Mat& input_data, std::vector<int>& input_labels, int& num_classes, int& ss_dim)
{

	SubspaceAnalysis::unique_elements(input_labels, num_classes_, unique_labels_);
	cv::Mat data_work = input_data.clone();
	mean = cv::Mat::zeros(1, data_work.cols, CV_64FC1);
	calcDataMatMean(data_work, mean);
	//class labels have to be in a vector in ascending order - duplicates are
	//removed internally
	//{0,0,1,2,3,3,4,5}

	class_mean_arr = cv::Mat::zeros(num_classes_, input_data.cols, CV_64FC1);
	calcClassMean(data_work, input_labels, class_mean_arr, num_classes_);

	calcProjMatrix(data_work, input_labels);

	eigenvecs = eigenvecs(cv::Rect(0, 0, input_data.cols, ss_dim));
	eigenvals = eigenvals(cv::Rect(0, 0, 1, ss_dim)).t();
	//cv::normalize(eigenvecs,eigenvecs);

}
void SubspaceAnalysis::LDA::calcClassMean(cv::Mat& data_mat, cv::Mat& label_mat, cv::Mat& class_mean_arr, int& num_classes)
{
	std::vector<int> label_vec;
	for (int i = 0; i < label_mat.cols; i++)
	{
		label_vec.push_back((int)label_mat.at<float>(i));
	}

	calcClassMean(data_mat, label_vec, class_mean_arr, num_classes);

}
void SubspaceAnalysis::LDA::calcClassMean(cv::Mat& data_mat, std::vector<int>& label_vec, cv::Mat& class_mean_arr, int& num_classes)
{

	std::vector<int> samples_per_class(num_classes, 0);

	int class_index;
	for (int i = 0; i < data_mat.rows; i++)
	{
		cv::Mat data_row = data_mat.row(i);
		class_index = label_vec[i];
		cv::Mat mean_row = class_mean_arr.row(class_index);

		add(mean_row, data_row, mean_row);
		samples_per_class[class_index]++;
	}

	for (int i = 0; i < num_classes; i++)
	{
		cv::Mat mean_arr_row = class_mean_arr.row(i);
		mean_arr_row.convertTo(mean_arr_row, CV_64FC1, 1.0 / static_cast<double>(samples_per_class[i]));
	}

}

void SubspaceAnalysis::LDA::calcProjMatrix(cv::Mat& data_arr, std::vector<int>& label_vec)
{

	cv::Mat S_intra = cv::Mat::zeros(data_arr.cols, data_arr.cols, CV_64FC1);
	cv::Mat S_inter = cv::Mat::zeros(data_arr.cols, data_arr.cols, CV_64FC1);
	int class_index;

	for (int i = 0; i < data_arr.rows; ++i)
	{
		//reduce data matrix
		class_index = label_vec[i];
		cv::Mat data_row = data_arr.row(i);
		cv::Mat class_mean_row = class_mean_arr.row(class_index);
		cv::subtract(data_row, class_mean_row, data_row);
	}
	for (int c = 0; c < num_classes_; c++)
	{
		cv::Mat temp;
		class_index = unique_labels_[c];
		cv::Mat class_mean_row = class_mean_arr.row(class_index);
		cv::subtract(class_mean_row, mean, temp);
		cv::mulTransposed(temp, temp, true);
		cv::add(S_inter, temp, S_inter);

	}
	// //Intra class scatter
	cv::mulTransposed(data_arr, S_intra, true);

	cv::Mat S_intra_inv = S_intra.inv();

	cv::Mat P;
	gemm(S_intra_inv, S_inter, 1.0, cv::Mat(), 0.0, P);

	decomposeAsymmetricMatrix(P);

	return;

}

//---------------------------------------------------------------------------------
// ILDA
//---------------------------------------------------------------------------------
//
SubspaceAnalysis::ILDA::ILDA(cv::Mat& input_data, std::vector<int>& input_labels, int& num_classes, int& ss_dim)
{
	SubspaceAnalysis::unique_elements(input_labels, num_classes_, unique_labels_);
	cv::Mat data_work = input_data.clone();
	mean = cv::Mat::zeros(1, data_work.cols, CV_64FC1);
	calcDataMatMean(data_work, mean);
	num_classes_ = num_classes;
	//class labels have to be in a vector in ascending order - duplicates are
	//removed internally
	//{0,0,1,2,3,3,4,5}

	class_mean_arr = cv::Mat::zeros(num_classes_, input_data.cols, CV_64FC1);
	calcClassMean(data_work, input_labels, class_mean_arr, num_classes_);

	calcProjMatrix(data_work, input_labels);

	eigenvecs = eigenvecs(cv::Rect(0, 0, input_data.cols, ss_dim));
	eigenvals = eigenvals(cv::Rect(0, 0, 1, ss_dim)).t();
	//cv::normalize(eigenvecs,eigenvecs);
}
void SubspaceAnalysis::ILDA::calcProjMatrix(cv::Mat& data_arr, std::vector<int>& label_vec)
{

	cv::Mat S_intra = cv::Mat::zeros(data_arr.cols, data_arr.cols, CV_64FC1);
	cv::Mat S_inter = cv::Mat::zeros(data_arr.cols, data_arr.cols, CV_64FC1);
	int class_index;

	for (int i = 0; i < data_arr.rows; ++i)
	{
		//reduce data matrix
		class_index = label_vec[i];
		cv::Mat data_row = data_arr.row(i);
		cv::Mat class_mean_row = class_mean_arr.row(class_index);
		cv::subtract(data_row, class_mean_row, data_row);
	}
	for (int c = 0; c < num_classes_; c++)
	{
		cv::Mat temp;
		class_index = unique_labels_[c];
		cv::Mat class_mean_row = class_mean_arr.row(class_index);
		cv::subtract(class_mean_row, mean, temp);
		cv::mulTransposed(temp, temp, true);
		cv::add(S_inter, temp, S_inter);

	}
	// //Intra class scatter
	cv::mulTransposed(data_arr, S_intra, true);
	cv::Mat S_intra_inv = S_intra.inv();
	cv::Mat sigma = cv::Mat(1, num_classes_, CV_64FC1);

	for (int i = 0; i < num_classes_; ++i)
	{
		cv::Mat mu_i = class_mean_arr.row(i);
		for (int j = 0; j < num_classes_; ++j)
		{
			cv::Mat mu_j = class_mean_arr.row(j);

			cv::Mat delta_ij = ((mu_j - mu_i) * S_intra_inv * (mu_j - mu_i).t());
			for (int k = 0; k < data_arr.rows; k++)
			{

			}
			sigma.at<double>(j) = 1 / (delta_ij.at<double>(0, 0));
		}

	}

	for (int j = 0; j < num_classes_; j++)
	{
		class_index = label_vec[j];
		cv::Mat s_intra_row = S_intra.row(j);
		cv::Mat s_inter_row = S_inter.row(j);
		double sigma_j = sigma.at<double>(class_index);
		s_intra_row *= sigma_j;
		s_inter_row *= sigma_j;
	}

	S_intra_inv = S_intra.inv();

	cv::Mat P;
	gemm(S_intra_inv, S_inter, 1.0, cv::Mat(), 0.0, P);

	decomposeAsymmetricMatrix(P);

	return;

}
//---------------------------------------------------------------------------------
// PCA
//---------------------------------------------------------------------------------
//
SubspaceAnalysis::PCA::PCA(cv::Mat& input_data, int& ss_dim)
{

	ss_dim_ = ss_dim;
	PCA_OpenCv(input_data, ss_dim);
	return;
	cv::Mat data_work = input_data.clone();
	mean = cv::Mat::zeros(1, data_work.cols, CV_64FC1);
	calcDataMatMean(data_work, mean);
	calcProjMatrix(data_work);
	//truncate eigenvectors and eigenvals
	eigenvecs = eigenvecs(cv::Rect(0, 0, input_data.cols, ss_dim));
	eigenvals = eigenvals(cv::Rect(0, 0, 1, ss_dim)).t();
	//cv::normalize(eigenvecs,eigenvecs);

	//cv::Mat dummy;
	//eigenvecs.copyTo(dummy);
	//dummy.convertTo(dummy,CV_8UC1,1000);
	//dummy =dummy.reshape(1,dummy.rows*160);
	//cv::equalizeHist(dummy,dummy);
	//cv::imwrite("/home/goa-tz/Desktop/eigenfaces.jpg",dummy);


}
void SubspaceAnalysis::PCA::PCA_OpenCv(cv::Mat& input_data, int& ss_dim)
{
	ss_dim_ = ss_dim;
	cv::PCA ocv_pca(input_data, cv::Mat(), CV_PCA_DATA_AS_ROW, ss_dim);

	eigenvecs = ocv_pca.eigenvectors.clone();
	eigenvals = ocv_pca.eigenvalues.clone();
	mean = ocv_pca.mean.reshape(1, 1).clone();

	//truncate eigenvectors and eigenvals
	eigenvecs = eigenvecs(cv::Rect(0, 0, input_data.cols, ss_dim));
	eigenvals = eigenvals(cv::Rect(0, 0, 1, ss_dim)).t();
	//cv::normalize(eigenvecs,eigenvecs);

}

void SubspaceAnalysis::PCA::calcProjMatrix(cv::Mat& data)
{
	cv::Mat data_row;
	for (int i = 0; i < data.rows; ++i)
	{
		//reduce data matrix - total Scatter matrix
		data_row = data.row(i);
		cv::subtract(data_row, mean, data_row);
	}

	decomposeSVD(data);
}

