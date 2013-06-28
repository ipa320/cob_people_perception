#include<cob_people_detection/subspace_analysisNEW.h>

#include "boost/filesystem/operations.hpp"
#include "boost/filesystem/convenience.hpp"
#include "boost/filesystem/path.hpp"
#include <boost/thread/mutex.hpp>
#include<thirdparty/decomposition.hpp>


void SubspaceAnalysis::FaceRecognizer1D::calc_threshold(cv::Mat& data,double& thresh)
{
  std::vector<double> P(num_classes_,std::numeric_limits<double>::max());
  std::vector<double> D(num_classes_,std::numeric_limits<double>::min());
  std::vector<double> Phi(num_classes_,std::numeric_limits<double>::max());

  for(int i=0;i<data.rows;i++)
  {
    cv::Mat i_row=data.row(i);
    for(int n=0;n<data.rows;n++)
    {
      if(n==i) continue;
      cv::Mat n_row=data.row(n);
      double dist = cv::norm(i_row,n_row,cv::NORM_L2);
      if(model_label_vec_[n]==model_label_vec_[i])
      {
        D[model_label_vec_[i]]=std::max(dist,D[model_label_vec_[i]]);
      }
      else
      {
        P[model_label_vec_[i]]=std::min(dist,P[model_label_vec_[i]]);
      }
    }
  }

  // if only one class - P =D
  if(num_classes_==1)
  {
    P[0]=D[0];
  }

  for(int c=0;c<num_classes_;c++)
  {
    thresh=std::min(thresh,(P[c]+D[c]) *0.5);
  }
  std::cout<<"THRESH for db: "<<thresh<<std::endl;

}


void SubspaceAnalysis::FaceRecognizer1D::model_data_mat(std::vector<cv::Mat>& input_data,cv::Mat& data_mat)
{

  // convert input to data matrix,with images as rows

    for(int i = 0;i<input_data.size();i++)
    {
    cv::Mat src_mat;
      src_mat=input_data[i];
      src_mat=src_mat.reshape(1,1);
      cv::Mat dst_row=data_mat.row(i);
      src_mat.copyTo(dst_row);
      }

  return;
}

void SubspaceAnalysis::FaceRecognizer1D::extractFeatures(cv::Mat& src_mat,cv::Mat& proj_mat,cv::Mat& coeff_mat)
{


  //projection
  cv::gemm(src_mat,proj_mat,1.0,cv::Mat(),0.0,coeff_mat,cv::GEMM_2_T);

}

void SubspaceAnalysis::FaceRecognizer1D::classifyImage(cv::Mat& probe_mat,int& max_prob_index,cv::Mat& classification_probabilities)
{

  if(this->trained_ ==false)
  {
    std::cout<<"[FaceRecognizer] Model not trained - aborting classify"<<std::endl;
    return;
  }

  if(num_classes_<2)
  {
    std::cout<<"[FaceRecognizer] Less than 2 classes in model - aborting classify"<<std::endl;
    return;
  }
  //project query mat to feature space
  cv::Mat feature_arr=cv::Mat(1,target_dim_,CV_64FC1);
  // conversion from matrix format to array
  cv::Mat probe_arr=cv::Mat(1,probe_mat.total(),probe_mat.type());
  mat2arr(probe_mat,probe_arr);

  extractFeatures(probe_arr,projection_mat_,feature_arr);

  //calculate distance in face space DIFS
  double minDIFS;
  cv::Mat minDIFScoeffs;
  int minDIFSindex;
  calcDIFS(feature_arr,minDIFSindex,minDIFS,classification_probabilities);
  max_prob_index=(int)model_label_vec_[minDIFSindex];

  //check whether unknown threshold is exceeded
  if(use_unknown_thresh_)
  {
    if(! is_known(minDIFS,unknown_thresh_))max_prob_index=-1;
  }
  return;
}


void SubspaceAnalysis::FaceRecognizer1D::calcDIFS(cv::Mat& probe_mat,int& minDIFSindex,double& minDIFS,cv::Mat& probabilities)
{
    double norm;
    minDIFS=std::numeric_limits<int>::max();
    probabilities *=  std::numeric_limits<float>::max();
      for(int r=0;r<model_features_.rows;r++)
      {
        cv::Mat model_mat=model_features_.row(r);

        //calculate L2 norm between probe amd all model mats
        norm=cv::norm(probe_mat,model_mat,cv::NORM_L2);

        // update minimum distance and index if required
        if(norm < minDIFS )
        {
          minDIFSindex=r;
          minDIFS=norm;
        }
        //calculate cost for classification to every class in database
        probabilities.at<float>(model_label_vec_[r])=std::min(probabilities.at<float>(model_label_vec_[r]),(float)norm);
      }

    //process class_cost
    double min_cost,max_cost;
    probabilities=1/(probabilities.mul(probabilities));
    cv::minMaxLoc(probabilities,&min_cost,&max_cost,0,0);
    probabilities/=max_cost;

    return;
}

void SubspaceAnalysis::FaceRecognizer_Eigenfaces::trainModel(std::vector<cv::Mat>& img_vec,std::vector<int>& label_vec,int& target_dim)
{
        SubspaceAnalysis::unique_elements(label_vec,num_classes_,unique_labels_);
        SubspaceAnalysis::condense_labels(label_vec);

        //allocate all matrices
        model_data_arr_=cv::Mat(img_vec.size(),img_vec[0].total(),CV_64FC1);
        model_label_vec_.resize(img_vec.size());
        average_arr_=cv::Mat(1,img_vec[0].total(),CV_64FC1);
        projection_mat_=cv::Mat(target_dim_,img_vec[0].total(),CV_64FC1);
        eigenvalues_=cv::Mat(1,target_dim_-1,CV_64FC1);
        model_features_=cv::Mat(model_data_arr_.rows,target_dim_,CV_64FC1);

        model_data_mat(img_vec,model_data_arr_);
        //initiate PCA
        pca_=SubspaceAnalysis::PCA(model_data_arr_,target_dim_);

        //Assign model to member variables
        projection_mat_=pca_.eigenvecs;
        eigenvalues_=pca_.eigenvals;
        average_arr_=pca_.mean;


        extractFeatures(model_data_arr_,projection_mat_,model_features_);
        if(use_unknown_thresh_)
        {
        calc_threshold(model_features_,unknown_thresh_);
        }

  // set FaceRecognizer to trained
  this->trained_=true;

}










//---------------------------------------------------------------------------------------------------------------------<
//---------------------------------------------------------------------------------------------------------------------<
//---------------------------------------------------------------------------------------------------------------------<
//---------------------------------------------------------------------------------------------------------------------<
//---------------------------------------------------------------------------------------------------------------------<
//---------------------------------------------------------------------------------------------------------------------<
//---------------------------------------------------------------------------------------------------------------------<
void SubspaceAnalysis::mat2arr(cv::Mat& src_mat,cv::Mat& dst_mat)
{

    dst_mat=src_mat.clone().reshape(1,1);

  return;
}
void SubspaceAnalysis::error_prompt(std::string fct,std::string desc)
{
  std::cerr<<"ERROR\n";
  std::cerr<<"Function:......... "<<fct<<std::endl;
  std::cerr<<"Description:...... "<<desc<<std::endl;
}
void SubspaceAnalysis::dump_matrix(cv::Mat& mat,std::string filename)
{
  std::string path = "/share/goa-tz/people_detection/eval/vis/";
  path.append(filename.c_str());
  std::ofstream os(path.c_str() );
  for(int r=0;r<mat.rows;r++)
  {
    for(int c=0;c<mat.cols;c++)
    {
    os<<mat.at<double>(r,c)<<" ";
    }
    os<<"\n";
  }
  os.close();
}


void SubspaceAnalysis::condense_labels(std::vector<int>& labels)
{
  int min_val=std::numeric_limits<int>::max();
  for(int i =0;i<labels.size();i++)
  {
    if(labels[i] < min_val) min_val=labels[i];
  }
  if(min_val>0)
  {
    for(int i =0;i<labels.size();i++)
    {
      labels[i] -=min_val;
    }
  }
}
void  SubspaceAnalysis::mat_info(cv::Mat& mat)
{


  for(int r=0;r<mat.rows;r++)
  {
    for(int c=0;c<mat.cols;c++)
    {
      std::cout<<mat.at<float>(r,c)<<" ";
    }

    std::cout<<"\n";
  }
  std::cout<<"Matrix info:\n";
  std::cout<<"rows= "<<mat.rows<<"  cols= "<<mat.cols<<std::endl;
  std::cout<<"Type = "<<mat.type()<<std::endl;
  std::cout<<"Channels = "<<mat.channels()<<std::endl;
}
void SubspaceAnalysis::unique_elements(cv::Mat & mat,int& unique_elements,std::vector<int>& distinct_vec)
{
  bool unique=true;
  for(int i=0;i<mat.total();++i)
  {

    if(i!=0)
    {
    unique=true;
    for(int j=0;j<distinct_vec.size();j++)
    {
      if(mat.at<float>(i)==distinct_vec[j]) unique =false;
    }
    }
    if(unique==true)distinct_vec.push_back(mat.at<float>(i));
  }
  unique_elements=distinct_vec.size();
}
void SubspaceAnalysis::unique_elements(std::vector<int> & vec,int& unique_elements,std::vector<int>& distinct_vec)
{
  bool unique=true;
  for(int i=0;i<vec.size();++i)
  {

    if(i!=0)
    {
    unique=true;
    for(int j=0;j<distinct_vec.size();j++)
    {
      if(vec[i]==distinct_vec[j]) unique =false;
    }
    }
    if(unique==true)distinct_vec.push_back(vec[i]);
  }
  unique_elements=distinct_vec.size();
}
//---------------------------------------------------------------------------------
//  XFace XFaces
//---------------------------------------------------------------------------------
//
//
//
void SubspaceAnalysis::XFaces::calcDFFS(cv::Mat& orig_mat,cv::Mat& recon_mat,cv::Mat& avg,std::vector<double>& DFFS)
{

  cv::Mat temp=cv::Mat(orig_mat.rows,orig_mat.cols,orig_mat.type());
  orig_mat.copyTo(temp);
  DFFS.resize(recon_mat.rows);
  for(int i=0;i<orig_mat.rows;i++)
  {
    cv::Mat recon_row=recon_mat.row(i);
    cv::Mat temp_row=temp.row(i);
    cv::subtract(temp_row,avg,temp_row);
    DFFS[i]= cv::norm(recon_row,temp_row,cv::NORM_L2);
  }
  return;
}



void SubspaceAnalysis::XFaces::mat2arr(cv::Mat& src_mat,cv::Mat& dst_mat)
{

    dst_mat=src_mat.clone().reshape(1,1);

  return;
}
void SubspaceAnalysis::XFaces::project2D(std::vector<cv::Mat>& src_mats,cv::Mat& proj_mat,cv::Mat& avg_mat,std::vector<cv::Mat>& coeff_mat_vec)
{
  //calculate coefficients
  for(int i=0;i<src_mats.size();i++)
  {
  cv::Mat tmp;
  cv::gemm(src_mats[i],proj_mat,1.0,cv::Mat(),0.0,tmp,cv::GEMM_2_T);
  coeff_mat_vec.push_back(tmp);
  }
}

void SubspaceAnalysis::XFaces::project(cv::Mat& src_mat,cv::Mat& proj_mat,cv::Mat& avg_mat,cv::Mat& coeff_mat)
{
  //calculate coefficients
  cv::gemm(src_mat,proj_mat,1.0,cv::Mat(),0.0,coeff_mat,cv::GEMM_2_T);
}

void SubspaceAnalysis::XFaces::reconstruct(cv::Mat& coeffs,cv::Mat& proj_mat,cv::Mat& avg,cv::Mat& rec_im)
{
  cv::gemm(coeffs,proj_mat,1.0,cv::Mat(),0.0,rec_im);
  for(int i=0;i<rec_im.rows;i++)
  {
    cv::Mat curr_row=rec_im.row(i);
    cv::add(curr_row,avg.reshape(1,1),curr_row);
  }
}

void SubspaceAnalysis::XFaces::calcDataMat(std::vector<cv::Mat>& input_data,cv::Mat& data_mat)
{

  // convert input to data matrix,with images as rows

    for(int i = 0;i<input_data.size();i++)
    {
    cv::Mat src_mat;
      src_mat=input_data[i];
      src_mat=src_mat.reshape(1,1);
      cv::Mat dst_row=data_mat.row(i);
      src_mat.copyTo(dst_row);
      }

  return;
}

void SubspaceAnalysis::XFaces::retrieve(std::vector<cv::Mat>& out_eigenvectors,cv::Mat& out_eigenvalues,cv::Mat& out_avg,cv::Mat& out_proj_model_data)
{

  avg_arr_.copyTo(out_avg);
  proj_model_data_arr_.copyTo(out_proj_model_data);

  for(int r=0;r<eigenvector_arr_.rows;r++)
  {
    cv::Mat curr_row=eigenvector_arr_.row(r);
    //TODO: works only for square images
    curr_row.clone().reshape(1,sqrt(curr_row.cols));
    curr_row.convertTo(curr_row,CV_32FC1);
    curr_row.copyTo(out_eigenvectors[r]);

  }

  eigenvalue_arr_.copyTo(out_eigenvalues);

}

void SubspaceAnalysis::XFaces::getModel(cv::Mat& out_eigenvectors,cv::Mat& out_eigenvalues,cv::Mat& out_avg,cv::Mat& out_proj_model_data)
{
  avg_arr_            .copyTo( out_avg            ) ;
  proj_model_data_arr_.copyTo( out_proj_model_data) ;
  eigenvector_arr_    .copyTo( out_eigenvectors   ) ;
  eigenvalue_arr_     .copyTo( out_eigenvalues    ) ;
}

void SubspaceAnalysis::XFaces::retrieve(std::vector<cv::Mat>& out_eigenvectors,cv::Mat& out_eigenvalues,cv::Mat& out_avg,cv::Mat& out_proj_model_data,cv::Size output_dim)
{

  avg_arr_.copyTo(out_avg);
  proj_model_data_arr_.copyTo(out_proj_model_data);

  for(int r=0;r<eigenvector_arr_.rows;r++)
  {
    cv::Mat curr_row=eigenvector_arr_.row(r);
    //TODO: works only for square images
    curr_row.clone().reshape(1,output_dim.height);
    curr_row.convertTo(curr_row,CV_32FC1);
    curr_row.copyTo(out_eigenvectors[r]);

  }

  eigenvalue_arr_.copyTo(out_eigenvalues);

}



void SubspaceAnalysis::XFaces::projectToSubspace2D(cv::Mat& src_arr,std::vector<cv::Mat>& coeff_mat_vec,double& DFFS)
{

  if(this->trained ==false)
  {
    std::cout<<"XFaces --> Model not trained - aborting projectToSubspace\n";
    return;
  }


  // TODO: LOOK UP IS THIS RIGHT????
  // reduce im mat by mean
  //for(int i=0;i<src_arr.rows;i++)
  //{
  //  cv::Mat im=src_arr.row(i);
  //  cv::subtract(im,avg_arr_,im);
  //}

  //cv::normalize(src_arr,src_arr,1.0,0.0,cv::NORM_L1);
  //cv::subtract(src_arr,avg_arr_,src_arr);
  std::vector<cv::Mat> matvec;
  matvec.push_back(src_arr);
  project2D(matvec,eigenvector_arr_,avg_arr_,coeff_mat_vec);

  //cv::Mat rec_mat=cv::Mat(src_arr.rows,eigenvector_arr_.rows,CV_64FC1);
  //reconstruct(coeff_arr,eigenvector_arr_,avg_arr_,rec_mat);


  //std::vector<double> DFFS_vec;
  //DFFS_vec.push_back(DFFS);
  //calcDFFS(src_arr,rec_mat,avg_arr_,DFFS_vec);
  //DFFS=DFFS_vec[0];

  //cv::Mat dummy;
  //rec_mat.copyTo(dummy);
  //dummy.convertTo(dummy,CV_8UC1);
  //dummy =dummy.reshape(1,dummy.rows*160);
  ////cv::imshow("reconstuction",dummy);
  //cv::imwrite("/home/goa-tz/Desktop/reconstructed.jpg",dummy);
}
void SubspaceAnalysis::XFaces::projectToSubspace(cv::Mat& probe_mat,cv::Mat& coeff_arr,double& DFFS)
{

  if(this->trained ==false)
  {
    std::cout<<"XFaces --> Model not trained - aborting projectToSubspace\n";
    return;
  }

  cv::Mat src_arr;
  mat2arr(probe_mat,src_arr);

  // TODO: LOOK UP IS THIS RIGHT????
  // reduce im mat by mean
  //for(int i=0;i<src_arr.rows;i++)
  //{
  //  cv::Mat im=src_arr.row(i);
  //  cv::subtract(im,avg_arr_,im);
  //}

  //cv::normalize(src_arr,src_arr,1.0,0.0,cv::NORM_L1);
  //cv::subtract(src_arr,avg_arr_,src_arr);
  project(src_arr,eigenvector_arr_,avg_arr_,coeff_arr);

  cv::Mat rec_mat=cv::Mat(src_arr.rows,eigenvector_arr_.rows,CV_64FC1);
  reconstruct(coeff_arr,eigenvector_arr_,avg_arr_,rec_mat);


  std::vector<double> DFFS_vec;
  DFFS_vec.push_back(DFFS);
  calcDFFS(src_arr,rec_mat,avg_arr_,DFFS_vec);
  DFFS=DFFS_vec[0];

  //cv::Mat dummy;
  //rec_mat.copyTo(dummy);
  //dummy.convertTo(dummy,CV_8UC1);
  //dummy =dummy.reshape(1,dummy.rows*160);
  ////cv::imshow("reconstuction",dummy);
  //cv::imwrite("/home/goa-tz/Desktop/reconstructed.jpg",dummy);
}

void SubspaceAnalysis::XFaces::calc_threshold(cv::Mat& data,std::vector<cv::Mat>& thresh)
{
  //TODO: calculate sigma
  double sigma =1.7f;
  thresh.resize(num_classes_);
   class_centers_=cv::Mat::zeros(num_classes_,data.cols,CV_64FC1);
  SubspaceAnalysis::LDA lda;
  lda.calcClassMean(data,model_label_arr_,class_centers_,num_classes_);

  cv::Mat max_arr,curr_mean,curr_sample,curr_max;
  max_arr=cv::Mat::ones(num_classes_,data.cols,CV_64FC1);
  max_arr*=std::numeric_limits<double>::min();

  for(int i=0;i<data.rows;i++)
    {
      int index=(int)model_label_arr_.at<float>(i);
      curr_sample=data.row(i);
      curr_mean=class_centers_.row(index);
      curr_max=max_arr.row(index);
      cv::max(curr_max,curr_sample-curr_mean,curr_max);
     }

  for(int j=0;j<num_classes_;j++)
  {
    cv::Mat curr_row=max_arr.row(j);
    thresh[j]=sigma*curr_row;
  }
}
void SubspaceAnalysis::XFaces::calc_threshold2d(std::vector<cv::Mat>& data,double& thresh)
{
  //TODO implemented brute force method -> optimization
  // implement for case when Di>Pi
  //  don't use until that....
  std::vector<double> P(num_classes_,std::numeric_limits<double>::max());
  std::vector<double> D(num_classes_,std::numeric_limits<double>::min());
  std::vector<double> Phi(num_classes_,std::numeric_limits<double>::max());

      for(int m=0;m<proj_model_data_vec_.size();m++)
      {
    }

  for(int i=0;i<data.size();i++)
  {
    for(int n=0;n<data.size();n++)
    {
      if(n==i) continue;

        // subtract matrices
        cv::Mat work_mat;
        //L1 norm
        //cv::absdiff (probe_mat,proj_model_data_vec_[m],work_mat);
        cv::subtract (data[i],data[n],work_mat);
        cv::pow(work_mat,2,work_mat);
        //cv::sqrt(work_mat,work_mat);
        cv::Mat tmp_vec=cv::Mat::zeros(1,data[i].cols,CV_64FC1);
        cv::reduce(work_mat,tmp_vec,0,CV_REDUCE_SUM);

        //maha. dist
        //tmp_vec=tmp_vec/eigenvalue_arr_;
        cv::Scalar temp=cv::sum(tmp_vec);
      double dist = temp.val[0];
      if(model_label_arr_.at<float>(n)==model_label_arr_.at<float>(i))
      {
        D[model_label_arr_.at<float>(i)]=std::max(dist,D[model_label_arr_.at<float>(i)]);
      }
      else
      {
        P[model_label_arr_.at<float>(i)]=std::min(dist,P[model_label_arr_.at<float>(i)]);
      }
    }
  }

  // if only one class - P =D
  if(num_classes_==1)
  {
    P[0]=D[0];
  }

  for(int c=0;c<num_classes_;c++)
  {
    thresh=std::min(thresh,(P[c]+D[c]) *0.2);
    //std::cout<<"c"<<c<<": "<<D[c]<<" "<<P[c]<<std::endl;
  }
  std::cout<<"THRESH for db: "<<thresh<<std::endl;


}
void SubspaceAnalysis::XFaces::calc_threshold(cv::Mat& data,double& thresh)
{
  //TODO implemented brute force method -> optimization
  // implement for case when Di>Pi
  //  don't use until that....
  std::vector<double> P(num_classes_,std::numeric_limits<double>::max());
  std::vector<double> D(num_classes_,std::numeric_limits<double>::min());
  std::vector<double> Phi(num_classes_,std::numeric_limits<double>::max());

  for(int i=0;i<data.rows;i++)
  {
    cv::Mat i_row=data.row(i);
    for(int n=0;n<data.rows;n++)
    {
      if(n==i) continue;
      cv::Mat n_row=data.row(n);
      double dist = cv::norm(i_row,n_row,cv::NORM_L2);
      if(model_label_arr_.at<float>(n)==model_label_arr_.at<float>(i))
      {
        D[model_label_arr_.at<float>(i)]=std::max(dist,D[model_label_arr_.at<float>(i)]);
      }
      else
      {
        P[model_label_arr_.at<float>(i)]=std::min(dist,P[model_label_arr_.at<float>(i)]);
      }
    }
  }

  // if only one class - P =D
  if(num_classes_==1)
  {
    P[0]=D[0];
  }

  for(int c=0;c<num_classes_;c++)
  {
    thresh=std::min(thresh,(P[c]+D[c]) *0.5);
  }
  std::cout<<"THRESH for db: "<<thresh<<std::endl;


}


void SubspaceAnalysis::XFaces::prediction(cv::Mat& query_image,int& max_prob_index)
{
  //check whether model is 2d or 1 D and the choose ProjectToSubspace
  //accordingly

}
void SubspaceAnalysis::XFaces::classify(cv::Mat& coeff_arr,Classifier method,int& max_prob_index,cv::Mat& probabilities)
{
  classify(coeff_arr,method,max_prob_index);
  probabilities=class_cost_;
}

void SubspaceAnalysis::XFaces::classify(cv::Mat& coeff_arr,Classifier method,int& class_index)
{
  if(this->trained ==false)
  {
    std::cout<<"XFaces --> Model not trained - aborting classify\n";
    return;
  }

  if(num_classes_<2)
  {
    method = SubspaceAnalysis::CLASS_DIFS;
  }


  double minDIFS;
  switch(method)
  {
    case SubspaceAnalysis::CLASS_DIFS:
    {
      cv::Mat minDIFScoeffs;
      int minDIFSindex;
      calcDIFS(coeff_arr,minDIFSindex,minDIFS,minDIFScoeffs);
      class_index=(int)model_label_arr_.at<float>(minDIFSindex);

      break;
    };

    case SubspaceAnalysis::CLASS_KNN:
    {
      // train SVM when not already trained
      if(!knn_trained_)
      {

        cv::Mat data_float;
        proj_model_data_arr_.convertTo(data_float,CV_32FC1);


        knn_.train(data_float,model_label_arr_);
        knn_trained_=true;
      }

      // predict using knn
      cv::Mat sample;
      coeff_arr.convertTo(sample,CV_32FC1);
      //class_index=(int)svm_.predict(sample);
      cv::Mat result;

      class_index=(int)knn_.find_nearest(sample,3);


      break;
    }
    case SubspaceAnalysis::CLASS_SVM:
    {
      // train SVM when not already trained
      if(!svm_trained_)
      {
        //train svm with model data
        CvSVMParams params;
        params.svm_type = CvSVM::C_SVC;
        //params.kernel_type = CvSVM::LINEAR;
        params.kernel_type = CvSVM::LINEAR;
        params.term_crit = cvTermCriteria(CV_TERMCRIT_ITER,50,1e-5);

        cv::Mat data_float;
        proj_model_data_arr_.convertTo(data_float,CV_32FC1);


        svm_.train(data_float,model_label_arr_,cv::Mat(),cv::Mat(),params);
        svm_trained_=true;
      }

      // predict using svm
      cv::Mat sample;
      coeff_arr=coeff_arr.reshape(1,1);
      coeff_arr.convertTo(sample,CV_32FC1);
     class_index=(int)svm_.predict(sample);

      break;
    };
    case SubspaceAnalysis::CLASS_RF:
    {
      // train SVM when not already trained
      if(!rf_trained_)
      {
        //train svm with model data
        CvRTParams params;

        cv::Mat data_float;
        proj_model_data_arr_.convertTo(data_float,CV_32FC1);


        rf_.train(data_float,CV_ROW_SAMPLE,model_label_arr_,cv::Mat(),cv::Mat(),cv::Mat(),cv::Mat(),params);
        rf_trained_=true;
      }

      // predict using svm
      cv::Mat sample;
      coeff_arr.convertTo(sample,CV_32FC1);
      //class_index=(int)svm_.predict(sample);
     class_index=(int)rf_.predict(sample);

      break;
    };

    default: {
      std::cout<<"[CLASSIFICATION] method not implemented"<<std::endl;
      break;
    };
  }

  if(use_unknown_thresh_)
  {
    //if(! verifyClassification(coeff_arr,class_index))class_index=-1;
    if(! is_known(minDIFS,thresh_))class_index=-1;

  }
  return;
}


void SubspaceAnalysis::XFaces::calcDIFS(cv::Mat& probe_mat,int& minDIFSindex,double& minDIFS,cv::Mat& minDIFScoeffs)
{
    double temp;
    minDIFS=std::numeric_limits<int>::max();
    class_cost_*=  std::numeric_limits<float>::max();
    // if feature matrices not feature vectors ( 2D methods)
    if(probe_mat.rows>1)
    {
      for(int m=0;m<proj_model_data_vec_.size();m++)
      {
        // subtract matrices
        cv::Mat work_mat;
        //L1 norm
        //cv::absdiff (probe_mat,proj_model_data_vec_[m],work_mat);
        cv::subtract (probe_mat,proj_model_data_vec_[m],work_mat);
        cv::pow(work_mat,2,work_mat);
        //cv::sqrt(work_mat,work_mat);
        cv::Mat tmp_vec=cv::Mat::zeros(1,probe_mat.cols,CV_64FC1);
        cv::reduce(work_mat,tmp_vec,0,CV_REDUCE_SUM);

        //maha. dist
        //tmp_vec=tmp_vec/eigenvalue_arr_;
        cv::Scalar temp=cv::sum(tmp_vec);

        //temp=0;
        //for(int c=0;c<proj_model_data_vec_[m].cols;c++)
        //{
        //  cv::Mat model_col=proj_model_data_vec_[m].col(c);
        //  cv::Mat test_col=probe_mat.col(c);


        //  double col_dif=cv::norm(test_col,model_col,cv::NORM_L2);
        //  temp+=col_dif;
        //}

          if(temp[0] < minDIFS )
          {
            minDIFSindex=m;
            //minDIFScoeffs=diff_row;
            minDIFS=temp[0];
          }
        //calculate cost for classification to every class in database
        class_cost_.at<float>(model_label_arr_.at<float>(m))=std::min(class_cost_.at<float>(model_label_arr_.at<float>(m)),(float)temp[0]);
        }
    }
    // for 1D methods
    else
    {
      for(int r=0;r<proj_model_data_arr_.rows;r++)
      {
        cv::Mat model_mat=proj_model_data_arr_.row(r);
        cv::Mat diff_row;

        temp=cv::norm(probe_mat,model_mat,cv::NORM_L2);
        if(temp < minDIFS )
        {
          minDIFSindex=r;
          minDIFScoeffs=diff_row;
          minDIFS=temp;
        }
        //calculate cost for classification to every class in database
        class_cost_.at<float>(model_label_arr_.at<float>(r))=std::min(class_cost_.at<float>(model_label_arr_.at<float>(r)),(float)temp);
      }
    }


    //process class_cost_
    double min_cost,max_cost;
    //class_cost_-=(float)(min_cost);
    class_cost_=1/(class_cost_.mul(class_cost_));
    cv::minMaxLoc(class_cost_,&min_cost,&max_cost,0,0);
    //double class_cost_norm=cv::norm(class_cost_,cv::NORM_L1);
    class_cost_/=max_cost;
    //cv::normalize(class_cost_,class_cost_,1.0);

    return;
}



void SubspaceAnalysis::XFaces::releaseModel()
{
      num_classes_ =-1;
      ss_dim_ =-1;
      svm_trained_=false;
      knn_trained_=false;
      eigenvector_arr_.release();
      eigenvalue_arr_.release();
      avg_arr_.release();
      model_data_arr_.release();
      proj_model_data_arr_.release();
      model_label_arr_.release();;


      CvSVM svm_;
      CvKNearest knn_;

}


bool SubspaceAnalysis::XFaces::is_known(double& d,double& thresh)
{

  if( d>thresh) return false;
  else return true;

}

bool SubspaceAnalysis::XFaces::verifyClassification(cv::Mat& sample,int& index)
{


  double minDIFS=std::numeric_limits<double>::max();
  for(int n=0;n<proj_model_data_arr_.rows;n++)
  {
    if(model_label_arr_.at<float>(n)==(float)index)
    {
      cv::Mat curr_row=proj_model_data_arr_.row(n);
      double dist=cv::norm(curr_row,sample,cv::NORM_L2);
      minDIFS=std::min(dist,minDIFS);
    }
  }

  // Decision
  if(minDIFS>thresh_)
  {
    //std::cout<<"NEW threshold: unknown "<<minDIFS<<std::endl;
    index=-1;
    return false;
  }
  return true;
}



//---------------------------------------------------------------------------------
// FaceRecognizer
//---------------------------------------------------------------------------------

SubspaceAnalysis::FaceRecognizer::FaceRecognizer()
{
fallback_=false;
svm_trained_=false;
knn_trained_=false;
use_unknown_thresh_=true;

//initialize Threshold with maximum value
thresh_=std::numeric_limits<double>::max();

num_classes_=-1;

}

//-----------------------------------------------------------------------------------------------
///////////////////////////////////////////////////////////////////////////////////////////////////
//old interface - just for compability reasons  use new trainModel and
//loadModel
bool SubspaceAnalysis::FaceRecognizer::init(std::vector<cv::Mat>& img_vec,std::vector<int>& label_vec,int& red_dim)
{
  trainModel( img_vec, label_vec, red_dim);
}
bool SubspaceAnalysis::FaceRecognizer::init(std::vector<cv::Mat>& img_vec,std::vector<int>& label_vec,int& red_dim,Method method)
{
  trainModel(img_vec, label_vec, red_dim, method);
}
bool SubspaceAnalysis::FaceRecognizer::init(std::vector<cv::Mat>& img_vec,std::vector<int>& label_vec,int& red_dim,Method method,bool fallback,bool use_unknown_thresh)
{
  trainModel( img_vec, label_vec, red_dim,method,fallback, use_unknown_thresh);
}
//-----------------------------------------------------------------------------------------------
///////////////////////////////////////////////////////////////////////////////////////////////////


bool SubspaceAnalysis::XFaces::saveModel(std::string path)
{

  std::cout<<"saving model"<<std::endl;

		if (boost::filesystem::is_regular_file(path.c_str()))
		{
			if (boost::filesystem::remove(path.c_str()) == false)
			{

      error_prompt("saveModel()","old rdata.xml can not be removed");
				return false;
			}
		}
		cv::FileStorage fileStorage(path.c_str(), cv::FileStorage::WRITE);
		if(!fileStorage.isOpened())
		{
      error_prompt("saveModel()","Output path is invalid");
			return false;
		}

    //create string describing model ( 1D or 2D)
    std::string model_type_string;
    if(method_==METH_FISHER ||method_==METH_EIGEN)
    {
      model_type_string="1D";
    }
    if(method_==METH_LDA2D ||method_==METH_PCA2D)
    {
      model_type_string="2D";
    }

    // Modeltype ( 1D or 2D)
    fileStorage << "modeltype" << model_type_string;
    // Eigenvectors
    fileStorage << "eigenvectors" << eigenvector_arr_;
		// Eigenvalue matrix
		fileStorage << "eigenvalues" << eigenvalue_arr_;

		// Average image
		fileStorage << "average_image" << avg_arr_;

		// Projection coefficients of the training faces
		fileStorage << "projected_model_data" << proj_model_data_arr_;

    fileStorage << "model_data_labels"  << model_label_arr_;

		fileStorage.release();

	return true;
}

bool SubspaceAnalysis::XFaces::loadModelFromFile(std::string path,bool use_unknown_thresh)
{
  //if(this->trained)this->releaseModel();

	// secure this function with a mutex

		cv::FileStorage fileStorage(path.c_str(), cv::FileStorage::READ);
		if(!fileStorage.isOpened())
		{
      error_prompt("loadModelFromFile()","Invalid input file");
      return false;
		}
		else
		{
      fileStorage["eigenvectors"] >> eigenvector_arr_;
      fileStorage["eigenvalues"] >> eigenvalue_arr_;
      fileStorage["average_image"] >> avg_arr_;
      fileStorage["projected_model_data"] >> proj_model_data_arr_;
      fileStorage["model_data_labels"]  >> model_label_arr_;

		}
		fileStorage.release();

    //TODO keep only a selection instead of whole dataset depending on labels

    use_unknown_thresh_= use_unknown_thresh;

    SubspaceAnalysis::unique_elements(model_label_arr_,num_classes_,unique_labels_);

    ss_dim_=proj_model_data_arr_.cols;

    if(use_unknown_thresh_)
    {
    std::cout<<"calculating threshold...";
    calc_threshold(proj_model_data_arr_,thresh_);
    std::cout<<"done"<<std::endl;
    }
    this->trained=true;
    std::cout<<"FaceRecognizer --> model loaded successfully\n";
    return true;

}

bool SubspaceAnalysis::XFaces::loadModel(cv::Mat& eigenvec_arr,cv::Mat& eigenval_arr,cv::Mat& avg_arr,cv::Mat& proj_model,std::vector<int>& label_vec,bool use_unknown_thresh)
{
  //check if number of labels equals number of images in training set
  if(label_vec.size()!=proj_model.rows)
  {
    error_prompt("loadModel()","#labels != #rows in projected model data");
    return false;
  }

  //if(this->trained)this->releaseModel();
  eigenvector_arr_=eigenvec_arr;
  eigenvalue_arr_=eigenval_arr;
  proj_model_data_arr_=proj_model;
  avg_arr_=avg_arr;

  use_unknown_thresh_= use_unknown_thresh;

  SubspaceAnalysis::unique_elements(label_vec,num_classes_,unique_labels_);
  SubspaceAnalysis::condense_labels(label_vec);
  model_label_arr_=cv::Mat(1,label_vec.size(),CV_32FC1);
  for(int i=0;i<label_vec.size();i++)
  {
    model_label_arr_.at<float>(i)=static_cast<float>(label_vec[i]);
  }

  ss_dim_=proj_model_data_arr_.cols;

  if(use_unknown_thresh_)
  {
  std::cout<<"calculating threshold...";
  calc_threshold(proj_model_data_arr_,thresh_);
  std::cout<<"done"<<std::endl;
  }
  this->trained=true;
  std::cout<<"FaceRecognizer --> model loaded successfully\n";
  return true;
}



bool SubspaceAnalysis::FaceRecognizer::trainModel(std::vector<cv::Mat>& img_vec,std::vector<int>& label_vec,int& red_dim)
{
  trainModel( img_vec,label_vec, red_dim, SubspaceAnalysis::METH_FISHER, true,true);
}
bool SubspaceAnalysis::FaceRecognizer::trainModel(std::vector<cv::Mat>& img_vec,std::vector<int>& label_vec,int& red_dim,Method method)
{
  trainModel( img_vec,label_vec, red_dim, method, true,true);
}
bool SubspaceAnalysis::FaceRecognizer::trainModel(std::vector<cv::Mat>& img_vec,std::vector<int>& label_vec,int& red_dim,Method method,bool fallback,bool use_unknown_thresh)
{
  fallback_=fallback;
  use_unknown_thresh_= use_unknown_thresh;
  //initialize Threshold with maximum value
  //process_labels<int>(label_vec,model_label_arr_);


  SubspaceAnalysis::unique_elements(label_vec,num_classes_,unique_labels_);
  SubspaceAnalysis::condense_labels(label_vec);

  //input data checks
 
  //check if input has the same size
  ss_dim_=red_dim;
  // set subspace dimension here for 2D* and Eigenfaces methods
  ss_dim_=num_classes_-1;
  if(img_vec.size()<ss_dim_+1)
  {
    error_prompt("trainModel()","Invalid subspace dimension");
    return false;
  }
  //check if number of labels equals number of images in training set
  if(label_vec.size()!=img_vec.size())
  {
    return false;
  }
  //check if multiple classes and fallback  to eigenfaces if fallback_==true
  if(num_classes_<2 &&  method== SubspaceAnalysis::METH_FISHER)
  {
    if(fallback_)
    {
      method=SubspaceAnalysis::METH_EIGEN;
      std::cout<<"[SubspaceAnalysis] Automatic Fallback from Fisherfaces to Eigenfaces ( only 1 class)"<<std::endl;
    }
    else return false;
  }

  if(num_classes_<2 &&  method== SubspaceAnalysis::METH_LDA2D)
  {
    if(fallback_)
    {
      method=SubspaceAnalysis::METH_PCA2D;
      std::cout<<"[SubspaceAnalysis] Automatic Fallback from 2D LDA  to 2D PCA ( only 1 class)"<<std::endl;
    }
    else return false;
  }


  //initialize all matrices
  model_data_arr_=cv::Mat(img_vec.size(),img_vec[0].total(),CV_64FC1);
  model_label_arr_=cv::Mat(1,label_vec.size(),CV_32FC1);
  avg_arr_=cv::Mat(1,img_vec[0].total(),CV_64FC1);
  eigenvector_arr_=cv::Mat(ss_dim_,img_vec[0].total(),CV_64FC1);
  eigenvalue_arr_=cv::Mat(1,ss_dim_-1,CV_64FC1);
  class_cost_=cv::Mat::ones(1,num_classes_,CV_32FC1);
  class_cost_*=  std::numeric_limits<float>::max();

  //Allocation for 2D Methods
  if (method==SubspaceAnalysis::METH_LDA2D || method==SubspaceAnalysis::METH_PCA2D)
  {
    proj_model_data_arr_=cv::Mat(img_vec.size(),ss_dim_*img_vec[0].rows,CV_64FC1);
  }

  //Allocation and preprocessing for 1D Methods
  else
  {

    proj_model_data_arr_=cv::Mat(img_vec.size(),ss_dim_,CV_64FC1);
    calcDataMat(img_vec,model_data_arr_);
  }


  //Labels cast to float
  for(int i=0;i<label_vec.size();i++)
  {
    model_label_arr_.at<float>(i)=static_cast<float>(label_vec[i]);
  }


  
  int pca_dim;
  cv::Mat proj_model_data_arr_PCA,P_pca,P_lda;


  switch (method)
  {
    // Processing for Fisherfaces
    case SubspaceAnalysis::METH_FISHER:
      {
        std::cout<<"[SubspaceAnalysis]Fisherfaces - computing recognition model"<<std::endl;
        method_=SubspaceAnalysis::METH_FISHER;
        //subspace dimension is num classes -1
        ss_dim_=num_classes_ -1;

        // pca dimension  is N- num_classes
        pca_dim=model_data_arr_.rows-num_classes_;

        if(pca_dim<1)
        {
          pca_dim=num_classes_;
        }

        // Reduce dimension to  N - c via PCA
        pca_=SubspaceAnalysis::PCA(model_data_arr_,pca_dim);
        proj_model_data_arr_PCA=cv::Mat(model_data_arr_.rows,pca_dim,CV_64FC1);
        project(model_data_arr_,pca_.eigenvecs,pca_.mean,proj_model_data_arr_PCA);

        // get projection matrix pca
        P_pca=cv::Mat(pca_dim,img_vec[0].total(),CV_64FC1);
        P_pca=pca_.eigenvecs;

        //perform lda
        lda_=SubspaceAnalysis::LDA(proj_model_data_arr_PCA,label_vec,num_classes_,ss_dim_);

        // get projection matrix lda
        P_lda=cv::Mat(ss_dim_,pca_dim,CV_64FC1);
        P_lda=lda_.eigenvecs;


        // combine projection matrices
        cv::gemm(P_pca.t(),P_lda.t(),1.0,cv::Mat(),0.0,eigenvector_arr_);

        //Assign model to member variables
        eigenvector_arr_=eigenvector_arr_.t();
        eigenvalue_arr_=lda_.eigenvals;
        avg_arr_=pca_.mean;
        break;

      }

    case SubspaceAnalysis::METH_EIGEN: 
      {
        std::cout<<"[SubspaceAnalysis]Eigenfaces - computing recognition model"<<std::endl;
        method_=SubspaceAnalysis::METH_EIGEN;
        //initiate PCA
        pca_=SubspaceAnalysis::PCA(model_data_arr_,ss_dim_);

        //Assign model to member variables
        eigenvector_arr_=pca_.eigenvecs;
        eigenvalue_arr_=pca_.eigenvals;
        avg_arr_=pca_.mean;
        break;
      }

    case SubspaceAnalysis::METH_LDA2D:
      {
        std::cout<<"[SubspaceAnalysis] 2D-LDA - computing recognition model"<<std::endl;
        method_=SubspaceAnalysis::METH_LDA2D;
        //initiate 2D LDA
        lda2d_=SubspaceAnalysis::LDA2D(img_vec,label_vec,num_classes_,ss_dim_);

        //Assign model to member variables
        eigenvector_arr_=lda2d_.eigenvecs;
        eigenvalue_arr_=lda2d_.eigenvals;
        avg_arr_=lda2d_.mean;
        break;
      }
    case SubspaceAnalysis::METH_PCA2D:
      {
        std::cout<<"[SubspaceAnalysis] 2D-PCA - computing recognition model"<<std::endl;
        method_=SubspaceAnalysis::METH_PCA2D;
        // initiate 2D PCA
        pca2d_=SubspaceAnalysis::PCA2D(img_vec,label_vec,num_classes_,ss_dim_);

        //Assign model to member variables
        eigenvector_arr_=pca2d_.eigenvecs;
        eigenvalue_arr_= pca2d_.eigenvals;
        avg_arr_=pca2d_.mean;
        break;
      }

  }

  // Project training data to subspace  - 2D methods
  if (method==SubspaceAnalysis::METH_LDA2D || method == SubspaceAnalysis::METH_PCA2D)
  {
    project2D(img_vec,eigenvector_arr_,avg_arr_,proj_model_data_vec_);

    for(int i=0;i<proj_model_data_vec_.size();i++)
    {
      cv::Mat curr_row=proj_model_data_arr_.row(i);

      curr_row=proj_model_data_vec_[i].reshape(1,1);
    }
  if(use_unknown_thresh_)
  {
    // unknown threshold
    calc_threshold2d(proj_model_data_vec_,thresh_);
  }

  }

  // Project training data to subspace  - 1D methods
  else
  {
    proj_model_data_arr_=cv::Mat(model_data_arr_.rows,ss_dim_,CV_64FC1);
    project(model_data_arr_,eigenvector_arr_,avg_arr_,proj_model_data_arr_);
    if(use_unknown_thresh_)
    {
    calc_threshold(proj_model_data_arr_,thresh_);
    }
  }

  // set FaceRecognizer to trained
  this->trained=true;

}

//---------------------------------------------------------------------------------
// SSA
//---------------------------------------------------------------------------------
void SubspaceAnalysis::SSA::calcDataMatMean(cv::Mat& data,cv::Mat& mean_row)
{
  for(int i=0;i<data.rows;++i)
  {
    cv::Mat data_row=data.row(i);
    //calculate mean
    cv::add(mean_row,data_row,mean_row);
  }
  mean_row.convertTo(mean_row,CV_64F,1.0/static_cast<double>(data.rows));


}


void SubspaceAnalysis::SSA::decomposeAsymmetricMatrix(cv::Mat& data_mat)
{
EigenvalueDecomposition es(data_mat);
eigenvals=es.eigenvalues();
eigenvals=eigenvals.reshape(1,1).t();
eigenvecs=es.eigenvectors();
eigenvecs=eigenvecs.t();

}
void SubspaceAnalysis::SSA::decomposeSVD(cv::Mat& data_mat)
{

  data_mat.convertTo(data_mat,CV_64F,1/sqrt(data_mat.rows));
  cv::SVD svd(data_mat.t());
  eigenvecs=svd.u;
  //svd output is transposed
  //svd.u.copyTo(eigenvecs);
  eigenvecs=eigenvecs.t();
  //svd.w.copyTo(eigenvals);
  eigenvals=svd.w;


}

void SubspaceAnalysis::SSA::decomposeSymmetricMatrix(cv::Mat& data_mat)
{

  cv::eigen(data_mat,eigenvals,eigenvecs);


}



//---------------------------------------------------------------------------------
// LDA
//---------------------mean_arr_row--------------------------------------------
SubspaceAnalysis::PCA2D::PCA2D(std::vector<cv::Mat>& input_data,std::vector<int>& input_labels,int& num_classes,int& ss_dim) 
{
  SubspaceAnalysis::unique_elements(input_labels,num_classes_,unique_labels_);

  //calculate mean of matrices
  cv::Mat  mean= cv::Mat::zeros(input_data[0].rows,input_data[0].cols,CV_64FC1);


  for(int i=0;i<input_data.size();i++)
  {
    //add to class mean
    mean+=input_data[i];
  }
  // is this a valid matrix op TODO
  mean/=input_data.size();


  // TODO TEMPORARY PCA
  cv::Mat P=cv::Mat::zeros(input_data[0].cols,input_data[0].cols,CV_64FC1);

  for(int i=0;i<input_data.size();i++)
  {
    cv::Mat temp;
    cv::subtract(input_data[i],mean,temp);
    cv::mulTransposed(temp,temp,true);
    cv::add(temp,P,P);
  }

  P/=input_data.size();
  decomposeSymmetricMatrix(P);
 eigenvecs=eigenvecs(cv::Rect(0,0,input_data[0].cols,ss_dim));
 eigenvals=eigenvals(cv::Rect(0,0,1,ss_dim)).t();

}
SubspaceAnalysis::LDA2D::LDA2D(std::vector<cv::Mat>& input_data,std::vector<int>& input_labels,int& num_classes,int& ss_dim) 
{
  SubspaceAnalysis::unique_elements(input_labels,num_classes_,unique_labels_);
  mean=cv::Mat::zeros(input_data[0].rows,input_data[0].cols,CV_64FC1);

  //calculate mean of matrices
  cv::Mat  mean_mat = cv::Mat::zeros(input_data[0].rows,input_data[0].cols,CV_64FC1);
  std::vector<cv::Mat> class_means;
  std::vector<int> class_sizes;

  // resize vectors
  class_means.resize(num_classes_);
  class_sizes.resize(num_classes_);

  // initialize with zero
  for(int i =0;i<num_classes_;i++)
  {
    class_means[i]=cv::Mat::zeros(input_data[0].rows,input_data[0].cols,CV_64FC1);
    class_sizes[i]=0;
  }


  for(int i=0;i<input_data.size();i++)
  {
    //add to class mean
    class_means[input_labels[i]]+=input_data[i];
    mean_mat+=input_data[i];
    class_sizes[input_labels[i]]++;
  }
  // is this a valid matrix op TODO
  mean_mat/=input_data.size();


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

 for(int i=0;i<num_classes_;i++)
 {
   class_means[i]/=class_sizes[i];
 }

   cv::Mat S_intra=cv::Mat::zeros(input_data[0].cols,input_data[0].cols,CV_64FC1);
   cv::Mat S_inter=cv::Mat::zeros(input_data[0].cols,input_data[0].cols,CV_64FC1);
 int class_index;

 for(int i=0;i<input_data.size();++i)
 {
   //reduce data matrix
   class_index=input_labels[i];
   cv::Mat tmp;

   cv::subtract(input_data[i],class_means[class_index],tmp);
   cv::Mat temp;
   cv::mulTransposed(input_data[i],temp,true);
   cv::add(temp,S_intra,S_intra);
 }
 for(int c=0;c<num_classes_;c++)
 {
   cv::Mat temp;
   class_index=unique_labels_[c];
   cv::subtract(class_means[c],mean_mat,temp);
   cv::mulTransposed(temp,temp,true);
   temp=temp*class_sizes[c];
   cv::add(S_inter,temp,S_inter);
 }

 for( int i=0;i<input_data.size();i++)
 {
 }
  // //Intra class scatter

 cv::Mat S_intra_inv=S_intra.inv();

 cv::Mat P;
 gemm(S_intra_inv,S_inter,1.0,cv::Mat(),0.0,P);

 decomposeSymmetricMatrix(P);

 eigenvecs=eigenvecs(cv::Rect(0,0,input_data[0].cols,ss_dim));
 eigenvals=eigenvals(cv::Rect(0,0,1,ss_dim)).t();
  //cv::normalize(eigenvecs,eigenvecs);

}
SubspaceAnalysis::LDA::LDA(cv::Mat& input_data,std::vector<int>& input_labels,int& num_classes,int& ss_dim) 
{

  SubspaceAnalysis::unique_elements(input_labels,num_classes_,unique_labels_);
  cv::Mat data_work=input_data.clone();
  mean=cv::Mat::zeros(1,data_work.cols,CV_64FC1);
  calcDataMatMean(data_work,mean);
  //class labels have to be in a vector in ascending order - duplicates are
  //removed internally
  //{0,0,1,2,3,3,4,5}

  class_mean_arr=cv::Mat::zeros(num_classes_,input_data.cols,CV_64FC1);
  calcClassMean(data_work,input_labels,class_mean_arr,num_classes_);

  calcProjMatrix(data_work,input_labels);

  eigenvecs=eigenvecs(cv::Rect(0,0,input_data.cols,ss_dim));
  eigenvals=eigenvals(cv::Rect(0,0,1,ss_dim)).t();
  //cv::normalize(eigenvecs,eigenvecs);

}
void SubspaceAnalysis::LDA::calcClassMean(cv::Mat& data_mat,cv::Mat& label_mat,cv::Mat&  class_mean_arr,int& num_classes)
{
  std::vector<int> label_vec;
  for(int i=0;i<label_mat.cols;i++)
  {
    label_vec.push_back((int)label_mat.at<float>(i));
  }

  calcClassMean(data_mat,label_vec,class_mean_arr,num_classes);

}
void SubspaceAnalysis::LDA::calcClassMean(cv::Mat& data_mat,std::vector<int>& label_vec,cv::Mat&  class_mean_arr,int& num_classes)
{

  std::vector<int>     samples_per_class(num_classes,0);


  int class_index;
  for(int i =0;i<data_mat.rows;i++)
  {
    cv::Mat data_row=data_mat.row(i);
    class_index=label_vec[i];
    cv::Mat mean_row=class_mean_arr.row(class_index);

    add(mean_row,data_row,mean_row);
    samples_per_class[class_index]++;
  }

  for (int i = 0; i < num_classes; i++)
  {
    cv::Mat mean_arr_row=class_mean_arr.row(i);
    mean_arr_row.convertTo(mean_arr_row,CV_64FC1,1.0/static_cast<double>(samples_per_class[i]));
  }

}

void SubspaceAnalysis::LDA::calcProjMatrix(cv::Mat& data_arr,std::vector<int>& label_vec )
{

    cv::Mat S_intra=cv::Mat::zeros(data_arr.cols,data_arr.cols,CV_64FC1);
    cv::Mat S_inter=cv::Mat::zeros(data_arr.cols,data_arr.cols,CV_64FC1);
  int class_index;

  for(int i=0;i<data_arr.rows;++i)
  {
    //reduce data matrix
    class_index=label_vec[i];
    cv::Mat  data_row =data_arr.row(i);
    cv::Mat  class_mean_row=class_mean_arr.row(class_index);
    cv::subtract(data_row,class_mean_row,data_row);
  }
  for(int c=0;c<num_classes_;c++)
  {
    cv::Mat temp;
    class_index=unique_labels_[c];
    cv::Mat  class_mean_row=class_mean_arr.row(class_index);
    cv::subtract(class_mean_row,mean,temp);
    cv::mulTransposed(temp,temp,true);
    cv::add(S_inter,temp,S_inter);

  }
   // //Intra class scatter
    cv::mulTransposed(data_arr,S_intra,true);

  cv::Mat S_intra_inv=S_intra.inv();

  cv::Mat P;
  gemm(S_intra_inv,S_inter,1.0,cv::Mat(),0.0,P);

  decomposeAsymmetricMatrix(P);

  return;

}


//---------------------------------------------------------------------------------
// ILDA
//---------------------------------------------------------------------------------
//
SubspaceAnalysis::ILDA::ILDA(cv::Mat& input_data,std::vector<int>& input_labels,int& num_classes,int& ss_dim)
{
  SubspaceAnalysis::unique_elements(input_labels,num_classes_,unique_labels_);
  cv::Mat data_work=input_data.clone();
  mean=cv::Mat::zeros(1,data_work.cols,CV_64FC1);
  calcDataMatMean(data_work,mean);
  num_classes_=num_classes;
  //class labels have to be in a vector in ascending order - duplicates are
  //removed internally
  //{0,0,1,2,3,3,4,5}

  class_mean_arr=cv::Mat::zeros(num_classes_,input_data.cols,CV_64FC1);
  calcClassMean(data_work,input_labels,class_mean_arr,num_classes_);

  calcProjMatrix(data_work,input_labels);

  eigenvecs=eigenvecs(cv::Rect(0,0,input_data.cols,ss_dim));
  eigenvals=eigenvals(cv::Rect(0,0,1,ss_dim)).t();
  //cv::normalize(eigenvecs,eigenvecs);
}
void SubspaceAnalysis::ILDA::calcProjMatrix(cv::Mat& data_arr,std::vector<int>& label_vec )
{

    cv::Mat S_intra=cv::Mat::zeros(data_arr.cols,data_arr.cols,CV_64FC1);
    cv::Mat S_inter=cv::Mat::zeros(data_arr.cols,data_arr.cols,CV_64FC1);
  int class_index;

  for(int i=0;i<data_arr.rows;++i)
  {
    //reduce data matrix
    class_index=label_vec[i];
    cv::Mat  data_row =data_arr.row(i);
    cv::Mat  class_mean_row=class_mean_arr.row(class_index);
    cv::subtract(data_row,class_mean_row,data_row);
  }
  for(int c=0;c<num_classes_;c++)
  {
    cv::Mat temp;
    class_index=unique_labels_[c];
    cv::Mat  class_mean_row=class_mean_arr.row(class_index);
    cv::subtract(class_mean_row,mean,temp);
    cv::mulTransposed(temp,temp,true);
    cv::add(S_inter,temp,S_inter);

  }
   // //Intra class scatter
    cv::mulTransposed(data_arr,S_intra,true);
    cv::Mat S_intra_inv=S_intra.inv();
    cv::Mat sigma=cv::Mat(1,num_classes_,CV_64FC1);

    for( int i=0;i<num_classes_;++i)
    {
      cv::Mat mu_i=class_mean_arr.row(i);
      for( int j=0;j<num_classes_;++j)
      {
      cv::Mat mu_j=class_mean_arr.row(j);

      cv::Mat delta_ij=((mu_j-mu_i)*S_intra_inv*(mu_j-mu_i).t());
      for(int k=0;k<data_arr.rows;k++)
      {

      }
      sigma.at<double>(j)=1/(delta_ij.at<double>(0,0));
      }

    }


    for(int  j=0;j<num_classes_;j++)
    {
    class_index=label_vec[j];
    cv::Mat  s_intra_row =S_intra.row(j);
    cv::Mat  s_inter_row =S_inter.row(j);
    double  sigma_j=sigma.at<double>(class_index);
    s_intra_row*=sigma_j;
    s_inter_row*=sigma_j;
    }

  S_intra_inv=S_intra.inv();

  cv::Mat P;
  gemm(S_intra_inv,S_inter,1.0,cv::Mat(),0.0,P);

  decomposeAsymmetricMatrix(P);

  return;

}
//---------------------------------------------------------------------------------
// PCA
//---------------------------------------------------------------------------------
//
SubspaceAnalysis::PCA::PCA(cv::Mat& input_data,int& ss_dim)
{

  ss_dim_=ss_dim;
  PCA_OpenCv(input_data,ss_dim);
  return;
  cv::Mat data_work=input_data.clone();
  mean=cv::Mat::zeros(1,data_work.cols,CV_64FC1);
  calcDataMatMean(data_work,mean);
  calcProjMatrix(data_work);
  //truncate eigenvectors and eigenvals
  eigenvecs=eigenvecs(cv::Rect(0,0,input_data.cols,ss_dim));
  eigenvals=eigenvals(cv::Rect(0,0,1,ss_dim)).t();
  //cv::normalize(eigenvecs,eigenvecs);

  //cv::Mat dummy;
  //eigenvecs.copyTo(dummy);
  //dummy.convertTo(dummy,CV_8UC1,1000);
  //dummy =dummy.reshape(1,dummy.rows*160);
  //cv::equalizeHist(dummy,dummy);
  //cv::imwrite("/home/goa-tz/Desktop/eigenfaces.jpg",dummy);


}
void SubspaceAnalysis::PCA::PCA_OpenCv(cv::Mat& input_data,int& ss_dim)
{
  ss_dim_=ss_dim;
  cv::PCA ocv_pca(input_data,cv::Mat(),CV_PCA_DATA_AS_ROW,ss_dim);

    eigenvecs=ocv_pca.eigenvectors.clone();
    eigenvals=ocv_pca.eigenvalues.clone();
    mean=ocv_pca.mean.reshape(1,1).clone();

  //truncate eigenvectors and eigenvals
  eigenvecs=eigenvecs(cv::Rect(0,0,input_data.cols,ss_dim));
  eigenvals=eigenvals(cv::Rect(0,0,1,ss_dim)).t();
  //cv::normalize(eigenvecs,eigenvecs);

}

void SubspaceAnalysis::PCA::calcProjMatrix(cv::Mat& data)
{
     cv::Mat data_row;
     for(int i=0;i<data.rows;++i)
     {
       //reduce data matrix - total Scatter matrix
       data_row =data.row(i);
       cv::subtract(data_row,mean,data_row);
     }

     decomposeSVD(data);
}


