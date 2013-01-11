#include<cob_people_detection/subspace_analysis.h>


void SubspaceAnalysis::dump_matrix(cv::Mat& mat,std::string filename)
{
  std::string path = "/share/goa-tz/people_detection/debug/data/";
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

void SubspaceAnalysis:: mat_info(cv::Mat& mat)
{
  //for(int r=0;r<mat.rows;r++)
  //{
  //  for(int c=0;c<mat.cols;c++)
  //  {
  //    std::cout<<mat.at<double>(r,c)<<std::endl;
  //  }
  //}
  std::cout<<"Matrix info:\n";
  std::cout<<"rows= "<<mat.rows<<"  cols= "<<mat.cols<<std::endl;
  std::cout<<"Type = "<<mat.type()<<std::endl;
  std::cout<<"Channels = "<<mat.depth()<<std::endl;
}

//---------------------------------------------------------------------------------
//  XFace XFaces
//---------------------------------------------------------------------------------
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
void SubspaceAnalysis::XFaces::project(cv::Mat& src_mat,cv::Mat& proj_mat,cv::Mat& avg_mat,cv::Mat& coeff_mat)
{


  // reduce im mat by mean
  for(int i=0;i<src_mat.rows;i++)
  {
    cv::Mat im=src_mat.row(i);
    cv::subtract(im,avg_mat,im);
  }

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


    double* src_ptr;
    double*  dst_ptr;
    cv::Mat src_mat;
    for(int i = 0;i<input_data.size();i++)
    {
      input_data[i].copyTo(src_mat);

      src_ptr = src_mat.ptr<double>(0,0);
      //src_ptr = input_data[i].ptr<double>(0,0);
      dst_ptr = data_mat.ptr<double>(i,0);

      for(int j=0;j<input_data[i].rows;j++)
      {
        src_ptr=src_mat.ptr<double>(j,0);
        //src_ptr=input_data[i].ptr<double>(j,0);
        for(int col=0;col<input_data[i].cols;col++)
        {
        *dst_ptr=*src_ptr;
        src_ptr++;
        dst_ptr++;
        }
      }

    }


    //cv::Mat src_mat=cv::Mat(120,120,CV_64FC1);
    //src_mat= input_data[0].clone();
    ////src_mat.convertTo(src_mat,CV_64F);
    //std::cout<<"src"<<src_mat.rows<<","<<src_mat.cols<<std::endl;
    //src_mat=src_mat.reshape(1,1);
    //src_mat.clone();
    //std::cout<<"src"<<src_mat.rows<<","<<src_mat.cols<<std::endl;
    //cv::Mat dst_mat;
    //dst_mat.push_back(src_mat);

    //std::cout<<"dst"<<dst_mat.rows<<","<<dst_mat.cols<<std::endl;
    //dst_mat=dst_mat.reshape(1,120);
    //dst_mat.convertTo(dst_mat,CV_8UC1);
    //cv::imshow("DST",dst_mat);
    //cv::waitKey(0);

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


void SubspaceAnalysis::XFaces::classify(cv::Mat& src_mat,int class_index)
{
  cv::Mat src_arr=cv::Mat(1,src_mat.total(),CV_64FC1);
  mat2arr(src_mat,src_arr);
  std::vector<double> DIFS_vec;

  cv::Mat coeff_mat=cv::Mat(src_arr.rows,ss_dim_,CV_64FC1);
  project(src_arr,eigenvector_arr_,avg_arr_,coeff_mat);
  calcDIFS(coeff_mat,DIFS_vec);

  double min_val;
  int   min_index;
  for(int i=0;i<DIFS_vec.size();i++)
  {
    min_val=-1;
    min_index=-1;
    if(DIFS_vec[i] < min_val)
    {
      min_index=i;
      min_val=DIFS_vec[i];
    }
  }

  if(min_index != -1)
  {
    class_index=min_val;
  }

}


void SubspaceAnalysis::XFaces::calcDIFS(cv::Mat& probe_mat,std::vector<double>& DIFS)
{
     double temp;
    for(int r=0;r<proj_model_data_arr_.rows;r++)
    {
      cv::Mat model_mat=proj_model_data_arr_.row(r);
      temp=cv::norm(probe_mat,model_mat,cv::NORM_L2);
      DIFS.push_back(temp);
    }
    return;
}

//---------------------------------------------------------------------------------
// EIGENFACES
//---------------------------------------------------------------------------------
//
//
void SubspaceAnalysis::Eigenfaces::init(std::vector<cv::Mat>& img_vec,int& red_dim)
{

  ss_dim_=red_dim;

  //check if input has the same size
  if(img_vec.size()<ss_dim_+1)
  {
    //TODO: ROS ERROR
    std::cout<<"EIGFACE: Invalid subspace dimension\n";
    return;
  }

  //initialize all matrices
  model_data_arr_=cv::Mat(img_vec.size(),img_vec[0].total(),CV_64FC1);
  avg_arr_=cv::Mat(1,img_vec[0].total(),CV_64FC1);
  proj_model_data_arr_=cv::Mat(img_vec.size(),ss_dim_,CV_64FC1);
  eigenvector_arr_=cv::Mat(ss_dim_,img_vec[0].total(),CV_64FC1);
  eigenvalue_arr_=cv::Mat(ss_dim_,ss_dim_,CV_64FC1);

  calcDataMat(img_vec,model_data_arr_);

  //cv::Mat dummy;
  //model_data_arr_.copyTo(dummy);
  //dummy.convertTo(dummy,CV_8UC1);
  //dummy =dummy.reshape(1,dummy.rows*160);
  //cv::imshow("reconstuction",dummy);
  //cv::imwrite("/home/goa-tz/Desktop/model_data_.jpg",dummy);


  //initiate PCA
  pca_=SubspaceAnalysis::PCA(model_data_arr_,ss_dim_);

  eigenvector_arr_=pca_.eigenvecs;
  eigenvalue_arr_=pca_.eigenvals;
  avg_arr_=pca_.mean;



 // cv::Mat ss=model_data_(cv::Rect(0,0,120*120,3));
 // projectToSubspace(ss,proj_model_data_,DFFS);
 // dump_matrix(proj_model_data_,"projection1");

 // cv::Mat ss2=model_data_(cv::Rect(0,2,120*120,3));
 // projectToSubspace(ss2,proj_model_data_,DFFS);
 // dump_matrix(proj_model_data_,"projection2");

  project(model_data_arr_,eigenvector_arr_,avg_arr_,proj_model_data_arr_);

}



void SubspaceAnalysis::Eigenfaces::projectToSubspace(cv::Mat& src_mat,cv::Mat& dst_mat,double& DFFS)
{

  cv::Mat src_arr;
  mat2arr(src_mat,src_arr);

  project(src_arr,eigenvector_arr_,avg_arr_,dst_mat);

  cv::Mat rec_mat=cv::Mat(src_arr.rows,eigenvector_arr_.rows,CV_64FC1);
  reconstruct(dst_mat,eigenvector_arr_,avg_arr_,rec_mat);


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
void SubspaceAnalysis::Eigenfaces::meanCoeffs(cv::Mat& coeffs,std::vector<int>& label_vec,cv::Mat& mean_coeffs)
{
  std::vector<int> distinct_vec;
  bool unique=true;
  for(int i=0;i<label_vec.size();++i)
  {

    if(i!=0)
    {
    unique=true;
    for(int j=0;j<distinct_vec.size();j++)
    {
      if(label_vec[i]==distinct_vec[j]) unique =false;
    }
    }
    if(unique==true)distinct_vec.push_back(label_vec[i]);
  }
  int num_classes=distinct_vec.size();


  mean_coeffs=cv::Mat::zeros(num_classes,coeffs.cols,CV_64FC1);
  std::vector<int>     samples_per_class(num_classes);

  //initialize vectors with zeros
  for( int i =0;i<num_classes;i++)
  {
   samples_per_class[i]=0;
  }

  int class_index;
  for(int i =0;i<coeffs.rows;i++)
  {
    cv::Mat curr_row=coeffs.row(i);
    class_index=label_vec[i];

    cv::Mat mean_row=mean_coeffs.row(class_index);

    add(mean_row,curr_row,mean_row);
    samples_per_class[class_index]++;
  }

  for (int i = 0; i < num_classes; i++) {
  cv::Mat mean_row=mean_coeffs.row(i);
  mean_row.convertTo(mean_row,CV_64FC1,1.0/static_cast<double>(samples_per_class[i]));

  }
}



//---------------------------------------------------------------------------------
// FIsherfaces
//---------------------------------------------------------------------------------


void SubspaceAnalysis::Fisherfaces::init(std::vector<cv::Mat>& img_vec,std::vector<int>& label_vec)
{
  //initialize all matrices
  model_data_arr_=cv::Mat(img_vec.size(),img_vec[0].total(),CV_64FC1);
  avg_arr_=cv::Mat(1,img_vec[0].total(),CV_64FC1);


  //get number of classes and distinct classes
  std::vector<int> distinct_vec;
  bool unique=true;
  for(int i=0;i<label_vec.size();++i)
  {

    if(i!=0)
    {
    unique=true;
    for(int j=0;j<distinct_vec.size();j++)
    {
      if(label_vec[i]==distinct_vec[j]) unique =false;
    }
    }
    if(unique==true)distinct_vec.push_back(label_vec[i]);
  }

  //number of classes
  num_classes_=distinct_vec.size();

  //subspace dimension is num classes -1
  ss_dim_=num_classes_ -1;
  // pca dimension  is N- num classes
  int pca_dim=model_data_arr_.rows-num_classes_;


  calcDataMat(img_vec,model_data_arr_);

  // Reduce dimension to  N - c via PCA
  pca_=SubspaceAnalysis::PCA(model_data_arr_,pca_dim);

  cv::Mat proj_model_data_arr_PCA=cv::Mat(model_data_arr_.rows,pca_dim,CV_64FC1);
  project(model_data_arr_,pca_.eigenvecs,pca_.mean,proj_model_data_arr_PCA);

  // get projection matrix pca
  cv::Mat P_pca=cv::Mat(pca_dim,img_vec[0].total(),CV_64FC1);
  P_pca=pca_.eigenvecs;
  avg_arr_=pca_.mean;


  //perform lda
  lda_=SubspaceAnalysis::LDA(proj_model_data_arr_PCA,label_vec,num_classes_,ss_dim_);


  // get projection matrix lda
  cv::Mat P_lda=cv::Mat(ss_dim_,pca_dim,CV_64FC1);
  P_lda=lda_.eigenvecs;


  // combine projection matrices
  cv::gemm(P_pca.t(),P_lda.t(),1.0,cv::Mat(),0.0,eigenvector_arr_);

  eigenvector_arr_=eigenvector_arr_.t();

 // cv::Mat ss=model_data_(cv::Rect(0,0,120*120,3));
 // projectToSubspace(ss,proj_model_data_,DFFS);
 // dump_matrix(proj_model_data_,"projection1");

 // cv::Mat ss2=model_data_(cv::Rect(0,2,120*120,3));
 // projectToSubspace(ss2,proj_model_data_,DFFS);
 // dump_matrix(proj_model_data_,"projection2");

  proj_model_data_arr_=cv::Mat(img_vec.size(),ss_dim_,CV_64FC1);


  project(model_data_arr_,eigenvector_arr_,avg_arr_,proj_model_data_arr_);


  dump_matrix(proj_model_data_arr_,"FF_proj");

}



//---------------------------------------------------------------------------------
// SSA
//---------------------------------------------------------------------------------


SubspaceAnalysis::SSA::SSA(cv::Mat& data_mat)
{
  calcDataMatMean(data_mat,mean);
}


void SubspaceAnalysis::SSA::calcDataMatMean(cv::Mat& data,cv::Mat& mean)
{
  cv::Mat row_cum=cv::Mat::zeros(1,data.cols,CV_64FC1);
  for(int i=0;i<data.rows;++i)
  {
    cv::Mat data_row=data.row(i);
    //calculate mean
    cv::add(row_cum,data_row,row_cum);
  }
  row_cum.convertTo(mean,CV_64F,1.0/static_cast<double>(data.rows));

  //mean.convertTo(mean,CV_8UC1);
  //mean=mean.reshape(1,120);
  //cv::imshow("mean",mean);
  //cv::waitKey(0);
  //return;
}


void SubspaceAnalysis::SSA::decompose(cv::Mat& data_mat)
{

  data_mat.convertTo(data_mat,CV_64F,1/sqrt(3));
  cv::SVD svd(data_mat.t());
  eigenvecs=svd.u;
  //svd.u.copyTo(eigenvecs);
  eigenvecs=eigenvecs.t();
  //svd.w.copyTo(eigenvals);
  eigenvals=svd.w;


}




//---------------------------------------------------------------------------------
// LDA
//---------------------------------------------------------------------------------
SubspaceAnalysis::LDA::LDA(cv::Mat& input_data,std::vector<int>& input_labels,int& num_classes,int& ss_dim): SSA(input_data)
{

  cv::Mat data_work=input_data.clone();
  num_classes_=num_classes;
  //class labels have to be in a vector in ascending order - duplicates are
  //removed internally
  //{0,0,1,2,3,3,4,5}

  class_mean_arr=cv::Mat(num_classes_,input_data.cols,CV_64FC1);
  calcClassMean(data_work,input_labels,class_mean_arr);

  calcProjMatrix(data_work,input_labels);

  eigenvecs=eigenvecs(cv::Rect(0,0,input_data.cols,ss_dim));
  eigenvals=eigenvals(cv::Rect(0,0,1,ss_dim)).t();
  cv::normalize(eigenvecs,eigenvecs);

}
void SubspaceAnalysis::LDA::calcClassMean(cv::Mat& data_mat,std::vector<int>& label_vec,cv::Mat&  class_mean_arr)
{

  std::vector<cv::Mat> mean_of_class(num_classes_,cv::Mat::zeros(1,data_mat.cols,CV_64FC1));
  std::vector<int>     samples_per_class(num_classes_,0);


  int class_index;
  for(int i =0;i<data_mat.rows;i++)
  {
    cv::Mat data_row=data_mat.row(i);
    class_index=label_vec[i];

    add(mean_of_class[class_index],data_row,mean_of_class[class_index]);
    samples_per_class[class_index]++;
  }

  for (int i = 0; i < num_classes_; i++) {
    cv::Mat mean_arr_row=class_mean_arr.row(i);
    mean_of_class[i].convertTo(mean_arr_row,CV_64FC1,1.0/static_cast<double>(samples_per_class[i]));
  }



}

void SubspaceAnalysis::LDA::calcProjMatrix(cv::Mat& data_arr,std::vector<int>& label_vec )
{
 //reduce data matrix with class means and compute inter class scatter
  // inter class scatter

    cv::Mat S_intra=cv::Mat::zeros(data_arr.cols,data_arr.cols,CV_64FC1);
    cv::Mat S_inter=cv::Mat::zeros(data_arr.cols,data_arr.cols,CV_64FC1);
  int class_index;
  for(int i=0;i<num_classes_;++i)
  {
    //reduce data matrix
    class_index=label_vec[i];
    cv::Mat  data_row =data_arr.row(i);
    cv::Mat  class_mean_row=class_mean_arr.row(class_index);
    cv::subtract(data_row,class_mean_row,data_row);


    cv::Mat temp;
    cv::subtract(class_mean_row,mean,temp);
    cv::mulTransposed(temp,temp,true);
    cv::add(S_inter,temp,S_inter);

  }
    //Intra class scatter
    cv::mulTransposed(data_arr,S_intra,true);

    //compute interclass scatte

 // for(int i=0;i<num_classes_;i++)
 // {
 //   cv::Mat temp;
 //   cv::subtract(mean_row,mean,temp);
 //   cv::mulTransposed(temp,temp,true);
 //   cv::add(S_inter,temp,S_inter);
 // }

  cv::Mat S_intra_inv=S_intra.inv();

  cv::Mat P;
  gemm(S_intra_inv,S_inter,1.0,cv::Mat(),0.0,P);

  decompose(P);

  return;

}

//---------------------------------------------------------------------------------
// PCA
//---------------------------------------------------------------------------------
//
SubspaceAnalysis::PCA::PCA(cv::Mat& input_data,int& ss_dim):SSA(input_data)
{
  cv::Mat data_work=input_data.clone();
  calcProjMatrix(data_work);
  //truncate eigenvectors and eigenvals
  eigenvecs=eigenvecs(cv::Rect(0,0,input_data.cols,ss_dim));
  eigenvals=eigenvals(cv::Rect(0,0,1,ss_dim)).t();
  cv::normalize(eigenvecs,eigenvecs);

  //cv::Mat dummy;
  //eigenvecs.copyTo(dummy);
  //dummy.convertTo(dummy,CV_8UC1,1000);
  //dummy =dummy.reshape(1,dummy.rows*160);
  //cv::equalizeHist(dummy,dummy);
  //cv::imwrite("/home/goa-tz/Desktop/eigenfaces.jpg",dummy);


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

     decompose(data);
}


