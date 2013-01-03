#include"cob_people_detection/subspace_analysis.h"


//---------------------------------------------------------------------------------
// EIGENFACES
//---------------------------------------------------------------------------------
//
//
SubspaceAnalysis::Eigenfaces::Eigenfaces(std::vector<cv::mat>& img_vec,std::vector<int>label_vec,int& dim_ss)
{
  //check if input has the same size
  if(img_vec.size()!=label_vec.size())
  {
    //TODO: ROS ERROR
    std::cout<<"EIGFACE: input has to be the same sie\n";
    return;
  }

  //initialize all matrices
  model_data_=cv::Mat(img_vec.size(),img_vec[0].total(),CV_64FC1);
  avg_=cv::Mat(1,img_vec[0].total(),CV_64FC1);
  proj_model_data_=cv::Mat(img_vec.size(),dim_ss,CV_64FC1);
  avg_=cv::Mat(dim_ss,img_vec[0].total(),CV_64FC1);


  SubspaceAnalysis::calcDataMat(img_vec,model_data_);

  //initiate PCA
  pca_(model_data_,dim_ss);
  proj_=pca.eigenvecs.clone();
  avg_=pca.mean.clone();

  std::vector<double> DFFS;
  SubspaceAnalysis::projectToSubspace(model_data_,proj_model_data_,DFFS);

}


void SubspaceAnalysis::calcDataMat(std::vector<cv::Mat>& input_data,cv::Mat& data_mat)
{

  // convert input to data matrix,with images as rows

  for(int i=0;i<data_mat.rows;++i)
  {
    cv::Mat curr_row = data_mat.row(i);
    if(!input_data[0].isContinuous())
    {
      input_data[0].reshape(1,1).convertTo(curr_row,CV_64FC1);
    }
    else
    {
      input_data[0].clone().reshape(1,1).convertTo(curr_row,CV_64FC1);
    }
  }

  return;
}
SubspaceAnalysis::projectToSubspace(cv::Mat& src_mat,cv::Mat& dst_mat,std::vector<double>& DFFS)
{
  SubspaceAnalysis::project(src_mat,proj_,avg_,dst_mat);

  cv::Mat rec_mat;
  SubspaceAnalysis::reconstruct(dst_mat,proj_,avg_,rec_mat);
  SubspaceAnalysis::DFFS(src_mat,rec_mat,avg_,DFFS);
}
//TODO:
//provide interface for recosntruction and projection
//with autmatic generation of DFFS and DIFS rspectively
//
//
SubspaceAnalysis::Fisherfaces::Fisherfaces()
{
  //TODO:
  //initiate PCA
  //forward truncated data to LDA
  //get projection matrixw
}
//TODO:
//provide interface for recosntruction and projection
//with autmatic generation of DFFS and DIFS rspectively
//
//MAKE Sure every matrix is transformed to one image per row matrix"!!!!!!
//

void SubspaceAnalysis::DFFS(cv::Mat& orig_mat,cv::Mat& recon_mat,cv::Mat& avg,std::vector<double>& DFFS)
{

  cv::Mat temp=cv::Mat(orig_mat.rows,orig_mat.cols,orig_mat.type());
  orig_mat.copyTo(temp);
  DFFS.resize(recon_mat.rows);
  for(int i=0;i<src_mat.rows;i++)
  {
    cv::Mat recon_row=recon_mat.row(i);
    cv::Mat temp_row=temp.row(i);
    cv::subtract(temp_row,avg,temp_row);
    DFFS[i]= cv::norm(recon_row,orig_row,cv::NORM_L2);
  }
  return;
}
void SubspaceAnalysis::project(cv::Mat& src_mat,cv::Mat& proj_mat,cv::Mat& avg_mat,cv::Mat& coeff_mat)
{

  // reduce im mat by mean
  for(int i=0;i<src_mat.rows;i++)
  {
    cv::Mat im=src_mat.row(i);
    cv::subtract(im,avg_mat.reshape(1,1),im);
  }

  //calculate coefficients

  cv::gemm(im_mat,proj_mat,1.0,cv::Mat(),0.0,coeff_mat);

}

void SubspaceAnalysis::reconstruct(cv::Mat& coeffs,cv::Mat& rec_im)
{
  cv::gemm(coeffs,P,1.0,Mat(),0.0,rec_im,GEMM_2_T);
  for(int i=0;i<rec_im.rows;i++)
  {
    cv::Mat curr_row=rec_im.row(i);
    cv::add(curr_row,mean.reshape(1,1),curr_row);
  }
}

void SubspaceAnalysis::Eigenfaces::meanCoeffs(cv::Mat& coeffs,std::vector<int> label_vec,cv::Mat& mean_coeffs)
{
  std::vector<int> distinct_vec;
  bool unique=true;
  for(int i=0;i<label_vec.size();++i)
  {
    if(i==0)distinct_vec.push_back(label_vec[i]);continue;

    unique=true;
    for(int j=0;j<distinct_vec.size();j++)
    {
      if(label_vec[i]==distinct_vec[j]) unique =false;
    }
    if(unique==true)distinct_vec.push_back(label_vec[i]);
  }
  num_classes=distinct_vec.size();



  std::vector<cv::Mat> mean_coeffs(num_classes);
  std::vector<int>     samples_per_class(num_classes);

  //initialize vectors with zeros
  for( int i =0;i<num_classes;i++)
  {
   mean_coeffs[i]=cv::Mat::zeros(1,coeffs.cols,CV_64FC1);
   samples_per_class[i]=0;
  }

  int class_index;
  for(int i =0;i<data_mat.rows;i++)
  {
    cv::Mat curr_row=coeffs.row(i);
    class_index=label_vec[i];

    add(mean_coeffs[class_index],curr_row,mean_coeffs[class_index]);
    samples_per_class[class_index]++;
  }

  for (int i = 0; i < num_classes; i++) {
  mean_coeffs[i].convertTo(mean_vec[i],CV_64FC1,1.0/static_cast<double>(samples_per_class[i]));

  }

}


//---------------------------------------------------------------------------------
// SSA
//---------------------------------------------------------------------------------


SubspaceAnalysis::SSA::SSA(cv::Mat& data_mat,int& ss_dim):dimension(ss_dim)
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

  return;
}


void SubspaceAnalysis::SSA::decompose(cv::Mat& data_mat)
{
  EigenvalueDecomposition evd(data_mat);
  cv::Mat eigenvals_unsorted,eigenvecs_unsorted;
  eigenvals_unsorted=evd.eigenvalues();
  eigenvals.reshape(1,1);
  eigenvecs_unsorted=evd.eigenvectors();

  //sort eigenvals and eigenvecs

  cv::Mat sort_indices;
  cv::sortIdx(eigenvals_unsorted,sort_indices,CV_SORT_DESCENDING);//TODO: are flags just right

  cv::Mat temp;
  for(size_t i=0;i<sort_indices.total();i++)
  {
    temp = eigenvals_unsorted.col(sort_indices.at<int>(i));
    temp.copyTo(eigenvals.col(i));
    temp = eigenvecs_unsorted.col(sort_indices.at<int>(i));
    temp.copyTo(eigenvecs.col(i));
  }

  eigenvals=Mat(eigenvals,cv::Range::all(),cv::Range(0,dimension));
  eigenvecs=Mat(eigenvecs,cv::Range::all(),cv::Range(0,dimension));

}


//---------------------------------------------------------------------------------
// LDA
//---------------------------------------------------------------------------------
SubspaceAnalysis::LDA::LDA(std::vector<cv::Mat>& input_data,std::vector<int>& input_labels,int& ss_dim): SSA(input_data,ss_dim)
{

  //class labels have to be in a vector in ascending order - duplicates are
  //removed internally
  //{0,0,1,2,3,3,4,5}

  calcClassMean(data,input_labels,class_means);
  calcModelMatrix(input_labels,model);
  decompose(model);

}
void SubspaceAnalysis::LDA::calcClassMean(cv::Mat& data_mat,std::vector<int>& label_vec,std::vector<cv::Mat>&  mean_vec)
{

  std::vector<int> distinct_vec;
  bool unique=true;
  for(int i=0;i<label_vec.size();++i)
  {
    if(i==0)distinct_vec.push_back(label_vec[i]);continue;

    unique=true;
    for(int j=0;j<distinct_vec.size();j++)
    {
      if(label_vec[i]==distinct_vec[j]) unique =false;
    }
    if(unique==true)distinct_vec.push_back(label_vec[i]);
  }
  num_classes=distinct_vec.size();



  std::vector<cv::Mat> mean_of_class(num_classes);
  std::vector<int>     samples_per_class(num_classes);

  //initialize vectors with zeros
  for( int i =0;i<num_classes;i++)
  {
   mean_of_class[i]=cv::Mat::zeros(1,data_mat.cols,CV_64FC1);
   samples_per_class[i]=0;
  }

  int class_index;
  for(int i =0;i<data_mat.rows;i++)
  {
    cv::Mat data_row=data_mat.row(i);
    class_index=label_vec[i];

    add(mean_of_class[class_index],data_row,mean_of_class[class_index]);
    samples_per_class[class_index]++;
  }

  for (int i = 0; i < num_classes; i++) {
  mean_of_class[i].convertTo(mean_vec[i],CV_64FC1,1.0/static_cast<double>(samples_per_class[i]));

  }
}

void SubspaceAnalysis::LDA::calcModelMatrix(std::vector<int>& label_vec,cv::Mat& M)
{
 //reduce data matrix with class means and compute inter class scatter
  // inter class scatter
  cv::Mat S_inter=cv::Mat::zeros(data.rows,data.rows,CV_64FC1);
  cv::Mat temp;

  int class_index;
  cv::Mat data_row;
  for(int i=0;i<num_classes;++i)
  {
    //reduce data matrix
    class_index=label_vec[i];
    data_row =data.row(i);
    cv::subtract(data_row,class_means[class_index],data_row);
    //compute interclass scatte
    cv::subtract(class_means[class_index],mean,temp);
    cv::mulTransposed(temp,temp,true);
    cv::add(S_inter,temp,S_inter);
  }

  //intra-class scatter
  cv::Mat S_intra=cv::Mat::zeros(data.rows,data.rows,CV_64FC1);
  mulTransposed(data,S_intra,true);
  cv::Mat S_intra_inv=S_intra.inv();

  gemm(S_intra_inv,S_inter,1.0,cv::Mat(),0.0,M);

  return;

}

//---------------------------------------------------------------------------------
// PCA
//---------------------------------------------------------------------------------
//
SubspaceAnalysis::PCA::PCA(cv::Mat& input_data,int& ss_dim):SSA(input_data,ss_dim)
{
  data=input_data;
  calcProjMatrix();
}

void SubspaceAnalysis::PCA::calcProjMatrix()
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


