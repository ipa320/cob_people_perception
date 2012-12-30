#include"cob_people_detection/subspace_analysis.h"


SubspaceAnalysis::SSA::SSA(std::vector<cv::Mat>& input_data,int& ss_dim):dimension(ss_dim)
{
  data=cv::Mat(input_data.size(),input_data[0].total(),CV_64FC1);
  calcDataMat(input_data,data);
  calcDataMatMean(data,mean);
}

void SubspaceAnalysis::SSA::calcDataMat(std::vector<cv::Mat>& input_data,cv::Mat& data_mat)
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


void SubspaceAnalysis::SSA::decomposeModel()
{
  EigenvalueDecomposition evd(model);
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
  decomposeModel();

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
SubspaceAnalysis::PCA::PCA(std::vector<cv::Mat>& input_data,int& ss_dim):SSA(input_data,ss_dim)
{
  model=cv::Mat(data.rows,data.cols,CV_64FC1);
  calcModelMatrix(model);
  decomposeModel();
}

void SubspaceAnalysis::PCA::calcModelMatrix(cv::Mat& M)
{

  cv::Mat data_row,model_row;
  for(int i=0;i<data.rows;++i)
  {
    //reduce data matrix - total Scatter matrix
    data_row =data.row(i);
    model_row=model.row(i);
    cv::subtract(data_row,mean,model_row);
  }
}


