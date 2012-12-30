#include"cob_people_detection/SSA.h"




SSA::SSA(std::vector<cv::Mat>& input_data,int& ss_dim):dimension(ss_dim)
{
  data=cv::Mat(input_data.size(),input_data[0].total(),CV_64FC1);
  calcDataMat(input_data,data);
  calcDataMatMean(data,mean);
}

void SSA::calcDataMat(std::vector<cv::Mat>& input_data,cv::Mat& data_mat)
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

void SSA::calcDataMatMean(cv::Mat& data,cv::Mat& mean)
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


void SSA::decomposeModel()
{
 //TODO include EigenvalueDecomposition of non symmetric matrices 
}


//---------------------------------------------------------------------------------
// LDA
//---------------------------------------------------------------------------------
LDA::LDA(std::vector<cv::Mat>& input_data,std::vector<int>& input_labels,int& ss_dim): SSA(input_data,ss_dim)
{

  calcClassMean(data,input_labels,class_means);
  calcModelMatrix(input_labels,model);

}
void LDA::calcClassMean(cv::Mat& data_mat,std::vector<int>& label_vec,std::vector<cv::Mat>&  mean_vec)
{

//TODO  get number of unique classes

  num_classes=0;

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

void LDA::calcModelMatrix(std::vector<int>& label_vec,cv::Mat& M)
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
void PCA::PCA(std::vector<cv::Mat>& input_data,int& ss_dim):SSA(input_data,ss_dim)
{

  calcModelMatrix(model);
  model=cv::Mat(data.rows,data.cols,CV_64FC1);
}

void PCA::calcModelMatrix(cv::Mat& M)
{

  for(int i=0;i<data.rows;++i)
  {
    //reduce data matrix - total Scatter matrix
    data_row =data.row(i);
    model_row=model.row(i);
    cv::subtract(data_row,mean,model_row);
  }
}


