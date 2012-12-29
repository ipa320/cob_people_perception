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




//---------------------------------------------------------------------------------
// LDA
//---------------------------------------------------------------------------------
LDA::LDA(std::vector<cv::Mat>& input_data,std::vector<int>& input_labels,int& ss_dim): SSA(input_data,ss_dim)
{

  calcClassMean(data,input_labels,class_means);

}
void LDA::calcClassMean(cv::Mat& data_mat,std::vector<int>& label_vec,std::vector<cv::Mat>&  mean_vec)
{

//TODO  get number of unique classes
  
  int num_classes=0;
  std::vector<int> unique_classes;
  for(int i=0;i<label_vec.size();++i)
  {
    if(i==0)unique_classes.push_back(label_vec[i]);
    // TODO GET UNIQUE ELEMENTS
  }
  

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

