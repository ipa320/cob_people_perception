
#include<cob_people_detection/subspace_analysis.h>
#include<opencv/cv.h>
#include<opencv/highgui.h>
#include<iostream>


int main(int argc, const char *argv[])
{

  std::string im1="/share/goa-tz/people_detection/debug/results/3_norm.jpg";
  std::string im2="/share/goa-tz/people_detection/debug/results/5_norm.jpg";
  std::string im3="/share/goa-tz/people_detection/debug/results/9_norm.jpg";

  std::vector<std::string> in_vec;
  in_vec.push_back(im1);
  in_vec.push_back(im2);
  in_vec.push_back(im3);

  std::vector<cv::Mat> img_vec;
  for(int i =0;i<3;i++)
  {
    cv::Mat img =cv::imread(in_vec[i]);
    
    img.convertTo(img,CV_64FC1);
    cv::resize(img,img,cv::Size(120,120));

    img_vec.push_back(img);
  }


 for(int i =0;i<3;i++)
 {
   cv::Mat out =img_vec[i];
   cv::imshow("vec",out);
   cv::waitKey(500);
 }
  std::vector<int> label_vec;
  label_vec.push_back(0);
  label_vec.push_back(1);
  label_vec.push_back(2);
  int ss_dim=1;
  SubspaceAnalysis::Eigenfaces EF(img_vec,label_vec,ss_dim);
  return 0;
}
