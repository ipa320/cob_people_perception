
#include<cob_people_detection/subspace_analysis.h>
#include<opencv/cv.h>
#include<opencv/highgui.h>
#include<iostream>


int main(int argc, const char *argv[])
{


  int probe_img=atoi(argv[1]);
  std::cout<<"Probe INDEX="<<probe_img<<std::endl;
  
  std::string im1="/share/goa-tz/people_detection/debug/test_files/63.bmp";
  std::string im2="/share/goa-tz/people_detection/debug/test_files/64.bmp";
  std::string im3="/share/goa-tz/people_detection/debug/test_files/65.bmp";
  std::string im4="/share/goa-tz/people_detection/debug/test_files/66.bmp";
  std::string im5="/share/goa-tz/people_detection/debug/test_files/67.bmp";
  std::string im6="/share/goa-tz/people_detection/debug/test_files/68.bmp";
  std::string im7="/share/goa-tz/people_detection/debug/test_files/69.bmp";
  std::string im8="/share/goa-tz/people_detection/debug/test_files/70.bmp";
  std::string im9="/share/goa-tz/people_detection/debug/test_files/71.bmp";
  std::string im10="/share/goa-tz/people_detection/debug/test_files/75.bmp";
  std::string im11="/share/goa-tz/people_detection/debug/test_files/76.bmp";
  std::string im12="/share/goa-tz/people_detection/debug/test_files/77.bmp";
  std::string im13="/share/goa-tz/people_detection/debug/test_files/78.bmp";
  std::string im14="/share/goa-tz/people_detection/debug/test_files/79.bmp";
  std::string im15="/share/goa-tz/people_detection/debug/test_files/80.bmp";
  std::string im16="/share/goa-tz/people_detection/debug/test_files/81.bmp";
  std::string im17="/share/goa-tz/people_detection/debug/test_files/82.bmp";
  std::string im18="/share/goa-tz/people_detection/debug/test_files/83.bmp";
  std::string im19="/share/goa-tz/people_detection/debug/test_files/0.bmp";
  std::string im20="/share/goa-tz/people_detection/debug/test_files/1.bmp";
  std::string im21="/share/goa-tz/people_detection/debug/test_files/2.bmp";

  std::string imL1="/share/goa-tz/people_detection/debug/test_files/lighting/0_0.jpg";
  std::string imL2="/share/goa-tz/people_detection/debug/test_files/lighting/0_1.jpg";
  std::string imL3="/share/goa-tz/people_detection/debug/test_files/lighting/0_2.jpg";
  std::string imL4="/share/goa-tz/people_detection/debug/test_files/lighting/1_0.jpg";
  std::string imL5="/share/goa-tz/people_detection/debug/test_files/lighting/1_1.jpg";
  std::string imL6="/share/goa-tz/people_detection/debug/test_files/lighting/1_2.jpg";
  std::string imL7="/share/goa-tz/people_detection/debug/test_files/lighting/1_3.jpg";





  std::vector<std::string> in_vec;
  in_vec.push_back(im1);
  in_vec.push_back(im2);
  in_vec.push_back(im3);
 // in_vec.push_back(im4);
 // in_vec.push_back(im5);
 // in_vec.push_back(im6);
 // in_vec.push_back(im7);
 // in_vec.push_back(im8);
 // in_vec.push_back(im9);
  in_vec.push_back(im10);
  in_vec.push_back(im11);
  in_vec.push_back(im12);
 // in_vec.push_back(im13);
 // in_vec.push_back(im14);
 // in_vec.push_back(im15);
 // in_vec.push_back(im16);
 // in_vec.push_back(im17);
 // in_vec.push_back(im18);
 // in_vec.push_back(im19);
 // in_vec.push_back(im20);
 // in_vec.push_back(im21);
 // in_vec.push_back(imL1);
 // in_vec.push_back(imL2);
 // in_vec.push_back(imL3);
 // in_vec.push_back(imL4);
 // in_vec.push_back(imL5);
 // in_vec.push_back(imL6);
 // in_vec.push_back(imL7);

  cv::Mat probe_mat=cv::imread(in_vec[probe_img],0);
  cv::resize(probe_mat,probe_mat,cv::Size(120,120));
  probe_mat.convertTo(probe_mat,CV_64FC1);

  int indices[in_vec.size()-1];
  int k = 0;
  for(int j=0;j<in_vec.size();j++)
  {
    if(j!=probe_img)
    {
      indices[k]=j;
      k++;
    }
  }


 //int  class_labels[]={0,0,1,1,2,2};
 int  class_labels[]={0,0,0,1,1,1};



  int size_ts=sizeof(indices)/sizeof(indices[0]);
  std::cout<< "SIZE TRAINING SET"<<size_ts<<std::endl;




  std::vector<int> label_vec;
  std::vector<cv::Mat> img_vec;
  for(int i =0;i<size_ts;i++)
  {
    cv::Mat img=cv::Mat(120,120,CV_64FC1);
    std::cout<<in_vec[indices[i]]<<std::endl;
    img =cv::imread(in_vec[indices[i]],0);
    //cv::imshow("img",img);
    //cv::waitKey(0);
    cv::resize(img,img,cv::Size(120,120));
    img.convertTo(img,CV_64FC1);
    img_vec.push_back(img);

    label_vec.push_back(class_labels[indices[i]]);
  }

  int ss_dim=2;


  SubspaceAnalysis::Eigenfaces EF;
   EF.init(img_vec,label_vec,ss_dim);
  std::vector<cv::Mat> eigenvecsEF(ss_dim);
  cv::Mat eigenvalsEF,avgEF,projsEF;
  EF.retrieve(eigenvecsEF,eigenvalsEF,avgEF,projsEF);
  SubspaceAnalysis::dump_matrix(projsEF,"projEF");

  //double DFFS_EF;
  //cv::Mat feats_EF;
  int c_EF;
  cv::Mat coeff_EF;
  double DFFS_EF;
  EF.projectToSubspace(probe_mat,coeff_EF,DFFS_EF);
  EF.classify(coeff_EF,SubspaceAnalysis::CLASS_KNN,c_EF);
  std::cout<<"class EF KNN= "<<c_EF<<std::endl;
  EF.classify(coeff_EF,SubspaceAnalysis::CLASS_MIN_DIFFS,c_EF);
  std::cout<<"class EF DIFFS= "<<c_EF<<std::endl;

  SubspaceAnalysis::dump_matrix(coeff_EF,"sampleEF");

  SubspaceAnalysis::Fisherfaces FF;
   FF.init(img_vec,label_vec);

  cv::Mat eigenvalsFF,avgFF,projsFF;
  std::vector<cv::Mat> eigenvecsFF(ss_dim);
  FF.retrieve(eigenvecsFF,eigenvalsFF,avgFF,projsFF);
  SubspaceAnalysis::dump_matrix(projsFF,"projFF");

  //double DFFS;
  //cv::Mat feats;
  cv::Mat coeff_FF;
  int c_FF;
  double DFFS_FF;
  FF.projectToSubspace(probe_mat,coeff_FF,DFFS_FF);
  FF.classify(coeff_FF,SubspaceAnalysis::CLASS_KNN,c_FF);
  std::cout<<"class FF KNN= "<<c_FF<<std::endl;
  FF.classify(coeff_FF,SubspaceAnalysis::CLASS_MIN_DIFFS,c_FF);
  std::cout<<"class FF DIFFS= "<<c_FF<<std::endl;

  SubspaceAnalysis::dump_matrix(coeff_FF,"sampleFF");

return 0;
}
