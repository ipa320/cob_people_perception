
#include<cob_people_detection/subspace_analysis.h>
#include<opencv/cv.h>
#include<opencv/highgui.h>
#include<iostream>
#include<fstream>


int main(int argc, const char *argv[])
{



  //HOME
  //std::string training_set_path="/home/tom/git/care-o-bot/cob_people_perception/cob_people_detection/debug/eval/training_set_list";
  //std::string probe_file_path="/home/tom/git/care-o-bot/cob_people_perception/cob_people_detection/debug/eval/probe_file_list";
  //IPA
  std::string training_set_path="/share/goa-tz/people_detection/eval/training_set_list";
  std::string probe_file_path="/share/goa-tz/people_detection/eval/probe_file_list";


  //read probe file
  std::ifstream probe_file_stream(probe_file_path.c_str());

  std::string probe_file;
  std::vector<std::string> probe_file_vec;

  while(probe_file_stream >> probe_file)
  {
      std::cout<<probe_file<<std::endl;
      probe_file_vec.push_back(probe_file);

      }



  // read training set
  std::ifstream in_file(training_set_path.c_str());

  std::string img_file;

  int label = 0;
  std::vector<std::string> in_vec;
  std::vector<int> label_vec;

  while(in_file >> img_file)
  {
    if( std::strcmp(img_file.c_str(),"$$")==0)
    {

      label++;
    }
    else
    { 
      in_vec.push_back(img_file);
      label_vec.push_back(label);
    }

      }





 // std::vector<int> label_vec;
 std::vector<cv::Mat> img_vec;
 for(int i =0;i<in_vec.size();i++)
 {
   cv::Mat img;
   img =cv::imread(in_vec[i],0);
   //cv::imshow("img",img);
   //cv::waitKey(0);
   cv::resize(img,img,cv::Size(120,120));
   img.convertTo(img,CV_64FC1);
   img_vec.push_back(img);

 }

 std::vector<cv::Mat> probe_mat_vec;
 for(int i =0 ;i<probe_file_vec.size();i++)
 {
  cv::Mat probe_mat=cv::imread(probe_file_vec[i],0);
  cv::resize(probe_mat,probe_mat,cv::Size(120,120));
  probe_mat.convertTo(probe_mat,CV_64FC1);
  probe_mat_vec.push_back(probe_mat);
 }




  std::cout<<"Size Training Set= "<<img_vec.size()<<std::endl;


  int ss_dim=2;
  //SubspaceAnalysis::Eigenfaces EF;
  // EF.init(img_vec,label_vec,ss_dim);
  //std::vector<cv::Mat> eigenvecsEF(ss_dim);
  //cv::Mat eigenvalsEF,avgEF,projsEF;
  //EF.retrieve(eigenvecsEF,eigenvalsEF,avgEF,projsEF,cv::Size(img_vec[0].cols,img_vec[0].rows));
  //SubspaceAnalysis::dump_matrix(projsEF,"projEF");


  //SubspaceAnalysis::Fisherfaces FF;
  // FF.init(img_vec,label_vec);

  //cv::Mat eigenvalsFF,avgFF,projsFF;
  //std::vector<cv::Mat> eigenvecsFF(ss_dim);
  //FF.retrieve(eigenvecsFF,eigenvalsFF,avgFF,projsFF);
  //SubspaceAnalysis::dump_matrix(projsFF,"projFF");

  SubspaceAnalysis::FishEigFaces EFF;

  EFF.init(img_vec,label_vec,ss_dim,SubspaceAnalysis::METH_FISHER);


  for(int i=0;i<probe_mat_vec.size();i++)
  {
    cv::Mat probe = probe_mat_vec[i];
  ////double DFFS;
  ////cv::Mat feats;
  //cv::Mat coeff_FF;
  //int c_FF;
  //double DFFS_FF;
  //FF.projectToSubspace(probe,coeff_FF,DFFS_FF);
  //FF.classify(coeff_FF,SubspaceAnalysis::CLASS_SVM,c_FF);
  //std::cout<<"class FF SVM= "<<c_FF<<std::endl;
  //FF.classify(coeff_FF,SubspaceAnalysis::CLASS_KNN,c_FF);
  //std::cout<<"class FF KNN= "<<c_FF<<std::endl;
  //FF.classify(coeff_FF,SubspaceAnalysis::CLASS_MIN_DIFFS,c_FF);
  //std::cout<<"class FF DIFFS= "<<c_FF<<std::endl;

  //SubspaceAnalysis::dump_matrix(coeff_FF,"sampleFF");
  //std::cout<<"--------------------------\n";

  //int c_EF;
  //cv::Mat coeff_EF;
  //double DFFS_EF;
  //EF.projectToSubspace(probe,coeff_EF,DFFS_EF);
  //EF.classify(coeff_EF,SubspaceAnalysis::CLASS_SVM,c_EF);
  //std::cout<<"class EF SVM= "<<c_EF<<std::endl;
  //EF.classify(coeff_EF,SubspaceAnalysis::CLASS_KNN,c_EF);
  //std::cout<<"class EF KNN= "<<c_EF<<std::endl;
  //EF.classify(coeff_EF,SubspaceAnalysis::CLASS_MIN_DIFFS,c_EF);
  //std::cout<<"class EF DIFFS= "<<c_EF<<std::endl;

  //SubspaceAnalysis::dump_matrix(coeff_EF,"sampleEF");

  //std::cout<<"--------------------------\n";
  int c_EFF;
  cv::Mat coeff_EFF;
  double DFFS_EFF;
  EFF.projectToSubspace(probe,coeff_EFF,DFFS_EFF);
  EFF.classify(coeff_EFF,SubspaceAnalysis::CLASS_SVM,c_EFF);
  std::cout<<"class EFF SVM= "<<c_EFF<<std::endl;
  EFF.classify(coeff_EFF,SubspaceAnalysis::CLASS_KNN,c_EFF);
  std::cout<<"class EFF KNN= "<<c_EFF<<std::endl;
  EFF.classify(coeff_EFF,SubspaceAnalysis::CLASS_MIN_DIFFS,c_EFF);
  std::cout<<"class EFF DIFFS= "<<c_EFF<<std::endl;

  SubspaceAnalysis::dump_matrix(coeff_EFF,"sampleEFF");
  }

return 0;
}
