
#include<cob_people_detection/subspace_analysis.h>
#include<cob_people_detection/face_normalizer.h>
#include<opencv/cv.h>
#include<opencv/highgui.h>
#include<iostream>
#include<fstream>






void preprocess(cv::Mat& img,FaceNormalizer* fn,bool normalize)
{
  double aspect_ratio=1;
  aspect_ratio=double(img.cols)/double(img.rows);
  cv::Size norm_size=cv::Size(round(120*aspect_ratio),120);
  //cv::Size norm_size=cv::Size(120,120);
  if(normalize)
  {
    fn->normalizeFace(img,norm_size);
    //cv::imshow("normalized",img);
    //cv::waitKey(5);

  }
  else
  {
    cv::resize(img,img,norm_size); 
  }




  img.convertTo(img,CV_64FC1);
}

int main(int argc, const char *argv[])
{

  FaceNormalizer* fn=new FaceNormalizer();

  // parse input arguments from command line
  std::string method_str,classifier_str;
  bool normalizer=false;
  if (argc==1)
  {
    method_str="FISHER";
    classifier_str="KNN";
    normalizer=false;
  }

  else if (argc==2)
  {
    method_str=argv[1];
    classifier_str="KNN";
    normalizer=false;
  }

  else if (argc==3)
  {
    method_str=argv[1];
    classifier_str=argv[2];
    normalizer=false;
  }

  else if (argc==4)
  {
    method_str=argv[1];
    classifier_str=argv[2];
    normalizer=true;
  }



  //  Configure input params for subspace analysis

  SubspaceAnalysis::Method method;
  SubspaceAnalysis::Classifier classifier;

  if(!method_str.compare("FISHER"))
  {
    std::cout<<"FISHER"<<std::endl;
    method = SubspaceAnalysis::METH_FISHER;
  }
  else if(!method_str.compare("IFLDA"))
  {
    std::cout<<"IFLDA"<<std::endl;
    method = SubspaceAnalysis::METH_IFLDA;
  }
  else if(!method_str.compare("EIGEN"))
  {
    std::cout<<"EIGEN"<<std::endl;
    method = SubspaceAnalysis::METH_EIGEN;
  }
  else
  {
    std::cout<<"ERROR: invalid method - use FISHER or EIGEN"<<std::endl;
  }

  if(!classifier_str.compare("KNN"))
  {
    std::cout<<"KNN"<<std::endl;
    classifier = SubspaceAnalysis::CLASS_KNN;
  }
  else if(!classifier_str.compare("DIFFS"))
  {
    std::cout<<"DIFFS"<<std::endl;
    classifier = SubspaceAnalysis::CLASS_MIN_DIFFS;
  }
  else if(!classifier_str.compare("SVM"))
  {
    std::cout<<"SVM"<<std::endl;
    classifier = SubspaceAnalysis::CLASS_SVM;
  }
  else
  {
    std::cout<<"ERROR: invalid classifier - use KNN or DIFFS or SVM"<<std::endl;
  }


  //HOME
  //std::string training_set_path="/home/tom/git/care-o-bot/cob_people_perception/cob_people_detection/debug/eval/training_set_list";
  //std::string probe_file_path="/home/tom/git/care-o-bot/cob_people_perception/cob_people_detection/debug/eval/probe_file_list";
  //IPA
  std::string training_set_path="/share/goa-tz/people_detection/eval/eval_tool_files/training_set_list";
  std::string probe_file_path="/share/goa-tz/people_detection/eval/eval_tool_files/probe_file_list";


  //read probe file
  std::ifstream probe_file_stream(probe_file_path.c_str());

  std::string probe_file;
  std::vector<std::string> probe_file_vec;

  while(probe_file_stream >> probe_file)
  {
      //std::cout<<probe_file<<std::endl;
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

  int num_classes = label;




  // load training images
 std::vector<cv::Mat> img_vec;
 for(int i =0;i<in_vec.size();i++)
 {
   cv::Mat img;
   img =cv::imread(in_vec[i],0);
   //cv::imshow("img",img);
   //cv::waitKey(0);
   preprocess(img,fn,normalizer);
   img_vec.push_back(img);

 }


// load test images
 std::vector<cv::Mat> probe_mat_vec;
 for(int i =0 ;i<probe_file_vec.size();i++)
 {
  std::stringstream ostr,nstr;
  nstr<<"/share/goa-tz/people_detection/eval/picdump/";
  ostr<<nstr.str().c_str()<<i<<"_orig"<<".jpg";

  cv::Mat probe_mat=cv::imread(probe_file_vec[i],0);

  cv::imwrite(ostr.str().c_str(),probe_mat);

  preprocess(probe_mat,fn,normalizer);

  cv::Mat oimg;
  probe_mat.convertTo(oimg,CV_8UC1);
  //cv::equalizeHist(oimg,oimg);
  nstr<<i<<"_norm"<<".jpg";
  cv::imwrite(nstr.str().c_str(),oimg);


  probe_mat_vec.push_back(probe_mat);
 }

  std::cout<<"Size Training Set= "<<img_vec.size()<<std::endl;

  int ss_dim=num_classes;

  SubspaceAnalysis::FishEigFaces EFF;

  // calculate Model
  EFF.init(img_vec,label_vec,ss_dim,method);
  std::cout<<"EFF model computed"<<std::endl;

  //open output file
  std::string path = "/share/goa-tz/people_detection/eval/eval_tool_files/classified_output";
  std::ofstream os(path.c_str() );

  for(int i=0;i<probe_mat_vec.size();i++)
  {
    cv::Mat probe = probe_mat_vec[i];
  int c_EFF;
  cv::Mat coeff_EFF;
  double DFFS_EFF;
  EFF.projectToSubspace(probe,coeff_EFF,DFFS_EFF);
  EFF.classify(coeff_EFF,classifier,c_EFF);
  os<<c_EFF<<"\n";
  }
  os.close();


  std::cout<<"EFF classified"<<std::endl;


return 0;
}
