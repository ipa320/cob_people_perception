
#include"cob_people_detection/face_normalizer.h"
#include<iostream>
#include<opencv/cv.h>
#include<opencv/highgui.h>
int main(int argc, const char *argv[])
{
  bool home=false;


  std::cout<<"[FaceNormalizer] running scene no. "<<argv[1]<<"...\n";
  FaceNormalizer::FNConfig cfg;
  cfg.eq_ill=true;
  cfg.align=false;
  cfg.resize=true;
  cfg.cvt2gray=false;
  cfg.extreme_illumination_condtions=false;

  FaceNormalizer fn(cfg);
  cv::Mat depth,img,xyz;
  std::string i_path;
  if(home)  i_path="/home/tom/git/care-o-bot/cob_people_perception/cob_people_detection/debug/eval/kinect3d_crops/";
  //if(home)  i_path="/home/tom/git/care-o-bot/cob_people_perception/cob_people_detection/debug/scenes/";
  //else      i_path="/share/goa-tz/people_detection/normalization/test_scenes/";
  else      i_path="/share/goa-tz/people_detection/eval/";

  i_path.append(argv[1]);
  std::string pgm_path=i_path;
  std::cout<<"FN running on "<<pgm_path<<std::endl;

  img=cv::imread(pgm_path);
  cv::Mat wmat1;
  img.copyTo(wmat1);
  fn.dump_img(wmat1,"original");
  cv::Size norm_size=cv::Size(img.cols,img.rows);
  //cv::cvtColor(wmat1,wmat1,CV_RGB2BGR);

  cv::Mat depth_res;
  fn.normalizeFace(wmat1,norm_size);

  //fn.normalizeFace(wmat1,norm_size);
 fn.dump_img(wmat1,"ill_corr");

  std::cout<<"..done\n";
  return 0;
}
