
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
  cfg.align=true;
  cfg.resize=true;
  cfg.cvt2gray=true;

  FaceNormalizer fn(cfg);
  cv::Mat depth,img,xyz;
  std::string i_path;
  if(home)  i_path="/home/tom/git/care-o-bot/cob_people_perception/cob_people_detection/debug/eval/kinect3d_crops/";
  //if(home)  i_path="/home/tom/git/care-o-bot/cob_people_perception/cob_people_detection/debug/scenes/";
  else      i_path="/share/goa-tz/people_detection/eval/Kinect3DSelect/";

  i_path.append(argv[1]);
  i_path.append(".xml");

  fn.read_scene(xyz,img,i_path);

  cv::Mat wmat1,wmat2;
  img.copyTo(wmat1);
  img.copyTo(wmat2);
  fn.dump_img(wmat1,"original");
  cv::Size norm_size=cv::Size(160,160);
  //cv::cvtColor(wmat1,wmat1,CV_RGB2BGR);

  cv::Mat depth_res;
  fn.normalizeFace(wmat1,xyz,norm_size,depth);
  depth.convertTo(depth,CV_8UC1,255);
  cv::equalizeHist(depth,depth);

  //fn.normalizeFace(wmat1,norm_size);
 fn.dump_img(wmat1,"processedRGB");

  std::cout<<"..done\n";
  return 0;
}
