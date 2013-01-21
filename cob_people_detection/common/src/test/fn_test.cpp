
#include"cob_people_detection/face_normalizer.h"
#include<iostream>
#include<opencv/cv.h>
#include<opencv/highgui.h>
int main(int argc, const char *argv[])
{
  std::cout<<"[FaceNormalizer] running scene no. "<<argv[1]<<"...";
  FaceNormalizer fn;
  cv::Mat depth,img;
  cv::Vec2f offset;
  //std::string i_path="/share/goa-tz/people_detection/debug/scenes/scene";
  std::string i_path="/home/tom/git/care-o-bot/cob_people_perception/cob_people_detection/debug/scenes/scene";
  i_path.append(argv[1]);
  i_path.append(".xml");

  fn.read_scene(depth,img,offset,i_path);

  cv::Mat wmat1,wmat2;
  img.copyTo(wmat1);
  img.copyTo(wmat2);
  fn.dump_img(wmat1,"original");
  cv::Size norm_size=cv::Size(120,120);
  //cv::cvtColor(wmat1,wmat1,CV_RGB2BGR);
  fn.normalizeFace(wmat1,depth,norm_size,offset);
  fn.dump_img(wmat1,"processedRGBD");
  fn.dump_img(depth,"processedDEPTH");
 // fn.normalizeFace(wmat2,rows);
 // fn.dump_img(wmat2,"processedRGB");

  std::cout<<"..done\n";
  return 0;
}
