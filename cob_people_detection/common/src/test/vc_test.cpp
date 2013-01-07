#include<opencv/cv.h>
#include<opencv/highgui.h>
#include <iostream>
#include<cob_people_detection/virtual_camera.h>
#include<cob_people_detection/face_normalizer.h>

int main(int argc, const char *argv[])
{
  std::cout<<"[Virtual CAmera] running scene no. "<<argv[1]<<"...";
  FaceNormalizer fn;
  cv::Mat depth,img;
  cv::Vec2f offset;
  std::string i_path="/home/tom/git/care-o-bot/cob_people_perception/cob_people_detection/debug/scenes/scene";
  i_path.append(argv[1]);
  i_path.append(".xml");

  fn.read_scene(depth,img,offset,i_path);


  VirtualCamera vc;

  std::vector<cv::Point2f> img_pts;
  std::vector<cv::Point3f> obj_pts;
  fn.get_feature_correspondences(img,depth,img_pts,obj_pts);
  vc.calc_extrinsics(obj_pts,img_pts,true);


  for (int i = 0;  i < 200;  i++)
  {
  cv::Mat res=cv::Mat::zeros(480,640,CV_8UC3);
    vc.sample_pc(depth,img,res);
    cv::Vec3f rotation=cv::Vec3f(i*0.02,0,0);


    cv::imshow("trans",res);
    cv::waitKey(100);

    vc.rotate_cam(rotation);

  }

  return 0;
}
