#include <opencv/cv.h>
#include <opencv/cvaux.h>
#include <opencv/highgui.h>
#include <opencv/ml.h>

#include <iostream>

#define PP_NOSE 1
#define PP_EYE_L 2
#define PP_EYE_R 3
#define PP_MOUTH 4

using namespace cv;
class FaceNormalizer{

  public:
    FaceNormalizer();
    ~FaceNormalizer();


    void normalizeFaces(std::vector<cv::Mat>& head_color,
                              std::vector<cv::Mat>& head_depth,
                              std::vector<std::vector<cv::Rect> >& face_rect);
    bool normalizeFace(cv::Mat & img);
    void getDepthInRect(cv::Mat& depth_map,cv::Rect& rect,float& depth);
    void calcM(cv::Vec3f& eye_l,cv::Vec3f & eye_r,cv::Vec3f & mouth);
    void resetNormFeatures();
    //void showFeatures(cv::Mat img,cv::Rect& nose_det,cv::Rect& mouth_det,cv::Rect& eye_l,cv::Rect& eye_r);
    void showFeatures(cv::Mat& img,cv::Vec3f& nose,cv::Vec3f& mouth,cv::Vec3f& eye_l,cv::Vec3f& eye_r);
    void showFeatures(cv::Mat& img);
    //bool getCoords(cv::Mat& img_color,cv::Mat& img_depth,cv::Vec3f& mouth,cv::Vec3f& nose,cv::Vec3f& eye_l,cv::Vec3f& eye_r);
    bool calcModel(cv::Mat& img_color,cv::Mat& img_depth);
    void transformAffine(cv::Mat& trafo);
    void transformPerspective(cv::Mat& trafo);
    void showImg(cv::Mat& img,std::string window_name);
    bool checkIntersection(cv::Rect& eye_l_rect,cv::Rect& eye_r_rect,cv::Rect& nose_rect,cv::Rect& mouth_rect);

    bool rectIntersect(cv::Rect& r1,cv::Rect& r2);
    bool detect_feature(cv::Mat& img,cv::Vec3f& coords,int code);
    void tf_crop(std::vector<cv::Mat>& head_color,std::vector<std::vector<cv::Rect> >& face_rect);
    void adjustRect(cv::Vec3f& middle,cv::Rect& r,cv::Mat& img);

    bool checkModel();

  protected:
  CvHaarClassifierCascade* nose_cascade_;
  CvMemStorage* nose_storage_;

  CvHaarClassifierCascade* eye_cascade_;
  CvMemStorage*            eye_storage_;

  CvHaarClassifierCascade* eye_l_cascade_;
  CvMemStorage*            eye_l_storage_;

  CvHaarClassifierCascade* eye_r_cascade_;
  CvMemStorage*            eye_r_storage_;

  CvHaarClassifierCascade* mouth_cascade_;
  CvMemStorage*            mouth_storage_;

  cv::Vec3f det_eye_l_;
  cv::Vec3f det_eye_r_;
  cv::Vec3f det_mouth_;
  cv::Vec3f det_nose_;
  cv::Vec3f norm_eye_l_;
  cv::Vec3f norm_eye_r_;
  cv::Vec3f norm_mouth_;
  cv::Vec3f norm_nose_;
  cv::Size  norm_size_;


  bool debug_;
  double scale_;



};
