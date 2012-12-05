#include <opencv/cv.h>
#include <opencv/cvaux.h>
#include <opencv/highgui.h>
#include <opencv/ml.h>

#include <iostream>

#include <boost/lexical_cast.hpp>
#define PP_NOSE 1
#define PP_EYE_L 2
#define PP_EYE_R 3
#define PP_MOUTH 4

using namespace cv;

struct FaceFeatures{
                    cv::Vec3f lefteye;
                    cv::Vec3f righteye;
                    cv::Vec3f nose;
                    cv::Vec3f mouth;
                  };
class FaceNormalizer{

  public:
    FaceNormalizer();
    ~FaceNormalizer();


    bool normalizeFace( cv::Mat & img,int& rows);
    void set_norm_face(int& size);
    bool normalize_geometry(cv::Mat& img);
    void get_transform_affine(cv::Mat& trafo);
    bool features_from_color(cv::Mat& img);
    bool detect_feature(cv::Mat& img,cv::Vec3f& coords,int code);
    void dyn_norm_face();
    void resetNormFeatures();
    void transformPerspective(cv::Mat& trafo);

    bool normalizeFace( cv::Mat & img,cv::Mat& depth,int& rows);
    bool normalize_geometry_depth(cv::Mat& img,cv::Mat& depth);
    bool features_from_depth(cv::Mat& depth);

    // Methods for geometric normalization
    bool normalize_radiometry(cv::Mat& img);
    void extractVChannel(cv::Mat& img,cv::Mat& V);
    void subVChannel(cv::Mat& img,cv::Mat& V);
    void eqHist(cv::Mat& img);
    void dct(cv::Mat& img);

    // Debug/Output methods
    void dump_features(cv::Mat& img);
    void dump_img(cv::Mat& data,std::string name);
    void showImg(cv::Mat& img,std::string window_name);
//---------------------------------------------------------

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


  FaceFeatures f_det_img_, f_norm_img_,f_det_xyz_;
  cv::Size  norm_size_;


  bool debug_;
  double scale_;



  int epoch_ctr;
  std::string debug_path_;
};
