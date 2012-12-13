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

template <class T>
class FaceFeatures{
                    public:
                    T lefteye;
                    T righteye;
                    T nose;
                    T mouth;
                    FaceFeatures<T>(){};
                    ~FaceFeatures<T>(){};

                    void sub_offset(int& ox,int& oy)
                    {
                      this->lefteye.x-=ox;
                      this->lefteye.y-=oy;
                      this->righteye.x-=ox;
                      this->righteye.y-=oy;
                      this->mouth.x-=ox;
                      this->mouth.y-=oy;
                      this->nose.x-=ox;
                      this->nose.y-=oy;
                    }
                    void add_offset(int& ox,int& oy)
                    {
                      this->lefteye.x+=ox;
                      this->lefteye.y+=oy;
                      this->righteye.x+=ox; 
                      this->righteye.y+=oy;
                      this->mouth.x+=ox;
                      this->mouth.y+=oy;
                      this->nose.x+=ox;
                      this->nose.y+=oy;
                    }

                    void as_vector(std::vector<T>& vec)
                    {
                     vec.push_back(lefteye);
                     vec.push_back(righteye);
                     vec.push_back(nose);
                     vec.push_back(mouth);
                    };
                    bool valid()
                    {
                      if(std::isnan(lefteye.x)|| std::isnan(lefteye.y)) return false;
                      if(std::isnan(righteye.x)|| std::isnan(righteye.y)) return false;
                      if(std::isnan(nose.x)|| std::isnan(nose.y)) return false;
                      if(std::isnan(mouth.x)|| std::isnan(mouth.y)) return false;
                      else return true;
                    }
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
    bool detect_feature(cv::Mat& img,cv::Point2f& coords,int code);
    void dyn_norm_face();
    void ident_face();
    void resetNormFeatures();
    void transformPerspective(cv::Mat& trafo);

    bool normalizeFace( cv::Mat & img,cv::Mat& depth,int& rows,cv::Vec2f& offset);
    bool normalize_geometry_depth(cv::Mat& img,cv::Mat& depth);
    bool features_from_depth(cv::Mat& depth);

    // Methods for geometric normalization
    bool normalize_radiometry(cv::Mat& img);
    void extractVChannel(cv::Mat& img,cv::Mat& V);
    void subVChannel(cv::Mat& img,cv::Mat& V);
    void eqHist(cv::Mat& img);
    void dct(cv::Mat& img);

    void calcPnP(cv::Mat& cam_mat,cv::Mat& rot,cv::Mat& trans);
    void resample_direct(cv::Mat& cam_mat,cv::Mat& depth,cv::Mat& img,cv::Mat& res);
    // Debug/Output methods
    void dump_features(cv::Mat& img);
    void dump_img(cv::Mat& data,std::string name);
    void showImg(cv::Mat& img,std::string window_name);
    bool save_scene(cv::Mat& depth,cv::Mat& color,cv::Vec2f& offset,std::string path);
    bool read_scene(cv::Mat& depth,cv::Mat& color,cv::Vec2f& offset,std::string path);
//---------------------------------------------------------

  protected:

    cv::Mat img_,depth_;

    cv::Vec2f offset_;
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


  FaceFeatures<cv::Point2f> f_det_img_, f_norm_img_;
  FaceFeatures<cv::Point3f> f_det_xyz_;
  cv::Size  norm_size_;


  bool debug_;
  double scale_;



  int epoch_ctr;
  std::string debug_path_;
};
