#include <opencv/cv.h>
#include <opencv/cvaux.h>
#include <opencv/highgui.h>
#include <opencv/ml.h>

#include <iostream>


#include <cob_people_detection/virtual_camera.h>
#include <boost/lexical_cast.hpp>


using namespace cv;

namespace FACE{
                      enum TYPE
                      {
                        LEFTEYE,
                        RIGHTEYE,
                        NOSE,
                        MOUTH,
                      };


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

                    void scale(double scale)
                    {
                      lefteye*=scale;
                      righteye*=scale;
                      nose*=scale;
                      mouth*=scale;
                    }

                    std::vector<T> as_vector()
                    {
                     std::vector<T> vec;
                     vec.push_back(lefteye);
                     vec.push_back(righteye);
                     vec.push_back(nose);
                     vec.push_back(mouth);
                     return vec;
                    };
                    bool valid()
                    {
                      if(std::isnan(lefteye.x)|| std::isnan(lefteye.y)) return false;
                      if(std::isnan(righteye.x)|| std::isnan(righteye.y)) return false;
                      if(std::isnan(nose.x)|| std::isnan(nose.y)) return false;
                      if(std::isnan(mouth.x)|| std::isnan(mouth.y)) return false;
                      else return true;
                    }
                    void print()
                    {
                      std::cout<<"--------------------"<<std::endl;
                      std::cout<<"lefteye:\n";
                      std::cout<<lefteye.x<<" , "<<lefteye.y<<std::endl;
                      std::cout<<"righteye:\n";
                      std::cout<<righteye.x<<" , "<<righteye.y<<std::endl;
                      std::cout<<"nose:\n";
                      std::cout<<nose.x<<" , "<<nose.y<<std::endl;
                      std::cout<<"mouth:\n";
                      std::cout<<mouth.x<<" , "<<mouth.y<<std::endl;
                      std::cout<<"--------------------"<<std::endl;
                    }
};


};

class FaceNormalizer{

  public:
    FaceNormalizer();
    ~FaceNormalizer();

      enum MOD
      {
        AFFINE,
        PERSPECTIVE,
      };


    bool normalizeFace( cv::Mat & img,int& rows);
    void set_norm_face(int& rows,int& cols);
    bool normalize_geometry(cv::Mat& img,MOD model);
    void get_transform_affine(cv::Mat& trafo);
    void get_transform_perspective(cv::Mat& trafo);
    bool features_from_color(cv::Mat& img);
    bool detect_feature(cv::Mat& img,cv::Point2f& coords,FACE::TYPE type);
    void dyn_norm_face();
    void ident_face();
    void resetNormFeatures();

    bool normalizeFace( cv::Mat & img,cv::Mat& depth,int& rows,cv::Vec2f& offset);
    bool normalize_geometry_depth(cv::Mat& img,cv::Mat& depth);
    bool features_from_depth(cv::Mat& depth);
    void kin2xyz(cv::Point3f& vec);
    void despeckle(cv::Mat& src,cv::Mat& dst);

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
    bool captureScene( cv::Mat& img,cv::Mat& depth,cv::Vec2f& offset);
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


  FACE::FaceFeatures<cv::Point2f> f_det_img_, f_norm_img_;
  FACE::FaceFeatures<cv::Point3f> f_det_xyz_;

  cv::Size  norm_size_;
  cv::Size  detect_size_;


  bool debug_;
  double scale_;



  int epoch_ctr;
  std::string debug_path_;

  VirtualCamera kinect;
};
