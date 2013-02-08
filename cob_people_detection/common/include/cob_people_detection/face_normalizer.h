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
                      std::cout<<lefteye.x<<" , "<<lefteye.y<<" , "<<lefteye.z<<std::endl;
                      std::cout<<"righteye:\n";
                      std::cout<<righteye.x<<" , "<<righteye.y<<" , "<<righteye.z<<std::endl;
                      std::cout<<"nose:\n";
                      std::cout<<nose.x<<" , "<<nose.y<<" , "<<nose.z<<std::endl;
                      std::cout<<"mouth:\n";
                      std::cout<<mouth.x<<" , "<<mouth.y<<" , "<<mouth.y<<std::endl;
                      std::cout<<"--------------------"<<std::endl;
                    }
};


};

class FaceNormalizer{

  public:
    FaceNormalizer();
    ~FaceNormalizer();

      enum TRAFO
      {
        AFFINE,
        PERSPECTIVE,
      };


    bool normalizeFace( cv::Mat & img,cv::Size& norm_size);
    bool normalizeFace( cv::Mat & img,cv::Mat& depth,cv::Size& norm_size,cv::Vec2f& offset,cv::Mat& depth_res);
    void set_norm_face(cv::Size& input_size);
    bool normalize_geometry(cv::Mat& img,TRAFO model);
    void get_transform_affine(cv::Mat& trafo);
    void get_transform_perspective(cv::Mat& trafo);
    bool features_from_color(cv::Mat& img);
    bool detect_feature(cv::Mat& img,cv::Point2f& coords,FACE::TYPE type);
    void dyn_norm_face();
    void ident_face();

    bool normalize_geometry_depth(cv::Mat& img,cv::Mat& depth);
    bool features_from_depth(cv::Mat& depth);
    //void despeckle(cv::Mat& src,cv::Mat& dst);
    void processDM(cv::Mat& dm,cv::Mat& dm_res);

    // Methods for radiometric normalization
    bool normalize_radiometry(cv::Mat& img);
    void extractVChannel(cv::Mat& img,cv::Mat& V);
    void subVChannel(cv::Mat& img,cv::Mat& V);
    void eqHist(cv::Mat& img);
    void dct(cv::Mat& img);
    void logAbout(cv::Mat& img);

    // Debug/Output methods
    void dump_features(cv::Mat& img);
    void dump_img(cv::Mat& data,std::string name);
    void showImg(cv::Mat& img,std::string window_name);
    bool save_scene(cv::Mat& depth,cv::Mat& color,cv::Vec2f& offset,std::string path);
    bool read_scene(cv::Mat& depth,cv::Mat& color,cv::Vec2f& offset,std::string path);
    bool captureScene( cv::Mat& img,cv::Mat& depth,cv::Vec2f& offset);
    bool get_feature_correspondences( cv::Mat& img,  cv::Mat& depth, std::vector<cv::Point2f>& img_pts,std::vector<cv::Point3f>& obj_pts);
    bool face_coordinate_system(FACE::FaceFeatures<cv::Point3f>& feat_world,FACE::FaceFeatures<cv::Point3f>& feat_local);

template <class T>
void despeckle(cv::Mat& src,cv::Mat& dst)
{




  if(src.channels()==1)
  {
    T* lptr=src.ptr<T>(1,0);
    T* rptr=src.ptr<T>(1,2);
    T* mptr=src.ptr<T>(1,1);
    T* uptr=src.ptr<T>(0,1);
    T* dptr=src.ptr<T>(2,1);

    int normalizer=4;

    for(int px=2*src.cols+2;px<(dst.rows*src.cols);++px)
    {
      if(*mptr==0)
      {
      normalizer=4;
      if(*lptr==0) normalizer-=1;
      if(*rptr==0) normalizer-=1;
      if(*uptr==0) normalizer-=1;
      if(*dptr==0) normalizer-=1;
      if(normalizer>0)*mptr=(*lptr + *rptr + *uptr +*dptr)/normalizer;
      }
      ++lptr;
      ++rptr;
      ++mptr;
      ++uptr;
      ++dptr;
    }
  }

  if(src.channels()==3)
  {
    cv::Vec<T,3>* lptr=src.ptr<cv::Vec<T,3> >(1,0);
    cv::Vec<T,3>* rptr=src.ptr<cv::Vec<T,3> >(1,2);
    cv::Vec<T,3>* mptr=src.ptr<cv::Vec<T,3> >(1,1);
    cv::Vec<T,3>* uptr=src.ptr<cv::Vec<T,3> >(0,1);
    cv::Vec<T,3>* dptr=src.ptr<cv::Vec<T,3> >(2,1);

    int normalizer=4;
    for(int px=2*src.cols+2;px<(dst.rows*src.cols);++px)
    {
      if((*mptr)[0]==0)
      {
      normalizer=4;
      if((*lptr)[0]==0) normalizer-=1;
      if((*rptr)[0]==0) normalizer-=1;
      if((*uptr)[0]==0) normalizer-=1;
      if((*dptr)[0]==0) normalizer-=1;
      if(normalizer>0)cv::divide((*lptr + *rptr + *uptr +*dptr),normalizer,*mptr);
      }
      ++lptr;
      ++rptr;
      ++mptr;
      ++uptr;
      ++dptr;
    }
  }
  cv::medianBlur(dst,dst,3);
}
//---------------------------------------------------------

  protected:
  int epoch_ctr;
  bool debug_;
  bool record_scene;
  std::string debug_path_;

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
  cv::Size  input_size_;

  VirtualCamera kinect;
};
