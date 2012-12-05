#include<cob_people_detection/face_normalizer.h>
using namespace cv;
FaceNormalizer::FaceNormalizer():scale_(1.0),
                                epoch_ctr(0),
                                debug_path_("/home/goa-tz/debug/"),
                                debug_(true)
{

  std::string nose_path="/usr/share/OpenCV-2.3.1/haarcascades/haarcascade_mcs_nose.xml";
  nose_cascade_=(CvHaarClassifierCascade*) cvLoad(nose_path.c_str(),0,0,0);
  nose_storage_=cvCreateMemStorage(0);

  std::string eye_path="/usr/share/OpenCV-2.3.1/haarcascades/haarcascade_eye.xml";
              eye_cascade_=(CvHaarClassifierCascade*) cvLoad(eye_path.c_str(),0,0,0);
              eye_storage_=cvCreateMemStorage(0);

 //std::string eye_l_path="/usr/share/OpenCV-2.3.1/haarcascades/haarcascade_mcs_lefteye.xml";
  std::string eye_l_path="/home/goa-tz/data/haarcascade_lefteye2splits.xml";
 //std::string eye_l_path="/home/goa-tz/git/care-o-bot/cob_people_perception/cob_people_detection/common/files/haarcascades/haarcascade_eye_tree_eyeglasses.xml";
              eye_l_cascade_=(CvHaarClassifierCascade*) cvLoad(eye_l_path.c_str(),0,0,0);
              eye_l_storage_=cvCreateMemStorage(0);

  //std::string eye_r_path="/usr/share/OpenCV-2.3.1/haarcascades/haarcascade_mcs_righteye.xml";
  std::string eye_r_path="/home/goa-tz/data/haarcascade_righteye2splits.xml";
              eye_r_cascade_=(CvHaarClassifierCascade*) cvLoad(eye_l_path.c_str(),0,0,0);
              eye_r_cascade_=(CvHaarClassifierCascade*) cvLoad(eye_r_path.c_str(),0,0,0);
              eye_r_storage_=cvCreateMemStorage(0);

  std::string mouth_path="/usr/share/OpenCV-2.3.1/haarcascades/haarcascade_mcs_mouth.xml";
              mouth_cascade_=(CvHaarClassifierCascade*) cvLoad(mouth_path.c_str(),0,0,0);
              mouth_storage_=cvCreateMemStorage(0);
}


FaceNormalizer::~FaceNormalizer(){};

void FaceNormalizer::set_norm_face(int& size)

{
  norm_size_.height=size;
  norm_size_.width=size;


  f_norm_img_.lefteye[0]=0.25*norm_size_.width;
  f_norm_img_.lefteye[1]=0.3*norm_size_.height;
  f_norm_img_.lefteye[2]=0;

  f_norm_img_.righteye[0]=0.75*norm_size_.width;
  f_norm_img_.righteye[1]=0.3*norm_size_.height;
  f_norm_img_.righteye[2]=0;

  f_norm_img_.mouth[0]=0.5*norm_size_.width;
  f_norm_img_.mouth[1]=0.85*norm_size_.height;
  f_norm_img_.mouth[2]=0;

  f_norm_img_.nose[0]=0.5*norm_size_.width;
  f_norm_img_.nose[1]=0.4*norm_size_.height;
  //f_norm_img_nose[1]=0.6*norm_size_.height;
  f_norm_img_.nose[2]=0;


  // reset detections
  f_det_img_.lefteye[0]=-1;
  f_det_img_.lefteye[1]=-1;
  f_det_img_.lefteye[2]=0;

  f_det_img_.righteye[0]=-1;
  f_det_img_.righteye[1]=-1;
  f_det_img_.righteye[1]=0;

  f_det_img_.mouth[0]=-1;
  f_det_img_.mouth[1]=-1;
  f_det_img_.mouth[2]=0;


  f_det_img_.nose [0]=-1;
  f_det_img_.nose [1]=-1;
  f_det_img_.nose [2]=0;

}


bool FaceNormalizer::normalizeFace( cv::Mat& img,cv::Mat& depth,int & rows)
{
  //norm size ffrom input image
  set_norm_face(rows);

  if(debug_)
  {
    cv::cvtColor(img,img,CV_BGR2RGB);
    dump_img(img,"0_original");
  }

  //geometric normalization
  if(!normalize_geometry_depth(img,depth)) return false;
  if(debug_)dump_img(img,"4_geometry");
  //resizing
  cv::resize(img,img,norm_size_,0,0);
  if(debug_)dump_img(img,"1_resized");

  // radiometric normalization
  if(!normalize_radiometry(img)) return false;
  dump_img(img,"2_radiometry");




  epoch_ctr++;
  return true;
}
bool FaceNormalizer::normalizeFace( cv::Mat& img,int & rows)
{
  //norm size ffrom input image
  set_norm_face(rows);

  if(debug_)
  {
    cv::cvtColor(img,img,CV_BGR2RGB);
    dump_img(img,"0_original");
  }

  //resizing
  cv::resize(img,img,norm_size_,0,0);
  if(debug_)dump_img(img,"1_resized");

  // radiometric normalization
  if(!normalize_radiometry(img)) return false;
  dump_img(img,"2_radiometry");


  //geometric normalization
  if(!normalize_geometry(img)) return false;
  if(debug_)dump_img(img,"4_geometry");


  epoch_ctr++;
  return true;
}

bool FaceNormalizer::normalize_radiometry(cv::Mat& img)
{
  cv::Mat v_channel;
  extractVChannel(img,v_channel);

  dct(v_channel);
  subVChannel(img,v_channel);

  return true;
}

void FaceNormalizer::extractVChannel( cv::Mat& img,cv::Mat& V)
{
  if(debug_)cv::cvtColor(img,img,CV_RGB2HSV);
  else cv::cvtColor(img,img,CV_BGR2HSV);

  std::vector<cv::Mat> channels;
  cv::split(img,channels);
  channels[2].copyTo(V);

  if(debug_)cv::cvtColor(img,img,CV_HSV2RGB);
  else cv::cvtColor(img,img,CV_HSV2BGR);

  return;

}

void FaceNormalizer::subVChannel(cv::Mat& img,cv::Mat& V)
{
  
  if(debug_)cv::cvtColor(img,img,CV_RGB2HSV);
  else cv::cvtColor(img,img,CV_BGR2HSV);

  std::vector<cv::Mat> channels;
  cv::split(img,channels);
  channels[2]=V;
  cv::merge(channels,img);

  if(debug_)cv::cvtColor(img,img,CV_HSV2RGB);
  else cv::cvtColor(img,img,CV_HSV2BGR);

  return;

}


void FaceNormalizer::eqHist(cv::Mat& img)
{

  cv::equalizeHist(img,img);

}

void FaceNormalizer::dct(cv::Mat& img)
{
// Dct conversion on logarithmic image

  img.convertTo(img,CV_32FC1);
  cv::Scalar mu=cv::mean(img);
  double C_00=log(mu.val[0])*sqrt(img.cols*img.rows);

  cv::log(img,img);
  cv::dct(img,img);

  //---------------------------------------
  if(debug_)std::cout<<"C_00 a priori="<<img.at<float>(0,0)<<std::endl;
  if(debug_)std::cout<<"C_00 a post="<<C_00<<std::endl;
  img.at<float>(0,0)=C_00;
  img.at<float>(0,1)=0;
  //--------------------------------------

  cv::idct(img,img);
  cv::exp(img,img);
  img.convertTo(img,CV_8UC1);

}




bool FaceNormalizer::normalize_geometry_depth(cv::Mat& img,cv::Mat& depth)
{

  return false;
  // detect features
  if(!features_from_color(img))return false;

   dyn_norm_face();

   if(!features_from_depth(depth)) return false;
  //calculate transformation

  return true;

}
bool FaceNormalizer::normalize_geometry(cv::Mat& img)
{

  // detect features
  if(!features_from_color(img))return false;

   dyn_norm_face();
  //calculate transformation
   cv::Mat trafo(2,3,CV_32FC1);
   get_transform_affine(trafo);

   if(debug_)dump_features(img);

  //warping
   cv::Mat warped = cv::Mat(img.rows,img.cols,img.type());
   cv::warpAffine(img,warped,trafo,norm_size_ );


  warped.copyTo(img);

  return true;

}

bool FaceNormalizer::features_from_color(cv::Mat& img_color)
{
  if(!detect_feature(img_color,f_det_img_.nose,PP_NOSE))
  {
    if(debug_)std::cout<<"no nose"<<std::endl;
     return false;
  }
  if(!detect_feature(img_color,f_det_img_.lefteye,PP_EYE_L))
  {
    if(debug_)std::cout<<"no eye_l"<<std::endl;
     return false;
  }
  if(!detect_feature(img_color,f_det_img_.righteye,PP_EYE_R))
  {
    if(debug_)std::cout<<"no eye_r"<<std::endl;
     return false;
  }
  detect_feature(img_color,f_det_img_.mouth,PP_MOUTH) ;

  return true;
}

bool FaceNormalizer::features_from_depth(cv::Mat& depth)
{


  return true;
}

bool FaceNormalizer::detect_feature(cv::Mat& img,cv::Vec3f& coords,int code)
{

  CvSeq* seq;
  cv::Vec2f offset;

  if(code==PP_NOSE)
  {
    offset<< (0,0);
    IplImage ipl_img=(IplImage)img;
     seq=cvHaarDetectObjects(&ipl_img,nose_cascade_,nose_storage_,1.3,2,CV_HAAR_DO_CANNY_PRUNING,cvSize(15,15));
  }


  if(code==PP_EYE_L)
  {
    offset[0]=0;
    offset[1]=0;
    cv::Mat sub_img=img.clone();
    sub_img=sub_img(cvRect(0,0,f_det_img_.nose[0],f_det_img_.nose[1]));
    //showImg(sub_img,"eye_l");
    IplImage ipl_img=(IplImage)sub_img;
     seq=cvHaarDetectObjects(&ipl_img,eye_l_cascade_,eye_l_storage_,1.1,1,0,cvSize(5,5));

  }

  if(code==PP_EYE_R)
  {
    offset[0]=((int)f_det_img_.nose[0]);
    offset[1]=0;
    cv::Mat sub_img=img.clone();
    sub_img=sub_img(cvRect(f_det_img_.nose[0],0,img.cols-f_det_img_.nose[0]-1,f_det_img_.nose[1]));
    IplImage ipl_img=(IplImage)sub_img;
     seq=cvHaarDetectObjects(&ipl_img,eye_r_cascade_,eye_r_storage_,1.1,1,0,cvSize(5,5));

  }

  if(code==PP_MOUTH)
  {
    offset[0]=0;
    offset[1]=(int)f_det_img_.nose[1];
    cv::Mat sub_img=img.clone();
    sub_img=sub_img(cvRect(0,f_det_img_.nose[1],img.cols,img.rows-f_det_img_.nose[1]-1));
    IplImage ipl_img=(IplImage)sub_img;
     seq=cvHaarDetectObjects(&ipl_img,mouth_cascade_,mouth_storage_,1.3,4,CV_HAAR_DO_CANNY_PRUNING,cvSize(15,15));

  }

    if(seq->total ==0) return false;
    Rect* seq_det=(Rect*)cvGetSeqElem(seq,0);
    coords[0]=(float)seq_det->x+seq_det->width/2+offset[0];
    coords[1]=(float)seq_det->y+seq_det->height/2+offset[1];
    coords[2]=0.0;

    return true;
}


void FaceNormalizer::dyn_norm_face()
{

  //measured values x

  //eye base
  cv::Vec3f base=f_det_img_.righteye-f_det_img_.lefteye;
  cv::normalize(base,base);
  double a=cv::norm(f_det_img_.lefteye,f_det_img_.righteye,cv::NORM_L2);
  cv::Vec3f dummy=f_det_img_.lefteye+(a*0.5*base);

  double b =cv::norm(dummy,f_det_img_.nose,cv::NORM_L2);
  double c=cv::norm(f_det_img_.nose,f_det_img_.mouth,cv::NORM_L2);

  double s1=a/b;
  double s2=b/c;

  //norm values
  f_norm_img_.nose[1]=f_norm_img_.lefteye[1]+(f_norm_img_.righteye[0]-f_norm_img_.lefteye[0])/s1;
  f_norm_img_.mouth[1]=f_norm_img_.nose[1]+(f_norm_img_.nose[1]-f_norm_img_.lefteye[1])/s2;


  return;
}




void FaceNormalizer::transformPerspective(cv::Mat& trafo)
{

  cv::Point2f src[4],dst[4];
  src[0]=Point2f(f_det_img_.lefteye[0],f_det_img_.lefteye[1]);
  src[1]=Point2f(f_det_img_.righteye[0],f_det_img_.righteye[1]);
  src[2]=Point2f(f_det_img_.mouth[0],f_det_img_.mouth[1]);
  src[3]=Point2f(f_det_img_.nose[0],f_det_img_.nose[1]);

  dst[0]=Point2f(f_norm_img_.lefteye[0],f_norm_img_.lefteye[1]);
  dst[1]=Point2f(f_norm_img_.righteye[0],f_norm_img_.righteye[1]);
  dst[2]=Point2f(f_norm_img_.mouth[0],f_norm_img_.mouth[1]);
  dst[3]=Point2f(f_norm_img_.nose[0],f_norm_img_.nose[1]);
  trafo=cv::getPerspectiveTransform(src,dst);
}

void FaceNormalizer::get_transform_affine(cv::Mat& trafo)
{
  cv::Point2f src[3],dst[3];

  src[0]=Point2f(f_det_img_.lefteye[0],f_det_img_.lefteye[1]);
  src[1]=Point2f(f_det_img_.righteye[0],f_det_img_.righteye[1]);
  src[2]=Point2f(f_det_img_.mouth[0],f_det_img_.mouth[1]);

  dst[0]=Point2f(f_norm_img_.lefteye[0],f_norm_img_.lefteye[1]);
  dst[1]=Point2f(f_norm_img_.righteye[0],f_norm_img_.righteye[1]);
  dst[2]=Point2f(f_norm_img_.mouth[0],f_norm_img_.mouth[1]);


  trafo = cv::getAffineTransform(src,dst);





}

void FaceNormalizer::resetNormFeatures(){

  scale_=1.0;
  f_norm_img_.lefteye= cv::Vec3f (50,60,0);
  f_norm_img_.righteye= cv::Vec3f (110,60,0);
  f_norm_img_.mouth= cv::Vec3f (80,140,0);
  f_norm_img_.nose=  cv::Vec3f (80,100,0);

  f_det_img_.lefteye=  cv::Vec3f (0,0,0);
  f_det_img_.righteye=  cv::Vec3f (0,0,0);
  f_det_img_.mouth=  cv::Vec3f (0,0,0);
  f_det_img_.nose=   cv::Vec3f (0,0,0);

}

void FaceNormalizer:: showImg(cv::Mat& img,std::string window_name){
  cv::namedWindow(window_name,CV_WINDOW_AUTOSIZE);
  cv::imshow(window_name,img);
  cv::waitKey(0);
    }

void FaceNormalizer::dump_img(cv::Mat& data,std::string name){
  std::string filename =debug_path_;
  filename.append(boost::lexical_cast<std::string>(epoch_ctr));
  filename.append("_");
  filename.append(name);
  filename.append(".jpg");

  cv::imwrite(filename,data);
}

void FaceNormalizer::dump_features(cv::Mat& img)
{

  cv::Mat img2;
  img.copyTo(img2);
  IplImage ipl_img=(IplImage)img2;
   cv::circle(img2,cv::Point(f_det_img_.nose[0], f_det_img_.nose[1]),5,CV_RGB(255,0,0));
   cv::circle(img2,cv::Point(f_det_img_.mouth[0],f_det_img_.mouth[1]),5,CV_RGB(0,255,0));
   cv::circle(img2,cv::Point(f_det_img_.lefteye[0],f_det_img_.lefteye[1]),5,CV_RGB(255,255,0));
   cv::circle(img2,cv::Point(f_det_img_.righteye[0],f_det_img_.righteye[1]),5,CV_RGB(255,0,255));
   dump_img(img2,"3_features");
}


int main(int argc, const char *argv[])
{
  FaceNormalizer fn;
  cv::Mat img;
  //std::string f_path="/home/goa-tz/data/CroppedYale/";
  std::string f_path="/home/goa-tz/debug/test_imgs/";
  std::string temp=f_path;
  temp.append(argv[1]);
  std::cout<<"using image from: "<<temp<<std::endl;
  img=cv::imread(temp,CV_LOAD_IMAGE_COLOR);
  cv::cvtColor(img,img,CV_RGB2BGR);
  int rows=160;
  fn.normalizeFace(img,rows);
  return 0;
}
