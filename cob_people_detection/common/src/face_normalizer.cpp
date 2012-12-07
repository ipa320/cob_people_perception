#include<cob_people_detection/face_normalizer.h>
using namespace cv;
FaceNormalizer::FaceNormalizer():scale_(1.0),
                                epoch_ctr(0),
                                debug_path_("/share/goa-tz/people_detection/debug/"),
                                debug_(false)
{
  std::string nose_path="/usr/share/OpenCV-2.3.1/haarcascades/haarcascade_mcs_nose.xml";
  nose_cascade_=(CvHaarClassifierCascade*) cvLoad(nose_path.c_str(),0,0,0);
  nose_storage_=cvCreateMemStorage(0);

  std::string eye_path="/usr/share/OpenCV-2.3.1/haarcascades/haarcascade_eye.xml";
              eye_cascade_=(CvHaarClassifierCascade*) cvLoad(eye_path.c_str(),0,0,0);
              eye_storage_=cvCreateMemStorage(0);

 //std::string eye_l_path="/usr/share/OpenCV-2.3.1/haarcascades/haarcascade_mcs_lefteye.xml";
  std::string eye_l_path="/home/goa-tz/data/haarcascade_lefteye_2splits.xml";
 //std::string eye_l_path="/home/goa-tz/git/care-o-bot/cob_people_perception/cob_people_detection/common/files/haarcascades/haarcascade_eye_tree_eyeglasses.xml";
              eye_l_cascade_=(CvHaarClassifierCascade*) cvLoad(eye_l_path.c_str(),0,0,0);
              eye_l_storage_=cvCreateMemStorage(0);

  //std::string eye_r_path="/usr/share/OpenCV-2.3.1/haarcascades/haarcascade_mcs_righteye.xml";
  std::string eye_r_path="/home/goa-tz/data/haarcascade_righteye_2splits.xml";
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


  f_norm_img_.lefteye.x=0.25*norm_size_.width;
  f_norm_img_.lefteye.y=0.3*norm_size_.height;

  f_norm_img_.righteye.x=0.75*norm_size_.width;
  f_norm_img_.righteye.y=0.3*norm_size_.height;

  f_norm_img_.mouth.x=0.5*norm_size_.width;
  f_norm_img_.mouth.y=0.85*norm_size_.height;

  f_norm_img_.nose.x=0.5*norm_size_.width;
  f_norm_img_.nose.y=0.4*norm_size_.height;


  // reset detections
  f_det_img_.lefteye.x=-1;
  f_det_img_.lefteye.y=-1;

  f_det_img_.righteye.x=-1;
  f_det_img_.righteye.y=-1;

  f_det_img_.mouth.x=-1;
  f_det_img_.mouth.y=-1;


  f_det_img_.nose .x=-1;
  f_det_img_.nose .y=-1;

}


bool FaceNormalizer::normalizeFace( cv::Mat& img,cv::Mat& depth,int & rows)
{
  epoch_ctr++;
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

  // detect features
  if(!features_from_color(img))return false;

   dyn_norm_face();

   if(!features_from_depth(depth)) return false;



  //calculate transformation

   std::vector<cv::Point3f> object_points;
   std::vector<cv::Point2f> img_points;
   f_norm_img_.as_vector(img_points);
   f_det_xyz_.as_vector(object_points);

   cv::Mat trans,rot;


   std::cout<<"size o-points="<<object_points.size();
   std::cout<<"size img-points="<<img_points.size();
    // pass camera model through launch file

   double fx=255;
   double fy=255;
   double cy=img.rows/2;
   double cx=img.cols/2;
   cv::Mat cam_mat=(cv::Mat_<double>(3,3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);

   std::vector<double> dist_coeffs;
   cv::solvePnP(object_points,img_points,cam_mat,dist_coeffs,rot,trans);

   std::vector<cv::Point3f> depth_vec;
   //std::vector<cv::Point2f> img_proj;
   cv::Mat img_proj;
   for(int i=0;i<depth.rows;++i)
   {
     for(int j=0;j<depth.cols;++j)
     {
       depth_vec.push_back((cv::Point3f)depth.at<cv::Vec3f>(i,j));
     }
   }

   cv::Mat coeff2;
   coeff2=cv::Mat::zeros(1,8,CV_32F);
   //cv::projectPoints(object_points,rot,trans,cam_mat,coeff2,img_proj);
   cv::projectPoints(depth_vec,rot,trans,cam_mat,coeff2,img_proj);

  //for(int k=0;k<img_proj.size();++k)
  //{
  //cv::Point2f res=img_proj[k]-img_points[k];
  //std::cout<<"res pt="<<res.x<<" "<<res.y<<std::endl;
  //}
   cv::Mat img_proj_rgb;


   cv::Vec3f dummy;
   cv::Vec3f* img_ptr=img.ptr<cv::Vec3f>(0,0);
   int tr,tc;
   for(int r=0;r<img_proj.rows;++r)
      for(int c=0;c<img_proj.cols;++c)
     {
       tr=(int)ceil(img_proj.at<cv::Vec2f>(r,c)[0]);
       tc=(int)ceil(img_proj.at<cv::Vec2f>(r,c)[1]);
       //std::cout<<" tr="<<tr<<std::endl;
       //std::cout<<" tc="<<tc<<std::endl;
       std::cout<<" error at" <<r<<" "<<c<<std::endl;
       if(tr>0 && tc>0)
       {
       img_proj_rgb.at<cv::Vec3i>(tr,tc)=img.at<cv::Vec3i>(r,c);
       }
       //img_proj_rgb.at<cv::Vec3f>(tr,tc)[0]=dummy[0];
       //img_proj_rgb.at<cv::Vec3f>(tr,tc)[1]=dummy[1];
       //img_proj_rgb.at<cv::Vec3f>(tr,tc)[2]=dummy[2];

       //img_proj_rgb.at<cv::Vec3f>(tr,tc)=img.at<cv::Vec3f>(i,j);
   }

   cv::namedWindow("reproj",CV_WINDOW_AUTOSIZE);
   cv::imshow("reproj",img_proj_rgb);
   cv::waitKey(0);

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

  //pick 3D points from pointcloud
  f_det_xyz_.nose=depth.at<cv::Vec3f>(f_det_img_.nose.x,f_det_img_.nose.y);
  f_det_xyz_.mouth=depth.at<cv::Vec3f>(f_det_img_.mouth.x,f_det_img_.mouth.y);
  f_det_xyz_.lefteye=depth.at<cv::Vec3f>(f_det_img_.lefteye.x,f_det_img_.lefteye.y);
  f_det_xyz_.righteye=depth.at<cv::Vec3f>(f_det_img_.righteye.x,f_det_img_.righteye.y);
  if(debug_)
  {
    std::cout<<"Nose: "<<f_det_xyz_.nose.x<<" "<<f_det_xyz_.nose.y<<" "<<f_det_xyz_.nose.z<<std::endl;
    std::cout<<"Mouth: "<<f_det_xyz_.mouth.x<<" "<<f_det_xyz_.mouth.y<<" "<<f_det_xyz_.mouth.z<<std::endl;
    std::cout<<"LEFTEYE: "<<f_det_xyz_.lefteye.x<<" "<<f_det_xyz_.lefteye.y<<" "<<f_det_xyz_.lefteye.z<<std::endl;
    std::cout<<"RIGTHEYE: "<<f_det_xyz_.righteye.x<<" "<<f_det_xyz_.righteye.y<<" "<<f_det_xyz_.righteye.z<<std::endl;
  }
  if(!f_det_xyz_.valid()) return false;


  return true;
}

bool FaceNormalizer::detect_feature(cv::Mat& img,cv::Point2f& coords,int code)
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
    sub_img=sub_img(cvRect(0,0,f_det_img_.nose.x,f_det_img_.nose.y));
    //showImg(sub_img,"eye_l");
    IplImage ipl_img=(IplImage)sub_img;
     seq=cvHaarDetectObjects(&ipl_img,eye_l_cascade_,eye_l_storage_,1.1,1,0,cvSize(5,5));

  }

  if(code==PP_EYE_R)
  {
    offset[0]=((int)f_det_img_.nose.x);
    offset[1]=0;
    cv::Mat sub_img=img.clone();
    sub_img=sub_img(cvRect(f_det_img_.nose.x,0,img.cols-f_det_img_.nose.x-1,f_det_img_.nose.y));
    IplImage ipl_img=(IplImage)sub_img;
     seq=cvHaarDetectObjects(&ipl_img,eye_r_cascade_,eye_r_storage_,1.1,1,0,cvSize(5,5));

  }

  if(code==PP_MOUTH)
  {
    offset[0]=0;
    offset[1]=(int)f_det_img_.nose.y;
    cv::Mat sub_img=img.clone();
    sub_img=sub_img(cvRect(0,f_det_img_.nose.y,img.cols,img.rows-f_det_img_.nose.y-1));
    IplImage ipl_img=(IplImage)sub_img;
     seq=cvHaarDetectObjects(&ipl_img,mouth_cascade_,mouth_storage_,1.3,4,CV_HAAR_DO_CANNY_PRUNING,cvSize(15,15));

  }

    if(seq->total ==0) return false;
    Rect* seq_det=(Rect*)cvGetSeqElem(seq,0);
    coords.x=(float)seq_det->x+seq_det->width/2+offset[0];
    coords.y=(float)seq_det->y+seq_det->height/2+offset[1];

    return true;
}


void FaceNormalizer::dyn_norm_face()
{

  //measured values x

  //eye base
  cv::Vec2f base=(cv::Vec2f)f_det_img_.righteye-(cv::Vec2f)f_det_img_.lefteye;
  cv::normalize(base,base);
  double a=cv::norm((cv::Vec2f)f_det_img_.lefteye,(cv::Vec2f)f_det_img_.righteye,cv::NORM_L2);
  cv::Vec2f dummy=(cv::Vec2f)f_det_img_.lefteye+(a*0.5*base);

  double b =cv::norm(dummy,(cv::Vec2f)f_det_img_.nose,cv::NORM_L2);
  double c=cv::norm((cv::Vec2f)f_det_img_.nose,(cv::Vec2f)f_det_img_.mouth,cv::NORM_L2);

  double s1=a/b;
  double s2=b/c;

  //norm values
  f_norm_img_.nose.y=f_norm_img_.lefteye.y+(f_norm_img_.righteye.x-f_norm_img_.lefteye.x)/s1;
  f_norm_img_.mouth.y=f_norm_img_.nose.y+(f_norm_img_.nose.y-f_norm_img_.lefteye.y)/s2;


  return;
}




void FaceNormalizer::transformPerspective(cv::Mat& trafo)
{

  cv::Point2f src[4],dst[4];
  src[0]=Point2f(f_det_img_.lefteye.x,f_det_img_.lefteye.y);
  src[1]=Point2f(f_det_img_.righteye.x,f_det_img_.righteye.y);
  src[2]=Point2f(f_det_img_.mouth.x,f_det_img_.mouth.y);
  src[3]=Point2f(f_det_img_.nose.x,f_det_img_.nose.y);

  dst[0]=Point2f(f_norm_img_.lefteye.x,f_norm_img_.lefteye.y);
  dst[1]=Point2f(f_norm_img_.righteye.x,f_norm_img_.righteye.y);
  dst[2]=Point2f(f_norm_img_.mouth.x,f_norm_img_.mouth.y);
  dst[3]=Point2f(f_norm_img_.nose.x,f_norm_img_.nose.y);
  trafo=cv::getPerspectiveTransform(src,dst);
}

void FaceNormalizer::get_transform_affine(cv::Mat& trafo)
{
  cv::Point2f src[3],dst[3];

  src[0]=Point2f(f_det_img_.lefteye.x,f_det_img_.lefteye.y);
  src[1]=Point2f(f_det_img_.righteye.x,f_det_img_.righteye.y);
  src[2]=Point2f(f_det_img_.mouth.x,f_det_img_.mouth.y);

  dst[0]=Point2f(f_norm_img_.lefteye.x,f_norm_img_.lefteye.y);
  dst[1]=Point2f(f_norm_img_.righteye.x,f_norm_img_.righteye.y);
  dst[2]=Point2f(f_norm_img_.mouth.x,f_norm_img_.mouth.y);


  trafo = cv::getAffineTransform(src,dst);





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
   cv::circle(img2,cv::Point(f_det_img_.nose.x, f_det_img_.nose.y),5,CV_RGB(255,0,0));
   cv::circle(img2,cv::Point(f_det_img_.mouth.x,f_det_img_.mouth.y),5,CV_RGB(0,255,0));
   cv::circle(img2,cv::Point(f_det_img_.lefteye.x,f_det_img_.lefteye.y),5,CV_RGB(255,255,0));
   cv::circle(img2,cv::Point(f_det_img_.righteye.x,f_det_img_.righteye.y),5,CV_RGB(255,0,255));
   dump_img(img2,"3_features");
}


//int main(int argc, const char *argv[])
//{
//  FaceNormalizer fn;
//  cv::Mat img;
//  //std::string f_path="/home/goa-tz/data/CroppedYale/";
//  std::string f_path="/home/goa-tz/debug/test_imgs/";
//  std::string temp=f_path;
//  temp.append(argv.y);
//  std::cout<<"using image from: "<<temp<<std::endl;
//  img=cv::imread(temp,CV_LOAD_IMAGE_COLOR);
//  cv::cvtColor(img,img,CV_RGB2BGR);
//  int rows=160;
//  fn.normalizeFace(img,rows);
//  return 0;
//}
