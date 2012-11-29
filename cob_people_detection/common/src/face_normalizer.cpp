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
  std::string eye_l_path="/home/goa-tz/data/haarcascade_lefteye_2splits.xml";
              eye_l_cascade_=(CvHaarClassifierCascade*) cvLoad(eye_l_path.c_str(),0,0,0);
              eye_l_storage_=cvCreateMemStorage(0);

  //std::string eye_r_path="/usr/share/OpenCV-2.3.1/haarcascades/haarcascade_mcs_righteye.xml";
  std::string eye_r_path="/home/goa-tz/data/haarcascade_righteye_2splits.xml";
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


  norm_eye_l_[0]=0.3*norm_size_.width;
  norm_eye_l_[1]=0.3*norm_size_.height;
  norm_eye_l_[2]=0;

  norm_eye_r_[0]=0.7*norm_size_.width;
  norm_eye_r_[1]=0.3*norm_size_.height;
  norm_eye_r_[2]=0;

  norm_mouth_[0]=0.5*norm_size_.width;
  norm_mouth_[1]=0.85*norm_size_.height;
  norm_mouth_[2]=0;

  norm_nose_[0]=0.5*norm_size_.width;
  norm_nose_[1]=0.4*norm_size_.height;
  //norm_nose_[1]=0.6*norm_size_.height;
  norm_nose_[2]=0;


  // reset detections
  det_eye_l_[0]=-1;
  det_eye_l_[1]=-1;
  det_eye_l_[2]=0;

  det_eye_r_[0]=-1;
  det_eye_r_[1]=-1;
  det_eye_r_[1]=0;

  det_mouth_[0]=-1;
  det_mouth_[1]=-1;
  det_mouth_[2]=0;


  det_nose_ [0]=-1;
  det_nose_ [1]=-1;
  det_nose_ [2]=0;

}


bool FaceNormalizer::normalizeFace( cv::Mat& img,int & rows)
{
  //norm size ffrom input image
  set_norm_face(rows);

  if(debug_)
  {
    cv::cvtColor(img,img,CV_BGR2RGB);
    dump_img(img,"orig",epoch_ctr);
  }

  // radiometric normalization
  if(!normalize_radiometry(img)) return false;
  dump_img(img,"eq",epoch_ctr);

  //resizing
  cv::resize(img,img,norm_size_,0,0);
  if(debug_)dump_img(img,"resized",epoch_ctr);

  //geometric normalization
  if(!normalize_geometry(img)) return false;
  if(debug_)dump_img(img,"warped",epoch_ctr);


  epoch_ctr++;
  return true;
}

bool FaceNormalizer::normalize_radiometry(cv::Mat& img)
{
  //to be implemented
  //- histogram normalization

  //convert image to HSV
  cv::Mat work_mat;
  std::vector<cv::Mat> channels;

  if(debug_) cv::cvtColor(img,work_mat,CV_RGB2HSV);
  else cv::cvtColor(img,work_mat,CV_BGR2HSV);

  cv::split(work_mat,channels);
  cv::equalizeHist(channels[2],channels[2]);
  cv::merge(channels,work_mat);
  if(debug_){
  cv::cvtColor(work_mat,work_mat,CV_HSV2RGB);
  work_mat.copyTo(img);
  }
  else cv::cvtColor(work_mat,img,CV_HSV2BGR);

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

  //warping
   cv::Mat warped = cv::Mat(img.rows,img.cols,img.type());
   cv::warpAffine(img,warped,trafo,norm_size_ );


  warped.copyTo(img);

  return true;

}

bool FaceNormalizer::features_from_color(cv::Mat& img_color)
{
  if(!detect_feature(img_color,det_nose_,PP_NOSE))
  {
    std::cout<<"no nose"<<std::endl;
     return false;
  }
  if(!detect_feature(img_color,det_eye_l_,PP_EYE_L))
  {
    std::cout<<"no eye_l"<<std::endl;
     return false;
  }
  if(!detect_feature(img_color,det_eye_r_,PP_EYE_R)) 
  {
    std::cout<<"no eye_r"<<std::endl;
     return false;
  }
  detect_feature(img_color,det_mouth_,PP_MOUTH) ;

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
    sub_img=sub_img(cvRect(0,0,det_nose_[0],det_nose_[1]));
    IplImage ipl_img=(IplImage)sub_img;
     seq=cvHaarDetectObjects(&ipl_img,eye_l_cascade_,eye_l_storage_,1.15,3,CV_HAAR_DO_CANNY_PRUNING,cvSize(25,15));

  }

  if(code==PP_EYE_R)
  {
    offset[0]=((int)det_nose_[0]);
    offset[1]=0;
    cv::Mat sub_img=img.clone();
    sub_img=sub_img(cvRect(det_nose_[0],0,img.cols-det_nose_[0]-1,det_nose_[1]));
    IplImage ipl_img=(IplImage)sub_img;
     seq=cvHaarDetectObjects(&ipl_img,eye_r_cascade_,eye_r_storage_,1.25,3,CV_HAAR_DO_CANNY_PRUNING,cvSize(25,15));

  }

  if(code==PP_MOUTH)
  {
    offset[0]=0;
    offset[1]=(int)det_nose_[1];
    cv::Mat sub_img=img.clone();
    sub_img=sub_img(cvRect(0,det_nose_[1],img.cols,img.rows-det_nose_[1]-1));
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



//------------------------------------------------------------------------------------------
void FaceNormalizer::adjustRect(cv::Vec3f& middle,cv::Rect& r,cv::Mat& img)
{


while(1){
  int overlap_x  =middle[0]+floor(r.width/2) -img.cols+1;
  int overlap_y  =middle[1]+floor(r.height/2) -img.rows+1;
  if(overlap_x >0)
  {
    r.width=floor(r.width/2-overlap_x)*2;
    r.height=r.width;
    r.x=floor(middle[0]-r.width/2);
  }
  else if (overlap_y >0)
  {
    r.height=(r.height/2-overlap_y)*2;
    r.width=r.height;
    r.y=floor(middle[0]-r.width/2);
  }
  else if(r.x<0)
    {
      r.x=0;
      r.width=2*middle[0];
      r.height=r.width;
    }

  else if(r.y<0)
    {
      r.y=0;
      r.height=2*middle[0];
      r.width=r.height;

    }
  else break;
}

}



void FaceNormalizer::tf_crop(std::vector<cv::Mat>& head_color,
                              std::vector<std::vector<cv::Rect> >& face_rect)
{
  for(int i=0;i<head_color.size();i++)
  {
    if(face_rect[i].size()==0)continue;
    {
    cv::Vec3f nose;
    bool feat_detected=detect_feature(head_color[i],nose,PP_NOSE);
      if(feat_detected==true)
      {

      //int norm_nose_x = ceil(face_rect[i][0].x+face_rect[i][0].width/2);
      //int norm_nose_y = ceil(face_rect[i][0].y+(norm_nose_[1]/norm_size_.height)*face_rect[i][0].height);

     // face_rect[i][0].x=floor(nose[0]-face_rect[i][0].width/2);
     // face_rect[i][0].y=floor(nose[1]-face_rect[i][0].height/2);

      //adjustRect(nose,face_rect[i][0],head_color[i]);
      
      //std::cout<<"head color size"<<head_color[i].rows <<" "<< head_color[i].cols<<std::endl;
      //std::cout<<"rect x"<<face_rect[i][0].x<<std::endl;
      //std::cout<<"rect y"<<face_rect[i][0].y<<std::endl;
      //std::cout<<"rect width"<<face_rect[i][0].width<<std::endl;
      //std::cout<<"rect height"<<face_rect[i][0].height<<std::endl;
      
      cv::Mat dummy;
      head_color[i].copyTo(dummy);
      dummy=dummy(face_rect[i][0]);
      resize(dummy,dummy,norm_size_);
      showImg(dummy,"crop");


      }
    }
  }

}



void FaceNormalizer::normalizeFaces(std::vector<cv::Mat>& head_color,
                              std::vector<cv::Mat>& head_depth,
                              std::vector<std::vector<cv::Rect> >& face_rect)
{

  //tf_crop(head_color,face_rect);
  if(head_color.size()==0) return;


 resetNormFeatures();
 for(int face=0;face<head_color.size();++face)
 {

  if(face_rect[face].size()==0) return;

  cv::Mat color_crop=head_color[face].clone();
  cv::Mat depth_crop=head_depth[face].clone();

   color_crop=color_crop(face_rect[face][0]);
   depth_crop=depth_crop(face_rect[face][0]);

   cvtColor(color_crop,color_crop,CV_BGR2RGB);
   cv::resize(color_crop,color_crop,norm_size_,0,0);
   cv::resize(depth_crop,depth_crop,norm_size_,0,0);
   bool coord=calcModel(color_crop,depth_crop);


   if(coord==false)
   {
   return;
   }

   //cv::Mat warped = cv::Mat(head_color[face].rows,head_color[face].cols,head_color[face].type());
   cv::Mat warped = cv::Mat(color_crop.rows,color_crop.cols,color_crop.type());

       cv::Mat trafo(2,3,CV_32FC1);
       get_transform_affine(trafo);
       cv::warpAffine(color_crop,warped,trafo,warped.size() );
       //cv::warpAffine(head_color[face],warped,trafo,warped.size() );

      //cv::Mat trafo(3,3,CV_32FC1);
      // transformPerspective(trafo);
      // cv::warpPerspective(head_color[face],warped,trafo,warped.size());

       warped.copyTo(color_crop);
       showImg(color_crop,"warped");
      //TODO: INPUIT IMAGE IS NOT MODIFIED
       //if(debug_)showImg(warped,"warped");
 }
}

void FaceNormalizer::transformPerspective(cv::Mat& trafo)
{

  cv::Point2f src[4],dst[4];
  src[0]=Point2f(det_eye_l_[0],det_eye_l_[1]);
  src[1]=Point2f(det_eye_r_[0],det_eye_r_[1]);
  src[2]=Point2f(det_mouth_[0],det_mouth_[1]);
  src[3]=Point2f(det_nose_[0],det_nose_[1]);

  dst[0]=Point2f(norm_eye_l_[0],norm_eye_l_[1]);
  dst[1]=Point2f(norm_eye_r_[0],norm_eye_r_[1]);
  dst[2]=Point2f(norm_mouth_[0],norm_mouth_[1]);
  dst[3]=Point2f(norm_nose_[0],norm_nose_[1]);
  trafo=cv::getPerspectiveTransform(src,dst);
}

void FaceNormalizer::get_transform_affine(cv::Mat& trafo)
{
  std::cout<<"norm mouth"<<norm_mouth_[0]<<" "<<norm_mouth_[1]<<" "<<norm_mouth_[2]<<std::endl;
  std::cout<<"norm eye_l"<<norm_eye_l_[0]<<" "<<norm_eye_l_[1]<<" "<<norm_eye_l_[2]<<std::endl;
  std::cout<<"norm eye_r"<<norm_eye_r_[0]<<" "<<norm_eye_r_[1]<<" "<<norm_eye_r_[2]<<std::endl;
  std::cout<<"norm nose"<<norm_nose_[0]<<" "<<norm_nose_[1]<<" "<<norm_nose_[2]<<std::endl;
  cv::Point2f src[3],dst[3];

  src[0]=Point2f(det_eye_l_[0],det_eye_l_[1]);
  src[1]=Point2f(det_eye_r_[0],det_eye_r_[1]);
  //src[2]=Point2f(det_mouth_[0],det_mouth_[1]);
  src[2]=Point2f(det_nose_[0],det_nose_[1]);

  dst[0]=Point2f(norm_eye_l_[0],norm_eye_l_[1]);
  dst[1]=Point2f(norm_eye_r_[0],norm_eye_r_[1]);
  //dst[2]=Point2f(norm_mouth_[0],norm_mouth_[1]);
  dst[2]=Point2f(norm_nose_[0],norm_nose_[1]);


  trafo = cv::getAffineTransform(src,dst);





}









bool FaceNormalizer::calcModel(cv::Mat& img_color,cv::Mat& img_depth)
{

  cv::Vec3f nose_coord,eye_l_coord,eye_r_coord,mouth_coord;
  if (detect_feature(img_color,nose_coord,PP_NOSE))  det_nose_ =nose_coord;
  else return false;
  if(detect_feature(img_color,eye_l_coord,PP_EYE_L)) det_eye_l_=eye_l_coord;
  else return false;
  if(detect_feature(img_color,eye_r_coord,PP_EYE_R)) det_eye_r_=eye_r_coord;
  else return false;
  if(detect_feature(img_color,mouth_coord,PP_MOUTH)) det_mouth_=mouth_coord;
  else return false;

  if(checkModel()) return true;
  else{
    std::cout<<"Model inconsistent\n";
    return false;}
}


void FaceNormalizer::getDepthInRect(cv::Mat& depth_map,cv::Rect& rect,float& depth)
{

  int middle_x=(int)rect.x+rect.width*0.5;
  int middle_y=(int)rect.y+rect.height*0.5;


  cv::Vec3f depth_vec=depth_map.at<cv::Vec3f>(middle_x,
        middle_y);
  depth=depth_vec[2];


}


void FaceNormalizer::calcM(cv::Vec3f& eye_l,cv::Vec3f& eye_r,cv::Vec3f& mouth){
 // distance eye to eye
 cv::Vec3f eye_to_eye=eye_r -eye_l;
 double B=cv::norm(eye_to_eye);
 normalize(eye_to_eye,eye_to_eye);
 cv::Vec3f eye_l_to_mouth= mouth-eye_l;
 normalize(eye_l_to_mouth,eye_l_to_mouth);
 cv::Vec3f  L=eye_l+fabs(eye_to_eye.dot(eye_l_to_mouth))*eye_to_eye;
 double M=norm(L,mouth);

 scale_=B/cv::norm(norm_eye_l_,norm_eye_r_);
 norm_mouth_[1]+=M/scale_;


 std::cout<<"dot"<<eye_to_eye.dot(eye_l_to_mouth)<<std::endl;
 std::cout<<"B="<<B<<std::endl;
 std::cout<<"M="<<M<<std::endl;
 std::cout<<"scale="<<scale_<<std::endl;
 std::cout<<norm_mouth_[1]<<std::endl;


}

void FaceNormalizer::resetNormFeatures(){

  scale_=1.0;
  norm_eye_l_= cv::Vec3f (50,60,0);
  norm_eye_r_= cv::Vec3f (110,60,0);
  norm_mouth_= cv::Vec3f (80,140,0);
  norm_nose_=  cv::Vec3f (80,100,0);
  det_eye_l_=  cv::Vec3f (0,0,0);
  det_eye_r_=  cv::Vec3f (0,0,0);
  det_mouth_=  cv::Vec3f (0,0,0);
  det_nose_=   cv::Vec3f (0,0,0);

}

void FaceNormalizer::showFeatures(cv::Mat& img)
{

  cv::Mat img2;
  img.copyTo(img2);
  IplImage ipl_img=(IplImage)img2;
   cv::circle(img2,cv::Point(det_nose_[0], det_nose_[1]),5,CV_RGB(255,0,0));
   cv::circle(img2,cv::Point(det_mouth_[0],det_mouth_[1]),5,CV_RGB(0,255,0));
   cv::circle(img2,cv::Point(det_eye_l_[0],det_eye_l_[1]),5,CV_RGB(255,255,0));
   cv::circle(img2,cv::Point(det_eye_r_[0],det_eye_r_[1]),5,CV_RGB(255,0,255));
    showImg(img2,"features");
}
void FaceNormalizer::showFeatures(cv::Mat& img,cv::Vec3f& nose,cv::Vec3f& mouth,cv::Vec3f& eye_l,cv::Vec3f& eye_r)
{

  cv::Mat img2;
  img.copyTo(img2);
  IplImage ipl_img=(IplImage)img2;
   cv::circle(img2,cv::Point(nose[0],nose[1]),5,CV_RGB(255,0,0));
   cv::circle(img2,cv::Point(mouth[0],mouth[1]),5,CV_RGB(0,255,0));
   cv::circle(img2,cv::Point(eye_l[0],eye_l[1]),5,CV_RGB(255,255,0));
   cv::circle(img2,cv::Point(eye_r[0],eye_r[1]),5,CV_RGB(255,0,255));
    showImg(img2,"features");
}

//void FaceNormalizer::showFeatures(cv::Mat img,cv::Rect& nose_det,cv::Rect& mouth_det,cv::Rect& eye_l,cv::Rect& eye_r)
//{
//
//  cv::Mat img2;
//  img.copyTo(img2);
//  IplImage ipl_img=(IplImage)img2;
//    cv
//    cvRectangle(&ipl_img,cvPoint(nose[0],nose[1]),
//                       cvPoint(nose.x+nose.width,nose.y+nose.height),
//                       CV_RGB(255,0,0),1,8,0);
//
//    cvRectangle(&ipl_img,cvPoint(mouth_det.x,mouth_det.y),
//                       cvPoint(mouth_det.x+mouth_det.width,mouth_det.y+mouth_det.height),
//                       CV_RGB(0,0,255),1,8,0);
//
//    cvRectangle(&ipl_img,cvPoint(eye_l.x,eye_l.y),
//                       cvPoint(eye_l.x+eye_l.width,eye_l.y+eye_l.height),
//                       CV_RGB(0,255,0),1,8,0);
//    cvRectangle(&ipl_img,cvPoint(eye_r.x,eye_r.y),
//                        cvPoint(eye_r.x+eye_r.width,eye_r.y+eye_r.height),
//                       CV_RGB(0,255,255),1,8,0);
//    showImg(img2,"features");
//}

void FaceNormalizer:: showImg(cv::Mat& img,std::string window_name){
  cv::namedWindow(window_name,CV_WINDOW_AUTOSIZE);
  cv::imshow(window_name,img);
  cv::waitKey(0);
    }

bool FaceNormalizer::checkModel()
{
//  eye_l------------eye_r
//            |
//            |
//           nose
//            |
//            |
//          mouth
//
return true;

//TODO: Always return false...
  if(det_nose_[1]<det_eye_l_[1])   return false;
  else if(det_nose_[1]<det_eye_r_[1])   return false;
  else if(det_mouth_[1]<det_eye_l_[1])  return false;
  else if(det_mouth_[1]<det_eye_r_[1])  return false;
  else if(det_mouth_[1]<det_nose_[1])   return false;

  else    return true;


}

bool FaceNormalizer::checkIntersection(cv::Rect& eye_l_rect,cv::Rect& eye_r_rect,cv::Rect& nose_rect,cv::Rect& mouth_rect)
{
  bool intersected=false;
  if (intersected==rectIntersect(eye_l_rect,eye_r_rect) ==true) return true;
  if (intersected==rectIntersect(eye_l_rect,nose_rect)  ==true) return true;
  if (intersected==rectIntersect(eye_l_rect,mouth_rect) ==true) return true;
  if (intersected==rectIntersect(eye_r_rect,mouth_rect) ==true) return true;
  if (intersected==rectIntersect(eye_r_rect,nose_rect)  ==true) return true;
  if (intersected==rectIntersect(mouth_rect,nose_rect)  ==true) return true;

}

bool FaceNormalizer::rectIntersect(cv::Rect& r1,cv::Rect& r2)
{
    if(r1.x <r2.x && r1.x+r1.width>r2.x && r1.y <r2.y+r2.height  && r1.y+r1.height > r2.y) return true;
    return false;



}




void FaceNormalizer::dump_img(cv::Mat& data,std::string name,int& epoch){
  std::string filename =debug_path_;
  filename.append(boost::lexical_cast<std::string>(epoch));
  filename.append("_");
  filename.append(name);
  filename.append(".jpg");

  cv::imwrite(filename,data);
}


void FaceNormalizer::dyn_norm_face()
{

  //measured values x

  //eye base
  cv::Vec3f base=det_eye_r_-det_eye_l_;
  cv::normalize(base,base);
  double a=cv::norm(det_eye_l_,det_eye_r_,cv::NORM_L2);
  cv::Vec3f dummy=det_eye_l_+(a*0.5*base);

  double b =cv::norm(dummy,det_nose_,cv::NORM_L2);
  double c=cv::norm(det_nose_,det_mouth_,cv::NORM_L2);

  double s1=a/b;
  double s2=b/c;

  //norm values
  norm_nose_[1]=norm_eye_l_[1]+(norm_eye_r_[0]-norm_eye_l_[0])/s1;
  norm_mouth_[1]=norm_nose_[1]+(norm_nose_[1]-norm_eye_l_[1])/s2;


  return;
}

int main(int argc, const char *argv[])
{
  FaceNormalizer fn;
  cv::Mat img;
  std::string f_path="/home/goa-tz/debug/test_imgs/";
  std::string temp=f_path;
  temp.append(argv[1]);
  std::cout<<temp<<std::endl;
  img=cv::imread(temp,CV_LOAD_IMAGE_COLOR);
  cv::cvtColor(img,img,CV_RGB2BGR);
  int rows=160;
  fn.normalizeFace(img,rows);
  return 0;
}
