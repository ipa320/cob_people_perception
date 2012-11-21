#include<cob_people_detection/face_normalizer.h>
using namespace cv;
FaceNormalizer::FaceNormalizer():scale_(1.0),
                                norm_eye_l_(50,60,0),
                                norm_eye_r_(110,60,0),
                                norm_mouth_(60,500,0),
                                norm_nose_(80,100),
                                norm_size_(160,160)
{

  std::string nose_path="/usr/share/OpenCV-2.3.1/haarcascades/haarcascade_mcs_nose.xml";
  nose_cascade_=(CvHaarClassifierCascade*) cvLoad(nose_path.c_str(),0,0,0);
  nose_storage_=cvCreateMemStorage(0);

  std::string eye_path="/usr/share/OpenCV-2.3.1/haarcascades/haarcascade_eye.xml";
              eye_cascade_=(CvHaarClassifierCascade*) cvLoad(eye_path.c_str(),0,0,0);
              eye_storage_=cvCreateMemStorage(0);

  std::string eye_l_path="/usr/share/OpenCV-2.3.1/haarcascades/haarcascade_mcs_lefteye.xml";
  //std::string eye_l_path="/home/goa-tz/data/haarcascade_lefteye_2splits.xml";
              eye_l_cascade_=(CvHaarClassifierCascade*) cvLoad(eye_l_path.c_str(),0,0,0);
              eye_l_storage_=cvCreateMemStorage(0);

  std::string eye_r_path="/usr/share/OpenCV-2.3.1/haarcascades/haarcascade_mcs_righteye.xml";
  //std::string eye_r_path="/home/goa-tz/data/haarcascade_righteye_2splits.xml";
              eye_r_cascade_=(CvHaarClassifierCascade*) cvLoad(eye_l_path.c_str(),0,0,0);
              eye_r_cascade_=(CvHaarClassifierCascade*) cvLoad(eye_r_path.c_str(),0,0,0);
              eye_r_storage_=cvCreateMemStorage(0);

  std::string mouth_path="/usr/share/OpenCV-2.3.1/haarcascades/haarcascade_mcs_mouth.xml";
              mouth_cascade_=(CvHaarClassifierCascade*) cvLoad(mouth_path.c_str(),0,0,0);
              mouth_storage_=cvCreateMemStorage(0);
}


FaceNormalizer::~FaceNormalizer(){};


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


bool FaceNormalizer::detect_feature(cv::Mat& img,cv::Vec3f& coords,int code)
{

  CvSeq* seq;
  if(code==PP_NOSE)
  {
    IplImage ipl_img=(IplImage)img;
     seq=cvHaarDetectObjects(&ipl_img,nose_cascade_,nose_storage_,1.3,2,CV_HAAR_DO_CANNY_PRUNING,cvSize(15,15));
  }


  if(code==PP_EYE_L)
  {
    IplImage ipl_img=(IplImage)img;
     seq=cvHaarDetectObjects(&ipl_img,eye_l_cascade_,eye_l_storage_,1.15,3,CV_HAAR_DO_CANNY_PRUNING,cvSize(25,15));
  }
  if(code==PP_EYE_R)
  {
    IplImage ipl_img=(IplImage)img;
     seq=cvHaarDetectObjects(&ipl_img,eye_r_cascade_,eye_r_storage_,1.25,3,CV_HAAR_DO_CANNY_PRUNING,cvSize(25,15));
  }
  if(code==PP_MOUTH)
  {
    IplImage ipl_img=(IplImage)img;
     seq=cvHaarDetectObjects(&ipl_img,mouth_cascade_,mouth_storage_,1.3,4,CV_HAAR_DO_CANNY_PRUNING,cvSize(15,15));
  }

    if(seq->total ==0) return false;
    std::cout<<"SEQ TOTAL"<<code<<" "<<seq->total<<std::endl;
    Rect* seq_det=(Rect*)cvGetSeqElem(seq,0);
    coords[0]=(float)seq_det->x+seq_det->width/2;
    coords[1]=(float)seq_det->y+seq_det->height/2;
    coords[2]=0.0;

    return true;

}


void FaceNormalizer::normalizeFaces(std::vector<cv::Mat>& head_color,
                              std::vector<cv::Mat>& head_depth,
                              std::vector<std::vector<cv::Rect> >& face_rect)
{

  //tf_crop(head_color,face_rect);
  if(head_color.size()==0) return;


 //resetNormFeatures();
 for(int face=0;face<head_color.size();++face)
 {

  if(face_rect[face].size()==0) return;

   head_color[face]=head_color[face](face_rect[face][0]);
   head_depth[face]=head_depth[face](face_rect[face][0]);
   cvtColor(head_color[face],head_color[face],CV_BGR2RGB);
   cv::resize(head_color[face],head_color[face],norm_size_,0,0);
   cv::resize(head_depth[face],head_depth[face],norm_size_,0,0);
   cv::Vec3f mouth,nose,eye_r,eye_l;
   bool coord=getCoords(head_color[face],head_depth[face],mouth,nose,eye_l,eye_r);

   if(coord==false)return;
  showFeatures(head_color[face],nose,mouth,eye_l,eye_r);

   cv::Mat trafo(2,3,CV_32FC1);
   cv::Mat warped = cv::Mat(head_color[face].rows,head_color[face].cols,head_color[face].type());

       transformAffine(eye_l,eye_r,nose,trafo);
       cv::warpAffine(head_color[face],warped,trafo,warped.size() );
       showImg(warped,"warped");
 }
}

void FaceNormalizer::transformAffine(cv::Vec3f& eye_l,cv::Vec3f& eye_r,cv::Vec3f& nose,cv::Mat& trafo)
{
  cv::Point2f src[3],dst[3];

  src[0]=Point2f(eye_l[0],eye_l[1]);
  src[1]=Point2f(eye_r[0],eye_r[1]);
  src[2]=Point2f(nose[0],nose[1]);

  dst[0]=Point2f(norm_eye_l_[0],norm_eye_l_[1]);
  dst[1]=Point2f(norm_eye_r_[0],norm_eye_r_[1]);
  dst[2]=Point2f(norm_nose_[0],norm_nose_[1]);


//----
  std::cout<<"x eye l"<<eye_l[0]<<" "<<norm_eye_l_[0]<<std::endl;
  std::cout<<"y eye l"<<eye_l[1]<<" "<<norm_eye_l_[1]<<std::endl;
  std::cout<<"z eye l"<<eye_l[2]<<" "<<norm_eye_l_[2]<<std::endl;
  std::cout<<"x eye_r"<<eye_r[0]<<" "<<norm_eye_r_[0]<<std::endl;
  std::cout<<"y eye_r"<<eye_r[1]<<" "<<norm_eye_r_[1]<<std::endl;
  std::cout<<"z eye_r"<<eye_r[2]<<" "<<norm_eye_r_[2]<<std::endl;
  std::cout<<"x nose"<<nose[0]<<" "<<norm_nose_[0]<<std::endl;
  std::cout<<"y nose"<<nose[1]<<" "<<norm_nose_[1]<<std::endl;
  std::cout<<"z nose"<<nose[2]<<" "<<norm_nose_[2]<<std::endl;
  std::cout<<"------------------------------------\n";
//----

trafo = cv::getAffineTransform(src,dst);
//trafo = cv::getAffineTransform(src,src);



}



bool FaceNormalizer::getCoords(cv::Mat& img_color,cv::Mat& img_depth,cv::Vec3f& mouth_coord,
    cv::Vec3f& nose_coord,cv::Vec3f& eye_l_coord,cv::Vec3f& eye_r_coord)
{

 // IplImage ipl_img=(IplImage)img_color;
//
//
//
//
//  if(noses->total >0 && eyes->total >1 && mouths->total >0)
//  {
//    Rect* nose_det=(Rect*)cvGetSeqElem(noses,0);
//    Rect* mouth_det=(Rect*)cvGetSeqElem(mouths,0);
//    Rect eye_l_det,eye_r_det;
//    for(int eye_it=0;eye_it<2;eye_it++){
//    Rect* eyes_det=(Rect*)cvGetSeqElem(eyes,eye_it);
//
//      if(eye_it==0){
//         eye_l_det=*eyes_det;
//      }
//      if(eye_it==1){
//         eye_r_det=*eyes_det;
//    }
//      }




// once detected -> polausibility  checks ( left right up down etc)
//
// get depth information
// nose coordinate
  //float  nose_depth=head_depth[face].at<CV_32F>((int)nose_det->x,nose_det->y+nose_det->height);

////NOSE
//  nose_coord[0]=(float)nose_det->x+nose_det->width/2;
//  nose_coord[1]=(float)nose_det->y+nose_det->height/2;
//  //getDepthInRect(img_depth,*nose_det,nose_coord[2]);
//
//  //MOUTH
//  mouth_coord[0]=(float)mouth_det->x+mouth_det->width/2;
//  mouth_coord[1]=(float)mouth_det->y+mouth_det->height/2;
//  //getDepthInRect(img_depth,*mouth_det,mouth_coord[2]);
//  //norm_mouth_=head_depth[face].at<cv::Vec3f>(norm_mouth_[0],norm_mouth_[1]);
//
////EYE LEFT
//  eye_l_coord[0]=(float)eye_l_det.x+eye_l_det.width/2;
//  eye_l_coord[1]=(float)eye_l_det.y+eye_l_det.height/2;
//  //getDepthInRect(img_depth,eye_l_det,eye_l_coord[2]);
//  //norm_eye_l_=head_depth[face].at<cv::Vec3f>(norm_eye_l_[0],norm_eye_l_[1]);
//
////EYE RIGHT
//  eye_r_coord[0]=(float)eye_r_det.x+eye_r_det.width/2;
//  eye_r_coord[1]=(float)eye_r_det.y+eye_r_det.height/2;
//  //getDepthInRect(img_depth,eye_r_det,eye_r_coord[2]);

  detect_feature(img_color,eye_l_coord,PP_EYE_L);
  detect_feature(img_color,eye_r_coord,PP_EYE_R);
  detect_feature(img_color,nose_coord,PP_NOSE);
  detect_feature(img_color,mouth_coord,PP_MOUTH);

//  bool intersected=checkIntersection(eye_l_det,eye_r_det,*nose_det,*mouth_det);
  bool plausible =checkPlausibility(eye_l_coord,eye_r_coord,nose_coord,mouth_coord);
 // if( plausible == true)  {
 //     showFeatures(img_color,*nose_det,*mouth_det,eye_l_det,eye_r_det);
 //     return true;
 // }
 // else{
 //   return false;
 // }
 //
 //}
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
  norm_eye_l_[0]=40;
  norm_eye_l_[1]=40;
  norm_eye_l_[2]=0;
  norm_eye_r_[0]=100;
  norm_eye_r_[1]=40;
  norm_eye_r_[2]=0;
  norm_mouth_[0]=60;
  norm_mouth_[1]=100;
  norm_mouth_[2]=0;
  norm_nose_[0]=80;
  norm_nose_[1]=90;
  norm_nose_[2]=0;

}

void FaceNormalizer::showFeatures(cv::Mat img,cv::Vec3f& nose,cv::Vec3f& mouth,cv::Vec3f& eye_l,cv::Vec3f& eye_r)
{

  cv::Mat img2;
  img.copyTo(img2);
  IplImage ipl_img=(IplImage)img2;
   cv::circle(img2,cv::Point(nose[0],nose[1]),3,CV_RGB(255,0,0));
   cv::circle(img2,cv::Point(mouth[0],mouth[1]),3,CV_RGB(0,255,0));
   cv::circle(img2,cv::Point(eye_l[0],eye_l[1]),3,CV_RGB(255,255,0));
   cv::circle(img2,cv::Point(eye_r[0],eye_r[1]),3,CV_RGB(255,0,255));
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
  cv::waitKey(50);
    }

bool FaceNormalizer::checkPlausibility(cv::Vec3f& eye_l,cv::Vec3f& eye_r,cv::Vec3f& nose,cv::Vec3f& mouth)
{
//  eye_l------------eye_r
//            |
//            |
//           nose
//            |
//            |
//          mouth
//
  cv::Vec3f temp;
  if(eye_l[0]>eye_r[0])
  {
    eye_l=temp;
    eye_r=eye_l;
    eye_l=temp;
  }

  if(nose[1]<eye_l[1])   return false;
  if(nose[1]<eye_r[1])   return false;
  if(mouth[1]<eye_l[1])  return false;
  if(mouth[1]<eye_r[1])  return false;
  if(mouth[1]<nose[1])   return false;

      return true;


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




