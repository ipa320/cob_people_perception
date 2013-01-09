#include"cob_people_detection/subspace_analysis.h"
#include<opencv/highgui.h>
#include<fstream>
#include<ostream>


void SubspaceAnalysis:: mat_info(cv::Mat& mat)
{
  //for(int r=0;r<mat.rows;r++)
  //{
  //  for(int c=0;c<mat.cols;c++)
  //  {
  //    std::cout<<mat.at<double>(r,c)<<std::endl;
  //  }
  //}
  std::cout<<"Matrix info:\n";
  std::cout<<"rows= "<<mat.rows<<"  cols= "<<mat.cols<<std::endl;
  std::cout<<"Type = "<<mat.type()<<std::endl;
  std::cout<<"Channels = "<<mat.depth()<<std::endl;
}

void SubspaceAnalysis::DFFS(cv::Mat& orig_mat,cv::Mat& recon_mat,cv::Mat& avg,std::vector<double>& DFFS)
{

  cv::Mat temp=cv::Mat(orig_mat.rows,orig_mat.cols,orig_mat.type());
  orig_mat.copyTo(temp);
  DFFS.resize(recon_mat.rows);
  for(int i=0;i<orig_mat.rows;i++)
  {
    cv::Mat recon_row=recon_mat.row(i);
    cv::Mat temp_row=temp.row(i);
    cv::subtract(temp_row,avg,temp_row);
    DFFS[i]= cv::norm(recon_row,temp_row,cv::NORM_L2);
  }
  return;
}
void SubspaceAnalysis::project(cv::Mat& src_mat,cv::Mat& proj_mat,cv::Mat& avg_mat,cv::Mat& coeff_mat)
{

  // reduce im mat by mean
  for(int i=0;i<src_mat.rows;i++)
  {
    cv::Mat im=src_mat.row(i);
    cv::subtract(im,avg_mat.reshape(1,1),im);
  }

  //calculate coefficients

  cv::gemm(src_mat,proj_mat,1.0,cv::Mat(),0.0,coeff_mat,cv::GEMM_2_T);

}

void SubspaceAnalysis::reconstruct(cv::Mat& coeffs,cv::Mat& proj_mat,cv::Mat& avg,cv::Mat& rec_im)
{
  cv::gemm(coeffs,proj_mat,1.0,cv::Mat(),0.0,rec_im);
  for(int i=0;i<rec_im.rows;i++)
  {
    cv::Mat curr_row=rec_im.row(i);
    cv::add(curr_row,avg.reshape(1,1),curr_row);
  }
}

void SubspaceAnalysis::calcDataMat(std::vector<cv::Mat>& input_data,cv::Mat& data_mat)
{

  // convert input to data matrix,with images as rows


    double* src_ptr;
    double*  dst_ptr;
    cv::Mat src_mat;
    for(int i = 0;i<input_data.size();i++)
    {
      input_data[i].copyTo(src_mat);

      src_ptr = src_mat.ptr<double>(0,0);
      //src_ptr = input_data[i].ptr<double>(0,0);
      dst_ptr = data_mat.ptr<double>(i,0);

      for(int j=0;j<input_data[i].rows;j++)
      {
        src_ptr=src_mat.ptr<double>(j,0);
        //src_ptr=input_data[i].ptr<double>(j,0);
        for(int col=0;col<input_data[i].cols;col++)
        {
        *dst_ptr=*src_ptr;
        src_ptr++;
        dst_ptr++;
        }
      }

    }


    //cv::Mat src_mat=cv::Mat(120,120,CV_64FC1);
    //src_mat= input_data[0].clone();
    ////src_mat.convertTo(src_mat,CV_64F);
    //std::cout<<"src"<<src_mat.rows<<","<<src_mat.cols<<std::endl;
    //src_mat=src_mat.reshape(1,1);
    //src_mat.clone();
    //std::cout<<"src"<<src_mat.rows<<","<<src_mat.cols<<std::endl;
    //cv::Mat dst_mat;
    //dst_mat.push_back(src_mat);

    //std::cout<<"dst"<<dst_mat.rows<<","<<dst_mat.cols<<std::endl;
    //dst_mat=dst_mat.reshape(1,120);
    //dst_mat.convertTo(dst_mat,CV_8UC1);
    //cv::imshow("DST",dst_mat);
    //cv::waitKey(0);
    
    

  return;
}

void SubspaceAnalysis::dump_matrix(cv::Mat& mat,std::string filename)
{
  std::string path = "/share/goa-tz/people_detection/debug/data/";
  path.append(filename.c_str());
  std::ofstream os(path.c_str() );
  for(int r=0;r<mat.rows;r++)
  {
    for(int c=0;c<mat.cols;c++)
    {
    os<<mat.at<double>(r,c)<<" ";
    }
    os<<"\n";
  }
  os.close();
}

//---------------------------------------------------------------------------------
// EIGENFACES
//---------------------------------------------------------------------------------
//
//
SubspaceAnalysis::Eigenfaces::Eigenfaces(std::vector<cv::Mat>& img_vec,int& dim_ss)
{
  //check if input has the same size
  if(img_vec.size()<dim_ss+1)
  {
    //TODO: ROS ERROR
    std::cout<<"EIGFACE: Invalid subspace dimension\n";
    return;
  }

  //initialize all matrices
  model_data_arr_=cv::Mat(img_vec.size(),img_vec[0].total(),CV_64FC1);
  avg_arr_=cv::Mat(1,img_vec[0].total(),CV_64FC1);
  proj_model_data_arr_=cv::Mat(img_vec.size(),dim_ss,CV_64FC1);
  eigenvector_arr_=cv::Mat(img_vec.size(),img_vec[0].total(),CV_64FC1);
  eigenvalue_arr_=cv::Mat(dim_ss,dim_ss,CV_64FC1);

  std::cout<<"[EF]-->calc data mat"<<std::endl;
  SubspaceAnalysis::calcDataMat(img_vec,model_data_arr_);
  std::cout<<"[EF]-->done"<<std::endl;

  cv::Mat dummy;
  model_data_arr_.copyTo(dummy);
  dummy.convertTo(dummy,CV_8UC1);
  dummy =dummy.reshape(1,dummy.rows*160);
  cv::imshow("reconstuction",dummy);
  cv::imwrite("/home/goa-tz/Desktop/model_data_.jpg",dummy);


  //initiate PCA
  std::cout<<"[EF]-->calc PCA"<<std::endl;
  pca_=SubspaceAnalysis::PCA(model_data_arr_,dim_ss);
  std::cout<<"[EF]-->done"<<std::endl;

  eigenvector_arr_=pca_.eigenvecs;
  eigenvalue_arr_=pca_.eigenvals;
  avg_arr_=pca_.mean;

  std::vector<double> DFFS;


 // cv::Mat ss=model_data_(cv::Rect(0,0,120*120,3));
 // projectToSubspace(ss,proj_model_data_,DFFS);
 // dump_matrix(proj_model_data_,"projection1");

 // cv::Mat ss2=model_data_(cv::Rect(0,2,120*120,3));
 // projectToSubspace(ss2,proj_model_data_,DFFS);
 // dump_matrix(proj_model_data_,"projection2");

  std::cout<<"[EF]-->projecting to subspace"<<std::endl;
  projectToSubspace(model_data_arr_,proj_model_data_arr_,DFFS);
  std::cout<<"[EF]-->done"<<std::endl;

}



void SubspaceAnalysis::Eigenfaces::projectToSubspace(cv::Mat& src_mat,cv::Mat& dst_mat,std::vector<double>& DFFS)
{
  SubspaceAnalysis::project(src_mat,eigenvector_arr_,avg_arr_,dst_mat);

  cv::Mat rec_mat;
  SubspaceAnalysis::reconstruct(dst_mat,eigenvector_arr_,avg_arr_,rec_mat);

  SubspaceAnalysis::DFFS(src_mat,rec_mat,avg_arr_,DFFS);

  cv::Mat dummy;
  rec_mat.copyTo(dummy);
  dummy.convertTo(dummy,CV_8UC1);
  dummy =dummy.reshape(1,dummy.rows*160);
  cv::imshow("reconstuction",dummy);
  cv::imwrite("/home/goa-tz/Desktop/reconstructed.jpg",dummy);

}
void SubspaceAnalysis::Eigenfaces::meanCoeffs(cv::Mat& coeffs,std::vector<int>& label_vec,cv::Mat& mean_coeffs)
{
  std::vector<int> distinct_vec;
  bool unique=true;
  for(int i=0;i<label_vec.size();++i)
  {

    if(i!=0)
    {
    unique=true;
    for(int j=0;j<distinct_vec.size();j++)
    {
      if(label_vec[i]==distinct_vec[j]) unique =false;
    }
    }
    if(unique==true)distinct_vec.push_back(label_vec[i]);
  }
  int num_classes=distinct_vec.size();


  mean_coeffs=cv::Mat::zeros(num_classes,coeffs.cols,CV_64FC1);
  std::vector<int>     samples_per_class(num_classes);

  //initialize vectors with zeros
  for( int i =0;i<num_classes;i++)
  {
   samples_per_class[i]=0;
  }

  int class_index;
  for(int i =0;i<coeffs.rows;i++)
  {
    cv::Mat curr_row=coeffs.row(i);
    class_index=label_vec[i];

    cv::Mat mean_row=mean_coeffs.row(class_index);

    add(mean_row,curr_row,mean_row);
    samples_per_class[class_index]++;
  }

  for (int i = 0; i < num_classes; i++) {
  cv::Mat mean_row=mean_coeffs.row(i);
  mean_row.convertTo(mean_row,CV_64FC1,1.0/static_cast<double>(samples_per_class[i]));

  }
}

void SubspaceAnalysis::Eigenfaces::retrieve(std::vector<cv::Mat>& out_eigenvectors,cv::Mat& out_eigenvalues,cv::Mat& out_avg,cv::Mat& out_proj_model_data)
{
  avg_arr_.copyTo(out_avg);
  proj_model_data_arr_.copyTo(out_proj_model_data);

  for(int r=0;r<eigenvector_arr_.rows;r++)
  {
    cv::Mat curr_row=eigenvector_arr_.row(r);
    //TODO: works only for square images
    curr_row.clone().reshape(1,sqrt(curr_row.cols));
    curr_row.convertTo(curr_row,CV_32FC1);
    //TODO:THE NEXT LINE BREAKS THE ALGORITHM ! FIX IT!!!!!!!!!!!!1111111111
    curr_row.copyTo(out_eigenvectors[r]);
    //out_eigenvectors[r]=curr_row;
  }

  eigenvalue_arr_.copyTo(out_eigenvalues);
  //double* ev_ptr=out_eigenvalues.ptr<double>(0);
  //for(int rc=0;rc<eigenvector_arr_.cols;rc++)
  //{
  //  *ev_ptr=eigenvalue_arr_.at<double>(rc,rc);
  //  std::cout<<"EV"<<*ev_ptr<<std::endl;
  //}

}




//---------------------------------------------------------------------------------
// SSA
//---------------------------------------------------------------------------------


SubspaceAnalysis::SSA::SSA(cv::Mat& data_mat,int& ss_dim):dimension(ss_dim)
{
  calcDataMatMean(data_mat,mean);
}


void SubspaceAnalysis::SSA::calcDataMatMean(cv::Mat& data,cv::Mat& mean)
{
  cv::Mat row_cum=cv::Mat::zeros(1,data.cols,CV_64FC1);
  for(int i=0;i<data.rows;++i)
  {
    cv::Mat data_row=data.row(i);
    //calculate mean
    cv::add(row_cum,data_row,row_cum);
  }
  row_cum.convertTo(mean,CV_64F,1.0/static_cast<double>(data.rows));

  //mean.convertTo(mean,CV_8UC1);
  //mean=mean.reshape(1,120);
  //cv::imshow("mean",mean);
  //cv::waitKey(0);
  //return;
}


void SubspaceAnalysis::SSA::decompose(cv::Mat& data_mat)
{

  data_mat.convertTo(data_mat,CV_64F,1/sqrt(3));
  cv::SVD svd(data_mat.t());
  eigenvecs=svd.u;
  //svd.u.copyTo(eigenvecs);
  eigenvecs=eigenvecs.t();
  //svd.w.copyTo(eigenvals);
  eigenvals=svd.w;


}

void SubspaceAnalysis::SSA::decompose2(cv::Mat& data_mat)
{
  cv::Mat zero_mean=cv::Mat::zeros(1,data_mat.cols,CV_64FC1);
  cv::PCA pca(data_mat,zero_mean,CV_PCA_DATA_AS_ROW,dimension);
  eigenvecs=pca.eigenvectors;
  //pca.eigenvectors.copyTo(eigenvecs);
}



//---------------------------------------------------------------------------------
// LDA
//---------------------------------------------------------------------------------
SubspaceAnalysis::LDA::LDA(cv::Mat& input_data,std::vector<int>& input_labels,int& ss_dim): SSA(input_data,ss_dim)
{

  //class labels have to be in a vector in ascending order - duplicates are
  //removed internally
  //{0,0,1,2,3,3,4,5}

  calcClassMean(data,input_labels,class_means);
  calcModelMatrix(input_labels,model);
  decompose(model);

}
void SubspaceAnalysis::LDA::calcClassMean(cv::Mat& data_mat,std::vector<int>& label_vec,std::vector<cv::Mat>&  mean_vec)
{

  std::vector<int> distinct_vec;
  bool unique=true;
  for(int i=0;i<label_vec.size();++i)
  {
    if(i==0)distinct_vec.push_back(label_vec[i]);continue;

    unique=true;
    for(int j=0;j<distinct_vec.size();j++)
    {
      if(label_vec[i]==distinct_vec[j]) unique =false;
    }
    if(unique==true)distinct_vec.push_back(label_vec[i]);
  }
  num_classes=distinct_vec.size();



  std::vector<cv::Mat> mean_of_class(num_classes);
  std::vector<int>     samples_per_class(num_classes);

  //initialize vectors with zeros
  for( int i =0;i<num_classes;i++)
  {
   mean_of_class[i]=cv::Mat::zeros(1,data_mat.cols,CV_64FC1);
   samples_per_class[i]=0;
  }

  int class_index;
  for(int i =0;i<data_mat.rows;i++)
  {
    cv::Mat data_row=data_mat.row(i);
    class_index=label_vec[i];

    add(mean_of_class[class_index],data_row,mean_of_class[class_index]);
    samples_per_class[class_index]++;
  }

  for (int i = 0; i < num_classes; i++) {
  mean_of_class[i].convertTo(mean_vec[i],CV_64FC1,1.0/static_cast<double>(samples_per_class[i]));

  }
}

void SubspaceAnalysis::LDA::calcModelMatrix(std::vector<int>& label_vec,cv::Mat& M)
{
 //reduce data matrix with class means and compute inter class scatter
  // inter class scatter
  cv::Mat S_inter=cv::Mat::zeros(data.rows,data.rows,CV_64FC1);
  cv::Mat temp;

  int class_index;
  cv::Mat data_row;
  for(int i=0;i<num_classes;++i)
  {
    //reduce data matrix
    class_index=label_vec[i];
    data_row =data.row(i);
    cv::subtract(data_row,class_means[class_index],data_row);
    //compute interclass scatte
    cv::subtract(class_means[class_index],mean,temp);
    cv::mulTransposed(temp,temp,true);
    cv::add(S_inter,temp,S_inter);
  }

  //intra-class scatter
  cv::Mat S_intra=cv::Mat::zeros(data.rows,data.rows,CV_64FC1);
  mulTransposed(data,S_intra,true);
  cv::Mat S_intra_inv=S_intra.inv();

  gemm(S_intra_inv,S_inter,1.0,cv::Mat(),0.0,M);

  return;

}

//---------------------------------------------------------------------------------
// PCA
//---------------------------------------------------------------------------------
//
SubspaceAnalysis::PCA::PCA(cv::Mat& input_data,int& ss_dim):SSA(input_data,ss_dim)
{
  data=input_data;
  calcProjMatrix(input_data);
  //truncate eigenvectors and eigenvals
  eigenvecs=eigenvecs(cv::Rect(0,0,input_data.cols,dimension));
  eigenvals=eigenvals(cv::Rect(0,0,1,dimension)).t();
  cv::normalize(eigenvecs,eigenvecs);

  cv::Mat dummy;
  eigenvecs.copyTo(dummy);
  dummy.convertTo(dummy,CV_8UC1,1000);
  dummy =dummy.reshape(1,dummy.rows*160);
  cv::equalizeHist(dummy,dummy);
  cv::imwrite("/home/goa-tz/Desktop/eigenfaces.jpg",dummy);


}

void SubspaceAnalysis::PCA::calcProjMatrix(cv::Mat& data)
{
     cv::Mat data_row;
     for(int i=0;i<data.rows;++i)
     {
       //reduce data matrix - total Scatter matrix
       data_row =data.row(i);
       cv::subtract(data_row,mean,data_row);
     }

     decompose(data);
}


