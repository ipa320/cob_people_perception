#include"cob_people_detection/face_normalizer.h"
#include<iostream>
#include<opencv/cv.h>
#include<opencv/highgui.h>
int main(int argc, const char *argv[])
{

	std::cout << "[FaceNormalizer] running scene no. " << argv[1] << "...\n";
	FaceNormalizer::FNConfig cfg;
	cfg.eq_ill = true;
	cfg.align = true;
	cfg.resize = true;
	cfg.cvt2gray = false;
	cfg.extreme_illumination_condtions = false;

	FaceNormalizer fn;
	fn.init(cfg);

	cv::Mat depth, img, xyz;
	std::string i_path;
	i_path = "/share/goa-tz/people_detection/eval/kinect3d_features/";

	i_path.append(argv[1]);
	std::string pgm_path = i_path;
	std::cout << "FN running on " << pgm_path << std::endl;

	img = cv::imread(pgm_path);
	cv::Mat wmat1;
	img.copyTo(wmat1);
	cv::Size norm_size = cv::Size(img.cols, img.rows);
	//cv::cvtColor(wmat1,wmat1,CV_RGB2BGR);

	cv::imshow("ORIGINAL", wmat1);
	cv::Mat depth_res;
	fn.normalizeFace(wmat1, norm_size);

	//fn.normalizeFace(wmat1,norm_size);
	//
	cv::imshow("NORMALIZED", wmat1);
	cv::waitKey(0);

	std::cout << "..done\n";
	return 0;
}
