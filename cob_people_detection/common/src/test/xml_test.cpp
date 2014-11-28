#include<cob_people_detection/subspace_analysis.h>
#include<opencv/cv.h>
#include<opencv/highgui.h>
#include<iostream>
#include<fstream>

int main(int argc, const char *argv[])
{

	std::string file1 = "/home/goa-tz/git/care-o-bot/cob_people_perception/cob_people_detection/common/files/training_data/30.xml";
	std::string file2 = "/home/goa-tz/git/care-o-bot/cob_people_perception/cob_people_detection/common/files/training_data/31.xml";
	std::string file3 = "/home/goa-tz/git/care-o-bot/cob_people_perception/cob_people_detection/common/files/training_data/32.xml";
	std::string file4 = "/home/goa-tz/git/care-o-bot/cob_people_perception/cob_people_detection/common/files/training_data/33.xml";
	std::string file5 = "/home/goa-tz/git/care-o-bot/cob_people_perception/cob_people_detection/common/files/training_data/34.xml";
	std::string file6 = "/home/goa-tz/git/care-o-bot/cob_people_perception/cob_people_detection/common/files/training_data/35.xml";
	std::string file7 = "/home/goa-tz/git/care-o-bot/cob_people_perception/cob_people_detection/common/files/training_data/36.xml";
	std::string file8 = "/home/goa-tz/git/care-o-bot/cob_people_perception/cob_people_detection/common/files/training_data/37.xml";
	std::string file9 = "/home/goa-tz/git/care-o-bot/cob_people_perception/cob_people_detection/common/files/training_data/38.xml";
	std::string file10 = "/home/goa-tz/git/care-o-bot/cob_people_perception/cob_people_detection/common/files/training_data/39.xml";
	std::string file11 = "/home/goa-tz/git/care-o-bot/cob_people_perception/cob_people_detection/common/files/training_data/40.xml";
	std::string file12 = "/home/goa-tz/git/care-o-bot/cob_people_perception/cob_people_detection/common/files/training_data/41.xml";
	std::string file13 = "/home/goa-tz/git/care-o-bot/cob_people_perception/cob_people_detection/common/files/training_data/42.xml";
	std::string file14 = "/home/goa-tz/git/care-o-bot/cob_people_perception/cob_people_detection/common/files/training_data/43.xml";

	std::vector < std::string > in_vec;
	std::vector<int> label_vec;
	in_vec.push_back(file1);
	label_vec.push_back(0);
	in_vec.push_back(file2);
	label_vec.push_back(0);
	in_vec.push_back(file3);
	label_vec.push_back(0);
	in_vec.push_back(file4);
	label_vec.push_back(0);
	in_vec.push_back(file5);
	label_vec.push_back(0);
	in_vec.push_back(file6);
	label_vec.push_back(0);
	in_vec.push_back(file7);
	label_vec.push_back(0);
	in_vec.push_back(file8);
	label_vec.push_back(1);
	in_vec.push_back(file9);
	label_vec.push_back(1);
	in_vec.push_back(file10);
	label_vec.push_back(1);
	in_vec.push_back(file11);
	label_vec.push_back(1);
	in_vec.push_back(file12);
	label_vec.push_back(1);
	in_vec.push_back(file13);
	label_vec.push_back(1);
	in_vec.push_back(file14);
	label_vec.push_back(1);
	// std::vector<int> label_vec;
	std::vector<cv::Mat> img_vec;

	for (int i = 0; i < in_vec.size(); i++)
	{
		cv::Mat img;
		cv::FileStorage fs(in_vec[i], cv::FileStorage::READ);
		fs["depthmap"] >> img;
		//img.convertTo(img,CV_8UC1,255);
		//cv::equalizeHist(img,img);
		//cv::imshow("img",img);
		//cv::waitKey(0);
		cv::resize(img, img, cv::Size(120, 120));
		img.convertTo(img, CV_64FC1);
		img_vec.push_back(img);
		fs.release();

	}

	std::vector<cv::Mat> probe_mat_vec;
	probe_mat_vec.push_back(img_vec[0]);

	std::cout << "Size Training Set= " << img_vec.size() << std::endl;

	int ss_dim = 2;
	//SubspaceAnalysis::Eigenfaces EF;
	// EF.init(img_vec,label_vec,ss_dim);
	//std::vector<cv::Mat> eigenvecsEF(ss_dim);
	//cv::Mat eigenvalsEF,avgEF,projsEF;
	//EF.retrieve(eigenvecsEF,eigenvalsEF,avgEF,projsEF,cv::Size(img_vec[0].cols,img_vec[0].rows));
	//SubspaceAnalysis::dump_matrix(projsEF,"projEF");


	//SubspaceAnalysis::Fisherfaces FF;
	// FF.init(img_vec,label_vec);

	//cv::Mat eigenvalsFF,avgFF,projsFF;
	//std::vector<cv::Mat> eigenvecsFF(ss_dim);
	//FF.retrieve(eigenvecsFF,eigenvalsFF,avgFF,projsFF);
	//SubspaceAnalysis::dump_matrix(projsFF,"projFF");

	SubspaceAnalysis::FishEigFaces EFF;

	EFF.init(img_vec, label_vec, ss_dim, SubspaceAnalysis::METH_FISHER, true, true);

	for (int i = 0; i < probe_mat_vec.size(); i++)
	{
		cv::Mat probe = probe_mat_vec[i];
		////double DFFS;
		////cv::Mat feats;
		//cv::Mat coeff_FF;
		//int c_FF;
		//double DFFS_FF;
		//FF.projectToSubspace(probe,coeff_FF,DFFS_FF);
		//FF.classify(coeff_FF,SubspaceAnalysis::CLASS_SVM,c_FF);
		//std::cout<<"class FF SVM= "<<c_FF<<std::endl;
		//FF.classify(coeff_FF,SubspaceAnalysis::CLASS_KNN,c_FF);
		//std::cout<<"class FF KNN= "<<c_FF<<std::endl;
		//FF.classify(coeff_FF,SubspaceAnalysis::CLASS_DIFS,c_FF);
		//std::cout<<"class FF DIFFS= "<<c_FF<<std::endl;

		//SubspaceAnalysis::dump_matrix(coeff_FF,"sampleFF");
		//std::cout<<"--------------------------\n";

		//int c_EF;
		//cv::Mat coeff_EF;
		//double DFFS_EF;
		//EF.projectToSubspace(probe,coeff_EF,DFFS_EF);
		//EF.classify(coeff_EF,SubspaceAnalysis::CLASS_SVM,c_EF);
		//std::cout<<"class EF SVM= "<<c_EF<<std::endl;
		//EF.classify(coeff_EF,SubspaceAnalysis::CLASS_KNN,c_EF);
		//std::cout<<"class EF KNN= "<<c_EF<<std::endl;
		//EF.classify(coeff_EF,SubspaceAnalysis::CLASS_DIFS,c_EF);
		//std::cout<<"class EF DIFFS= "<<c_EF<<std::endl;

		//SubspaceAnalysis::dump_matrix(coeff_EF,"sampleEF");

		//std::cout<<"--------------------------\n";

		int c_EFF;
		cv::Mat coeff_EFF;
		double DFFS_EFF;
		EFF.projectToSubspace(probe, coeff_EFF, DFFS_EFF);
		EFF.classify(coeff_EFF, SubspaceAnalysis::CLASS_SVM, c_EFF);
		std::cout << "class EFF SVM= " << c_EFF << std::endl;
		EFF.classify(coeff_EFF, SubspaceAnalysis::CLASS_KNN, c_EFF);
		std::cout << "class EFF KNN= " << c_EFF << std::endl;
		EFF.classify(coeff_EFF, SubspaceAnalysis::CLASS_DIFS, c_EFF);
		std::cout << "class EFF DIFFS= " << c_EFF << std::endl;

		SubspaceAnalysis::dump_matrix(coeff_EFF, "sampleEFF");
	}

	return 0;
}
