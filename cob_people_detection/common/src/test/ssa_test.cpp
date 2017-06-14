#include <cob_people_detection/subspace_analysis.h>
#include <cob_people_detection/face_normalizer.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <sys/time.h>
#include <boost/timer.hpp>

bool preprocess(cv::Mat& img, cv::Mat& xyz, FaceNormalizer* fn, bool normalize, cv::Size& norm_size, cv::Mat& dm)
{
	//cv::Size norm_size=cv::Size(120,120);
	bool valid = true;
	if (normalize)
	{
		valid = fn->normalizeFace(img, xyz, norm_size, dm);

		// dm.convertTo(dm,CV_8UC1);
		// cv::equalizeHist(dm,dm);
		// cv::imshow("normalized",dm);
		// cv::waitKey(5);

	}
	else
	{
		cv::resize(img, img, norm_size);
	}

	dm.convertTo(dm, CV_64FC1);
	img.convertTo(img, CV_64FC1);
	return valid;
}
bool preprocess(cv::Mat& img, FaceNormalizer* fn, bool normalize, cv::Size& norm_size)
{
	bool valid = true;
	//cv::Size norm_size=cv::Size(120,120);
	if (normalize)
	{
		valid = fn->normalizeFace(img, norm_size);
		//cv::imshow("normalized",img);
		//cv::waitKey(5);

	}
	else
	{
		cv::resize(img, img, norm_size);
	}

	img.convertTo(img, CV_64FC1);
	return valid;
}

int main(int argc, const char *argv[])
{

	FaceNormalizer::FNConfig config;
	config.eq_ill = true;
	config.align = false;
	config.resize = true;
	config.cvt2gray = true;
	config.extreme_illumination_condtions = false;

	FaceNormalizer* fn = new FaceNormalizer();
	fn->init(config);

	// parse input arguments from command line
	std::string method_str, classifier_str;
	bool use_xyz = true;
	bool normalizer = false;
	if (argc == 1)
	{
		method_str = "FISHER";
		classifier_str = "SVM";
		normalizer = false;
	}

	else if (argc == 2)
	{
		method_str = argv[1];
		classifier_str = "KNN";
		normalizer = false;
	}

	else if (argc == 3)
	{
		method_str = argv[1];
		classifier_str = argv[2];
		normalizer = false;
	}

	else if (argc == 4)
	{
		method_str = argv[1];
		classifier_str = argv[2];

		if (std::strcmp(argv[3], "0") == 0)
			normalizer = false;
		if (std::strcmp(argv[3], "1") == 0)
			normalizer = true;

		if (std::strcmp(argv[4], "0") == 0)
			use_xyz = false;
		if (std::strcmp(argv[4], "1") == 0)
			use_xyz = true;
	}

	else if (argc == 5)
	{
		method_str = argv[1];
		classifier_str = argv[2];
		if (std::strcmp(argv[3], "0") == 0)
			normalizer = false;
		if (std::strcmp(argv[3], "1") == 0)
			normalizer = true;

		if (std::strcmp(argv[4], "0") == 0)
			use_xyz = false;
		if (std::strcmp(argv[4], "1") == 0)
			use_xyz = true;
	}

	//  Configure input params for subspace analysis

	SubspaceAnalysis::Method method;
	SubspaceAnalysis::Classifier classifier;

	if (!method_str.compare("FISHER"))
	{
		std::cout << "FISHER" << std::endl;
		method = SubspaceAnalysis::METH_FISHER;
	}
	else if (!method_str.compare("EIGEN"))
	{
		std::cout << "EIGEN" << std::endl;
		method = SubspaceAnalysis::METH_EIGEN;
	}
	else if (!method_str.compare("LDA2D"))
	{
		std::cout << "LDA2D" << std::endl;
		method = SubspaceAnalysis::METH_LDA2D;
	}
	else if (!method_str.compare("PCA2D"))
	{
		std::cout << "PCA2D" << std::endl;
		method = SubspaceAnalysis::METH_PCA2D;
	}
	else
	{
		std::cout << "ERROR: invalid method - use FISHER or EIGEN" << std::endl;
	}

	if (!classifier_str.compare("KNN"))
	{
		std::cout << "KNN" << std::endl;
		classifier = SubspaceAnalysis::CLASS_KNN;
	}
	else if (!classifier_str.compare("DIFFS"))
	{
		std::cout << "DIFFS" << std::endl;
		classifier = SubspaceAnalysis::CLASS_DIFS;
	}
	else if (!classifier_str.compare("SVM"))
	{
		std::cout << "SVM" << std::endl;
		classifier = SubspaceAnalysis::CLASS_SVM;
	}
	else if (!classifier_str.compare("RF"))
	{
		std::cout << "RF" << std::endl;
		classifier = SubspaceAnalysis::CLASS_RF;
	}
	else
	{
		std::cout << "ERROR: invalid classifier - use KNN or DIFFS or SVM" << std::endl;
	}

	std::cout << "SSA test configuration:" << std::endl;
	std::cout << "classifier: " << classifier_str << std::endl;
	std::cout << "method: " << method_str << std::endl;
	std::cout << "normalizing: " << normalizer << std::endl;
	std::cout << "use xyz: " << use_xyz << std::endl;
	//HOME
	//std::string training_set_path=     "/home/tom/git/care-o-bot/cob_people_perception/cob_people_detection/debug/eval/eval_tool_files/training_set_list";
	//std::string training_set_xyz_path= "/home/tom/git/care-o-bot/cob_people_perception/cob_people_detection/debug/eval/eval_tool_files/training_set_xyz_list";
	//std::string probe_file_path=       "/home/tom/git/care-o-bot/cob_people_perception/cob_people_detection/debug/eval/eval_tool_files/probe_file_list";
	//std::string probe_file_xyz_path=   "/home/tom/git/care-o-bot/cob_people_perception/cob_people_detection/debug/eval/eval_tool_files/probe_file_xyz_list";
	//IPA
	std::string training_set_path = "/share/goa-tz/people_detection/eval/eval_tool_files/training_set_list";
	std::string training_set_xyz_path = "/share/goa-tz/people_detection/eval/eval_tool_files/training_set_xyz_list";
	std::string probe_file_path = "/share/goa-tz/people_detection/eval/eval_tool_files/probe_file_list";
	std::string probe_file_xyz_path = "/share/goa-tz/people_detection/eval/eval_tool_files/probe_file_xyz_list";

	//read probe file
	std::ifstream probe_file_stream(probe_file_path.c_str());

	std::string probe_file;
	std::vector < std::string > probe_file_vec;

	while (probe_file_stream >> probe_file)
	{
		//std::cout<<probe_file<<std::endl;
		probe_file_vec.push_back(probe_file);

	}

	std::vector < std::string > probe_file_xyz_vec;
	if (use_xyz)
	{
		std::ifstream probe_file_xyz_stream(probe_file_xyz_path.c_str());
		std::string probe_file_xyz;

		while (probe_file_xyz_stream >> probe_file_xyz)
		{
			//std::cout<<probe_file<<std::endl;
			probe_file_xyz_vec.push_back(probe_file_xyz);

		}
	}

	// read training set
	std::ifstream in_file(training_set_path.c_str());
	std::string img_file;

	int label = 0;
	std::vector < std::string > in_vec;
	std::vector<int> label_vec;

	while (in_file >> img_file)
	{
		if (std::strcmp(img_file.c_str(), "$$") == 0)
		{

			label++;
		}
		else
		{
			in_vec.push_back(img_file);
			label_vec.push_back(label);
		}

	}

	std::vector < std::string > in_vec_xyz;
	if (use_xyz)
	{
		// read xyz data
		std::ifstream in_file_xyz(training_set_xyz_path.c_str());
		std::string xml_file;

		while (in_file_xyz >> xml_file)
		{
			if (!std::strcmp(xml_file.c_str(), "$$") == 0)
			{
				in_vec_xyz.push_back(xml_file);
			}

		}
	}

	if ((use_xyz == true) && (in_vec.size() != in_vec_xyz.size() || probe_file_vec.size() != probe_file_xyz_vec.size()))
	{
		use_xyz = false;
		std::cerr << "Error - not for every image 3d information could be loaded - ignoring 3d information\n";
	}

	int num_classes = label;
	cv::Size norm_size;
	double aspect_ratio = 1;
	// load training images
	std::vector<cv::Mat> img_vec;
	std::vector<cv::Mat> dm_vec;

	std::string invalid_path = "/share/goa-tz/people_detection/eval/eval_tool_files/nrm_failed";
	//std::string invalid_path="/home/tom/git/care-o-bot/cob_people_perception/cob_people_detection/eval/eval_tool_files/nrm_failed";
	std::ofstream os_inv(invalid_path.c_str());
	bool valid;
	for (int i = 0; i < in_vec.size(); i++)
	{
		cv::Mat img;
		cv::Mat xyz;
		if (use_xyz)
		{
			cv::FileStorage fs(in_vec_xyz[i], FileStorage::READ);
			fs["depth"] >> xyz;
			fs["color"] >> img;
			fs.release();
		}
		else
		{
			img = cv::imread(in_vec[i], 0);
		}

		if (i == 0)
		{
			aspect_ratio = double(img.cols) / double(img.rows);
			norm_size = cv::Size(round(160 * aspect_ratio), 160);
			//norm_size=cv::Size(img.rows,img.cols);
		}
		valid = true;
		cv::Mat dm;
		if (use_xyz)
			valid = preprocess(img, xyz, fn, normalizer, norm_size, dm);
		//if(use_xyz)valid=preprocess(img,fn,normalizer,norm_size);
		if (!use_xyz)
			valid = preprocess(img, fn, normalizer, norm_size);

		img_vec.push_back(img);
		if (use_xyz)
			dm_vec.push_back(dm);

		if (!valid)
		{
			os_inv << in_vec[i] << "\n";
		}

	}

	// load test images
	std::vector<cv::Mat> probe_mat_vec;
	std::vector<cv::Mat> probe_dm_vec;
	for (int i = 0; i < probe_file_vec.size(); i++)
	{
		std::stringstream ostr, nstr;
		nstr << "/share/goa-tz/people_detection/eval/picdump/";
		//nstr<<"/home/tom/git/care-o-bot/cob_people_perception/cob_people_detection/debug/eval/picdump/";
		ostr << nstr.str().c_str() << i << "_orig" << ".jpg";

		cv::Mat probe_xyz, probe_img;
		if (use_xyz)
		{
			cv::FileStorage fs(probe_file_xyz_vec[i], FileStorage::READ);
			fs["depth"] >> probe_xyz;
			fs["color"] >> probe_img;
			fs.release();
		}
		else
		{
			probe_img = cv::imread(probe_file_vec[i], 0);
		}

		cv::imwrite(ostr.str().c_str(), probe_img);

		valid = true;
		cv::Mat dm;
		if (use_xyz)
			valid = preprocess(probe_img, probe_xyz, fn, normalizer, norm_size, dm);
		if (!use_xyz)
			valid = preprocess(probe_img, fn, normalizer, norm_size);

		cv::Mat oimg;
		probe_img.convertTo(oimg, CV_8UC1);
		//cv::equalizeHist(oimg,oimg);
		nstr << i << "_norm" << ".jpg";
		cv::imwrite(nstr.str().c_str(), oimg);

		probe_mat_vec.push_back(probe_img);
		if (use_xyz)
			probe_dm_vec.push_back(dm);

		if (!valid)
		{
			os_inv << probe_file_vec[i] << "\n";
		}

	}
	os_inv.close();

	std::cout << "Size Training Set= " << img_vec.size() << std::endl;
	std::cout << "Size Test Set= " << probe_file_vec.size() << std::endl;

	int ss_dim = num_classes;

	SubspaceAnalysis::FaceRecognizer* EFF = new SubspaceAnalysis::FaceRecognizer();
	// calculate Model

	// timeval t1,t2,t3,t4;
	// gettimeofday(&t1,NULL);
	boost::timer t;
	EFF->trainModel(img_vec, label_vec, ss_dim, method, true, false);
	//gettimeofday(&t2,NULL);jj

	//open output file
	std::string path = "/share/goa-tz/people_detection/eval/eval_tool_files/classification_labels";
	std::string probabilities_path = "/share/goa-tz/people_detection/eval/eval_tool_files/classification_probabilities";
	std::string timing_path = "/share/goa-tz/people_detection/eval/eval_tool_files/timing";
	//std::string path = "/home/tom/git/care-o-bot/cob_people_perception/cob_people_detection/debug/eval/eval_tool_files/classified_output";
	std::ofstream os(path.c_str());
	std::ofstream probabilities_os(probabilities_path.c_str());
	std::ofstream timing_os(timing_path.c_str(), std::ofstream::app);

	timing_os << t.elapsed() << ",";
	std::cout << ">>>>>>>>>>training time = " << t.elapsed() << std::endl;

	std::cout << "EFF model computed" << std::endl;
	//EFF->loadModelFromFile("/share/goa-tz/people_detection/debug/rdata.xml",true);


	SubspaceAnalysis::FaceRecognizer* EFF_depth = new SubspaceAnalysis::FaceRecognizer();
	//if(use_xyz)
	//{
	//EFF_depth->trainModel(dm_vec,label_vec,ss_dim,method,true,false);
	//}


	//restart timer
	t.restart();
	for (int i = 0; i < probe_mat_vec.size(); i++)
	{
		cv::Mat probe = probe_mat_vec[i];
		int c_EFF;
		cv::Mat coeff_EFF;
		double DFFS_EFF;
		cv::Mat probabilities;
		if (method == SubspaceAnalysis::METH_LDA2D || method == SubspaceAnalysis::METH_PCA2D)
		{
			std::vector<cv::Mat> coeff_EFF_vec;
			EFF->projectToSubspace2D(probe, coeff_EFF_vec, DFFS_EFF);
			EFF->classify(coeff_EFF_vec[0], classifier, c_EFF, probabilities);
		}
		else
		{
			EFF->projectToSubspace(probe, coeff_EFF, DFFS_EFF);
			EFF->classify(coeff_EFF, classifier, c_EFF, probabilities);
		}

		//c_EFF=model->predict(probe);
		//std::cout<<"RGB CLASS"<<c_EFF<<std::endl;

		//For use with depth data
		int c_EFF_dm;
		cv::Mat coeff_EFF_dm;
		double DFFS_EFF_dm;
		//if(use_xyz)
		//{
		cv::Mat probe_dm = probe_dm_vec[i];
		//EFF_depth->projectToSubspace(probe_dm,coeff_EFF_dm,DFFS_EFF_dm);
		//EFF_depth->classify(coeff_EFF_dm,classifier,c_EFF_dm);
		////std::cout<<"DM CLASS"<<c_EFF_dm<<std::endl;

		//}

		//Output to classified file
		os << c_EFF << "\n";
		probabilities_os << probabilities << "\n";
	}
	std::cout << ">>>>>>>>>>recognition time = " << t.elapsed() << std::endl;
	timing_os << t.elapsed() << std::endl;

	os.close();
	std::cout << "EFF classified" << std::endl;

	cv::Mat m1_evec, m1_eval, m1_avg, m1_pmd;

	//EFF->getModel(m1_evec,m1_eval,m1_avg,m1_pmd);
	//EFF->saveModel("/share/goa-tz/people_detection/debug/test.xml");


	//SubspaceAnalysis::FaceRecognizer* m2=new SubspaceAnalysis::FaceRecognizer();


	//m2->loadModel(m1_evec,m1_eval,m1_avg,m1_pmd,label_vec,false);
	//m2->loadModelFromFile("/share/goa-tz/people_detection/debug/rdata.xml",true);

	//double m2_dffs;
	//cv::Mat m2_coeff;
	//int m2_c;
	//cv::Mat probe=probe_mat_vec[0];
	//m2->projectToSubspace(probe,m2_coeff,m2_dffs);
	//m2->classify(m2_coeff,classifier,m2_c);


	// The following line predicts the label of a given
	// test image:
	//int predictedLabel = model->predict(testSample);


	return 0;
}
