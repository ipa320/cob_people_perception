/// @file CuiPeopleDetector.cpp
/// Implements GUI-Options
/// @author Daniel Seitz, modified Richard Bormann
/// @date Sep, 2009.


#ifdef __LINUX__
	#include "cob_people_detection/CuiPeopleDetector.h"
#else
	#include "cob_vision/cob_people_detection/common/include/cob_people_detection/CuiPeopleDetector.h"
#endif


using namespace ipa_PeopleDetector;


CuiPeopleDetector::CuiPeopleDetector()
{
	m_DetectorControlFlow = new PeopleDetectorControlFlow();
}

CuiPeopleDetector::~CuiPeopleDetector()
{
	delete m_DetectorControlFlow;
}

unsigned long CuiPeopleDetector::Init()
{
	if (m_DetectorControlFlow->Init("common/files/windows/") & ipa_Utils::RET_FAILED)
	{
		std::cerr << "ERROR - CuiPeopleDetector::Init:" << std::endl;
		std::cerr << "\t ... Error while initializing control flow\n";
		return ipa_Utils::RET_FAILED;
	}
	return ipa_Utils::RET_OK;
}

unsigned long CuiPeopleDetector::ConsoleGUI()
{
	// print out menu options
	std::cout << "INFO - PeopleDetector::ConsoleGUI:" << std::endl;
	std::cout << "\t ... Select one of the following options:" << std::endl << std::endl;

	std::cout << "-------------------- People Detector functions ---------------------" << std::endl;
	std::cout << std::endl;
	std::cout << "t: Train new Person (Faces)" << std::endl;
	std::cout << "r: Recognize Persons" << std::endl;
	std::cout << "e: Show Eigenfaces" << std::endl;
	std::cout << "a: Show AVG image" << std::endl;
	std::cout << "s: Save training data" << std::endl;
	std::cout << "l: Load training data" << std::endl;

	std::cout << std::endl << std::endl;
	std::cout << "q: Quit" << std::endl;

	std::string cmd;
	std::cin >> cmd;

	// Train
	if(strcmp("t", cmd.c_str())==0) return Train();

	// Recognize
	if(strcmp("r", cmd.c_str())==0) return Recognize();

	// Save
	if(strcmp("s", cmd.c_str())==0) return m_DetectorControlFlow->SaveTrainingData();

	// Load
	if(strcmp("l", cmd.c_str())==0) return m_DetectorControlFlow->LoadTrainingData();

	// Eigenfaces
	if(strcmp("e", cmd.c_str())==0) return ShowEigenfaces();

	// AVGImage
	if(strcmp("a", cmd.c_str())==0) return ShowAvgImage();

	// Exit
	if(strcmp("q", cmd.c_str())==0) return ipa_Utils::RET_FAILED;

	return ipa_Utils::RET_OK;
};

unsigned long CuiPeopleDetector::Train()
{
	int cmd;
	
	cv::Mat colorImage_8U3;
	std::string id;						// Id of trained face

	std::cout << "Please enter your name: ";
	std::cin >> id;

	std::cout << "INFO - PeopleDetector::ConsoleGUI:" << std::endl;
	std::cout << "\t ... Please press 'n' to acquire the next image" << std::endl;
	std::cout << "\t ... or press 's' to save the image" << std::endl;
	std::cout << "\t ... or press 'q' to quit!" << std::endl;

	cv::namedWindow("Face Detector");

	ipa_SensorFusion::ColoredPointCloudPtr pc;

	int count=0;

	while(true)
	{
		cmd = cvWaitKey();

		// acquire next image
		if(cmd == 'n')
		{
			if (m_DetectorControlFlow->GetColoredPointCloud(pc, ColoredPointCloudToolbox::MODE_SHARED, 1) & ipa_Utils::RET_FAILED)
			{
				std::cerr << "ERROR - CuiPeopleDetector::Train:" << std::endl;
				std::cerr << "\t ... Could not get Colored Point Cloud" << std::endl;
				return ipa_Utils::RET_FAILED;
			}
			
			//DWORD start = timeGetTime(); 
			m_DetectorControlFlow->DetectFaces(pc);
			std::cout << "INFO - CuiPeopleDetector::Train:" << std::endl;
			//std::cout << "\t ... Detection Time: " << (timeGetTime() - start) << std::endl;
			count++;

			(pc->GetColorImage()).copyTo(colorImage_8U3);

			for(int i=0; i<(int)m_DetectorControlFlow->m_colorFaces.size(); i++)
			{
				cv::Rect face = m_DetectorControlFlow->m_colorFaces[i];
				cv::rectangle(colorImage_8U3, cv::Point(face.x, face.y), cv::Point(face.x + face.width, face.y + face.height), CV_RGB( 0, 255, 0 ), 2, 8, 0);
			}

			cv::imshow("Face Detector", colorImage_8U3);
		}

		// save image
		if(cmd == 's')
		{
			// Check if there is more than one face detected
			if(m_DetectorControlFlow->m_colorFaces.size() > 1)
			{
				std::cout << "WARNING - CuiPeopleDetector::ConsoleGUI:" << std::endl;
				std::cout << "\t ... More than one faces are detected in image. Please try again." << std::endl;
				continue;
			}

			// Check if there is less than one face detected
			if(m_DetectorControlFlow->m_colorFaces.size() < 1)
			{
				std::cout << "WARNING - CuiPeopleDetector::ConsoleGUI:" << std::endl;
				std::cout << "\t ... Less than one faces are detected in image. Please try again." << std::endl;
				continue;
			}

			m_DetectorControlFlow->AddFace(pc->GetColorImage(), id);
			std::cout << "INFO - CuiPeopleDetector::ConsoleGUI:" << std::endl;
			std::cout << "\t ... Face saved. (" << m_DetectorControlFlow->m_faceImages.size() << ")" << std::endl;
		}

		// quit
		if(cmd == 'q')
		{
			// Exit
			break;
		}
	}
	
	cvDestroyAllWindows();

	return ipa_Utils::RET_OK;
}

unsigned long CuiPeopleDetector::Recognize()
{
	int cmd;
	cv::Mat colorImage_8U3;

	std::cout << "INFO - PeopleDetector::ConsoleGUI:" << std::endl;
	std::cout << "\t ... Please press 'n' to acquire the next image" << std::endl;
	std::cout << "\t ... or press 'q' to quit!" << std::endl;
	
	m_DetectorControlFlow->PCA();

	if(m_DetectorControlFlow->m_faceImages.size() < 2)
	{
		std::cout << "WARNING - PeopleDetector::ConsoleGUI:" << std::endl;
		std::cout << "\t ... Less than two images are trained.\n";
		return ipa_Utils::RET_OK;
	}

	cv::namedWindow("Face Detector");

	while(true)
	{
		cmd = cv::waitKey();
		if(cmd == 'n')
		{
			ipa_SensorFusion::ColoredPointCloudPtr pc;

			if (m_DetectorControlFlow->GetColoredPointCloud(pc, ColoredPointCloudToolbox::MODE_SHARED, 1) & ipa_Utils::RET_FAILED)
			{
				std::cerr << "ERROR - CuiPeopleDetector::Recognize:" << std::endl;
				std::cerr << "\t ... Could not get Colored Point Cloud" << std::endl;
				return ipa_Utils::RET_FAILED;
			}

			//DWORD start = timeGetTime(); 
			
			m_DetectorControlFlow->DetectFaces(pc);
			
			(pc->GetColorImage()).copyTo(colorImage_8U3);

			std::vector<int> index;
			m_DetectorControlFlow->RecognizeFace(pc, index);
			std::cout << "INFO - CuiPeopleDetector::Recognize:" << std::endl;
			//std::cout << "\t ... Recognize Time: " << (timeGetTime() - start) << std::endl;

			for(int i=0; i<(int)m_DetectorControlFlow->m_colorFaces.size(); i++)
			{
				cv::Rect face = m_DetectorControlFlow->m_colorFaces[i];

				cv::rectangle(colorImage_8U3, cv::Point(face.x, face.y), cv::Point(face.x + face.width, face.y + face.height), CV_RGB(0, 255, 0), 2, 8, 0);
				
				std::stringstream tmp;
				switch(index[i])
				{
				case -1:
					// Distance to face class is too high
					tmp << "Unknown";
					cv::putText(colorImage_8U3, tmp.str().c_str(), cv::Point(face.x,face.y+face.height+25), cv::FONT_HERSHEY_PLAIN, 2, CV_RGB( 255, 0, 0 ), 2);
					break;
				case -2:
					// Distance to face space is too high
					tmp << "No face";
					cv::putText(colorImage_8U3, tmp.str().c_str(), cv::Point(face.x,face.y+face.height+25), cv::FONT_HERSHEY_PLAIN, 2, CV_RGB( 255, 0, 0 ), 2);
					break;
				default:
					// Face classified
					tmp << m_DetectorControlFlow->m_id[index[i]].c_str();
					cv::putText(colorImage_8U3, tmp.str().c_str(), cv::Point(face.x,face.y+face.height+25), cv::FONT_HERSHEY_PLAIN, 2, CV_RGB( 0, 255, 0 ), 2);
				}
			}

			cv::imshow("Face Detector", colorImage_8U3);
		}
		
		if(cmd == 'q')
		{
			// Exit
			break;
		}
	}

	// Clear
	cvDestroyAllWindows();

	return ipa_Utils::RET_OK;
}

unsigned long CuiPeopleDetector::ShowEigenfaces()
{
	cv::Mat eigenface;
	
	cv::namedWindow("Eigenfaces");

	m_DetectorControlFlow->PCA();

	for(int i=0; i<(int)m_DetectorControlFlow->m_faceImages.size()-1; i++)
	{
		m_DetectorControlFlow->GetEigenface(eigenface, i);
		cv::imshow("Eigenfaces", eigenface);
	
		int cmd = cv::waitKey();
		if(cmd == 'q')
		{
			// Exit
			cvDestroyAllWindows();
			break;
		}
	}

	cvDestroyAllWindows();
	
	return ipa_Utils::RET_OK;
}
unsigned long CuiPeopleDetector::ShowAvgImage()
{
	cv::Mat avgImage(100, 100, CV_8UC1);

	m_DetectorControlFlow->PCA();

	m_DetectorControlFlow->ShowAVGImage(avgImage);

	cv::namedWindow("AVGImage");
	imshow("AVGImage", avgImage);
	cv::waitKey();

	cvDestroyAllWindows();

	return ipa_Utils::RET_OK;
}