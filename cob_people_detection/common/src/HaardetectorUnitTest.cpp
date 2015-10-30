/// @file HaardetectorUnitTest
/// Unit test for a trained haard detector. Outputs detection performance on the test set.
/// In Debug mode, each image is displayed and the threshold settings from the provided ini-file apply. In Release mode, the threshold is tested over a range of paramters to obtain the best setting.
/// @author Richard Bormann
/// @date 04/2011

#ifdef __LINUX__
#include "cob_people_detection/PeopleDetector.h"
#include "cob_people_detection/PeopleDetectorControlFlow.h"
#else
#include "cob_vision/cob_people_detection/common/include/cob_people_detection/PeopleDetector.h"
#include "cob_vision/cob_people_detection/common/include/cob_people_detection/PeopleDetectorControlFlow.h"
#endif

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>

template<typename T> inline T abs_(T x)
{
	return (x < 0) ? -x : x;
}

/// Checks whether two rectangles are located at approximately the same position.
/// E.g. for comparisons between a ground truth localization and the measurement.
/// @param gt Ground truth rectangle.
/// @param measurement The measured rectangle or any rectangle for comparison.
/// @param areaRatioThreshold The ratio of overlapping area to ground truth area or to detected area must not fall below this threshold otherwise it is no detection.
/// @return Returns true if both rectangles are similar, else it returns false.
bool checkRectangleCorrespondence(cv::Rect& gt, cv::Rect& measurement, double areaRatioThreshold)
{
	// overlapping area
	double dx = (min(gt.x + gt.width, measurement.x + measurement.width) - max(gt.x, measurement.x));
	double dy = (min(gt.y + gt.height, measurement.y + measurement.height) - max(gt.y, measurement.y));
	double overlapArea = dx * dy;

	// check area ratios
	double gtArea = gt.width * gt.height;
	double measurementArea = measurement.width * measurement.height;
	//std::cout << overlapArea/gtArea << "   " << overlapArea/measurementArea << "\n";

	if (dx < 0 || dy < 0 || (overlapArea / gtArea < areaRatioThreshold) || (overlapArea / measurementArea < areaRatioThreshold))
		return false;

	return true;
}

int main(int argc, char *argv[])
{
	if (argc != 4)
	{
		std::cout << "Usage of this tool:\nHaardetectorUnitTest <test_data_path> <detector_mode> <detector_path>\n";
		std::cout
				<< " test_data_path\t=\tPath to the 'testData.txt' file with the ground truth (each line contains the filename of the respective image followed by the number of contained faces followed by the coordinates of these faces in the style x y width height, use spaces as separator)\n";
		std::cout << " detector_mode\t=\tswitches between range image detector (put a 0 as argument) and color image detector (put a 1 as argument)\n\n";
		std::cout
				<< " detector_path\t=\tthe path to the range and color image haarcascades (the cacscades should be placed in a folder haarcascades/ , provide the path to that folder)\n\n";
		std::cout << "Example: HarrdetectorUnitTest ConfigurationFiles/TestData/ 0 ConfigurationFiles/\n\n";
		return ipa_Utils::RET_OK;
	}

	// detector mode (0=range, 1=color images)
	std::stringstream ss;
	ss << argv[2];
	int detectorMode;
	ss >> detectorMode;

	// initialize the people detector
	ipa_PeopleDetector::PeopleDetectorControlFlow peopleDetectorControlFlow;
	ipa_PeopleDetector::PeopleDetector peopleDetector;
	std::string directory = argv[3];
	std::string iniFileNameAndPath = directory + "peopleDetectorIni.xml";
	if (peopleDetector.Init(directory) & ipa_Utils::RET_FAILED)
	{
		std::cerr << "ERROR - HaardetectorUnitTest:" << std::endl;
		std::cerr << "\t ... Could not initialize people detector library.\n";
		return ipa_Utils::RET_FAILED;
	}
	peopleDetectorControlFlow.m_PeopleDetector = &peopleDetector;
	if (peopleDetectorControlFlow.LoadParameters(iniFileNameAndPath.c_str()) & ipa_CameraSensors::RET_FAILED)
	{
		std::cerr << "ERROR - HaardetectorUnitTest:" << std::endl;
		std::cerr << "\t ... Error while loading configuration file '" << std::endl;
		std::cerr << "\t ... " << iniFileNameAndPath << "'.\n";
		return ipa_Utils::RET_FAILED;
	}

	// try different parameter settings
	peopleDetector.m_range_drop_groups;
#ifndef __DEBUG__
	int* parameter = 0;
	if (detectorMode == 0)
		parameter = &(peopleDetector.m_range_drop_groups); // range mode
	else
		parameter = &(peopleDetector.m_faces_drop_groups); // color mode
	for (*parameter = 20; *parameter < 80; *parameter += 2)
	{
		if (detectorMode == 0)
			std::cout << "\n\n\n---------------------------------------------------\nparam peopleDetector.m_range_drop_groups = " << peopleDetector.m_range_drop_groups << "\n";
		else
			std::cout << "\n\n\n---------------------------------------------------\nparam peopleDetector.m_faces_drop_groups = " << peopleDetector.m_faces_drop_groups << "\n";
#endif

		// file containing the ground truth
		std::string testDataDirectory = argv[1];
		std::string filename = testDataDirectory + "testData.txt";

		// open the ground truth file
		std::fstream fin(filename.c_str(), std::fstream::in);
		if (!fin.is_open())
		{
			std::cerr << "ERROR - HaardetectorUnitTest:\n";
			std::cerr << "\t ... Could not open the ground truth file " << filename << ".\n";
			return ipa_Utils::RET_FAILED;
		}

		// statistics
		enum
		{
			TRUE_POS = 0, FALSE_NEG, FALSE_POS, TRUE_NEG
		};
		std::vector<int> stats(4, 0);
		int numberNegativeImages = 0; // number of negative images (without any face) in the test set
		int numberGroundtruthFaces = 0; // total number of faces in the whole test set

		// open each image of the ground truth, run the detector and examine the detection results
		while (!fin.eof())
		{
			// filename of the image, load image
			std::string imageFilename;
			fin >> imageFilename;
			if (imageFilename == "")
				break;
			imageFilename = testDataDirectory + imageFilename;
			cv::Mat img = cv::imread(imageFilename, -1);

			//for (int v=0; v<img.rows; v++)
			//{
			//	for (int u=0; u<img.cols; u++)
			//	{
			//		if (img.at<uchar>(v,u) == 0) img.at<uchar>(v,u) = 255;
			//	}
			//}

#ifdef __DEBUG__
			cv::Mat dispImg;
			cv::cvtColor(img, dispImg, CV_GRAY2BGR, 3);
#endif

			// the ratio of overlapping area to ground truth area or to detected area must not fall below this threshold otherwise it is no detection
			const double maxFaceDistance = 0.6;

			// number of contained faces
			int numFaces = 0;
			fin >> numFaces;
			numberGroundtruthFaces += numFaces;
			if (numFaces == 0)
				numberNegativeImages++;

			// detect faces
			std::vector<cv::Rect> faces;
			if (detectorMode == 0)
				peopleDetector.DetectRangeFace(img, faces, false); // range images
			else
				peopleDetector.DetectColorFaces(img, faces); // color images

			// check for each ground truth face whether it was found by the detector
			std::vector<bool> truePositiveDetection(faces.size(), false); // stores when a detected face turns out to be a true positive
			for (int gtfaceIndex = 0; gtfaceIndex < numFaces; gtfaceIndex++)
			{
				// read the ground truth faces from file
				cv::Rect faceGt;
				fin >> faceGt.x >> faceGt.y >> faceGt.width >> faceGt.height;

#ifdef __DEBUG__
				cv::rectangle(dispImg, faceGt, CV_RGB(0,255,0), 2);
#endif

				// compare with each detected face
				bool truePositiveFound = false;
				for (int faceIndex = 0; faceIndex < (int)faces.size(); faceIndex++)
				{
					if (checkRectangleCorrespondence(faceGt, faces[faceIndex], maxFaceDistance) == true)
					{
						// both rectangles correspond
						if (truePositiveFound == false)
							stats[TRUE_POS]++;
						truePositiveDetection[faceIndex] = true;
						truePositiveFound = true;
					}
				}

				// complete statistics - could the ground truth be detected in the image
				if (truePositiveFound == false)
					stats[FALSE_NEG]++;
			}

			// complete statistics - check for false positives
			for (int faceIndex = 0; faceIndex < (int)faces.size(); faceIndex++)
				if (truePositiveDetection[faceIndex] == false)
					stats[FALSE_POS]++;

			// complete statistics - check for true negative (no face in ground truth and no detections)
			if (numFaces == 0 && faces.size() == 0)
				stats[TRUE_NEG]++;

#ifdef __DEBUG__
			std::cout << stats[TRUE_POS] << "\t" << stats[FALSE_NEG] << "\t" << stats[FALSE_POS] << "\t" << stats[TRUE_NEG] << "\t\t" << numberGroundtruthFaces << "\t" << numberNegativeImages << "\n";
			for (int faceIndex=0; faceIndex<(int)faces.size(); faceIndex++)
			cv::rectangle(dispImg, faces[faceIndex], CV_RGB(0,0,255), 2);
			cv::imshow("Groundtruth (green) vs detections (blue)", dispImg);
			cv::waitKey();
#endif
		}
		fin.close();

		// output results
		std::cout << "-----------\nStatistics:\n-----------\nTruePos\tFalseNeg\tFalsePos\tTrueNeg\t\tNumFaces\tNumNegatives\n";
		std::cout << stats[TRUE_POS] << "\t" << stats[FALSE_NEG] << "\t\t" << stats[FALSE_POS] << "\t\t" << stats[TRUE_NEG] << "\t\t" << numberGroundtruthFaces << "\t\t"
				<< numberNegativeImages << "\n";
		std::cout << "\nPositive detection rate (#truePositives/#NumFaces): " << (double)stats[TRUE_POS] / (double)numberGroundtruthFaces << "\n";
		std::cout << "\nFalse positive rate (#falsePositives/#NumFaces): " << (double)stats[FALSE_POS] / (double)numberGroundtruthFaces << "\n";
		std::cout << "\nNegative detection rate (#trueNegatives/#NumNegatives): " << (double)stats[TRUE_NEG] / (double)numberNegativeImages << "\n";
#ifndef __DEBUG__
	}
#endif

	getchar();

	return ipa_Utils::RET_OK;
}
