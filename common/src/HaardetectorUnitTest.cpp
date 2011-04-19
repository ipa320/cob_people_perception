/// @file HaardetectorUnitTest
/// Unit test for a trained haard detector. Outputs detection performance.
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
	return (x<0) ? -x : x;
}

/// Checks whether two rectangles are located at approximately the same position.
/// E.g. for comparisons between a ground truth localization and the measurement.
/// @param gt Ground truth rectangle.
/// @param measurement The measured rectangle or any rectangle for comparison.
/// @param manhattanDistanceThreshold The maximum Manhattan distance of the corners of both rectangles which still renders them similar.
/// @return Returns true if both rectangles are similar, else it returns false.
bool checkRectangleCorrespondence(cv::Rect& gt, cv::Rect& measurement, double manhattanDistanceThreshold)
{
	if ((abs_(gt.x-measurement.x)<manhattanDistanceThreshold) && (abs_(gt.y-measurement.y)<manhattanDistanceThreshold) &&
		(abs_(gt.x+gt.width-(measurement.x+measurement.width))<manhattanDistanceThreshold) && (abs_(gt.y+gt.height-(measurement.y+measurement.height))<manhattanDistanceThreshold))
		return true;
	
	return false;
}


int main(int argc, char *argv[])
{
	if (argc != 4)
	{
		std::cout << "Usage of this tool:\nHaardetectorUnitTest <filename> <detector_mode> <detector_path>\n filename\t=\tName of the file with the ground truth (each line contains the filename of the respective image followed by the number of contained faces followed by the coordinates of these faces in the style x y width height, use spaces as separator)\n";
		std::cout << " detector_mode\t=\tswitches between range image detector (put a 0 as argument) and color image detector (put a 1 as argument)\n\n";
		std::cout << " detector_path\t=\tthe path to the range and color image haarcascades (the cacscades should be placed in a folder haarcascades/ , provide the path to that folder)\n\n";
		std::cout << "Example: HarrdetectorUnitTest groundtruth.txt 0 ConfigurationFiles/\n\n";
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
	if(peopleDetectorControlFlow.LoadParameters(iniFileNameAndPath.c_str()) & ipa_CameraSensors::RET_FAILED)
	{
		std::cerr << "ERROR - HaardetectorUnitTest:" << std::endl;
		std::cerr << "\t ... Error while loading configuration file '" << std::endl;
		std::cerr << "\t ... " << iniFileNameAndPath << "'.\n";
		return ipa_Utils::RET_FAILED;	
	}

	// file containing the ground truth
	std::string filename = argv[1];

	// open the ground truth file
	std::fstream fin(filename.c_str(), std::fstream::in);
	if (!fin.is_open())
	{
		std::cerr << "ERROR - HaardetectorUnitTest:\n";
		std::cerr << "\t ... Could not open the ground truth file " << filename << ".\n";
		return ipa_Utils::RET_FAILED;
	}

	// statistics
	enum {TRUE_POS=0, FALSE_NEG, FALSE_POS, TRUE_NEG};
	std::vector<int> stats(4, 0);
	int numberNegativeImages = 0;	// number of negative images (without any face) in the test set
	int numberGroundtruthFaces = 0;	// total number of faces in the whole test set

	// open each image of the ground truth, run the detector and examine the detection results
	while (!fin.eof())
	{
		// filename of the image, load image
		std::string imageFilename;
		fin >> imageFilename;
		cv::Mat img = cv::imread(imageFilename, -1);

		// maximum manhattan distance of the corners of ground truth and detected rectangles
		const double maxFaceDistance = (double)img.cols/75.0;

		// number of contained faces
		int numFaces = 0;
		fin >> numFaces;
		numberGroundtruthFaces += numFaces;
		if (numFaces==0) numberNegativeImages++;

		// detect faces
		std::vector<cv::Rect> faces;
		if (detectorMode==0)
			peopleDetector.DetectRangeFace(img, faces, false);	// range images
		else
			peopleDetector.DetectColorFaces(img, faces);	// color images

		// check for each ground truth face whether it was found by the detector
		std::vector<bool> truePositiveDetection(faces.size(), false);	// stores when a detected face turns out to be a true positive
		for (int gtfaceIndex=0; gtfaceIndex<numFaces; gtfaceIndex++)
		{
			// read the ground truth faces from file
			cv::Rect faceGt;
			fin >> faceGt.x >> faceGt.y >> faceGt.width >> faceGt.height;

#ifdef __DEBUG__
			cv::rectangle(img, faceGt, CV_RGB(0,255,0), 2);
#endif

			// compare with each detected face
			bool truePositiveFound = false;
			for (int faceIndex=0; faceIndex<(int)faces.size(); faceIndex++)
			{
				if (checkRectangleCorrespondence(faceGt, faces[faceIndex], maxFaceDistance) == true)
				{
					// both rectangles correspond
					if (truePositiveFound == false) stats[TRUE_POS]++;
					truePositiveDetection[faceIndex] = true;
					truePositiveFound = true;
				}
			}

			// complete statistics - could the ground truth be detected in the image
			if (truePositiveFound == false)
				stats[FALSE_NEG]++;
		}

		// complete statistics - check for false positives
		for (int faceIndex=0; faceIndex<(int)faces.size(); faceIndex++)
			if (truePositiveDetection[faceIndex]==false)
				stats[FALSE_POS]++;

		// complete statistics - check for true negative (no face in ground truth and no detections)
		if (numFaces==0 && faces.size()==0)
			stats[TRUE_NEG]++;

#ifdef __DEBUG__
		for (int faceIndex=0; faceIndex<(int)faces.size(); faceIndex++)
			cv::rectangle(img, faces[faceIndex], CV_RGB(0,0,255), 2);
		cv::imshow("Groundtruth (green) vs detections (blue)", img);
		cv::waitKey();
#endif
	}
	fin.close();

	// output results
	std::cout << "\n-----------\nStatistics:\n-----------\nTruePos\tFalseNeg\tFalsePos\tTrueNeg\t\tNumFaces\tNumNegatives\n";
	std::cout << stats[TRUE_POS] << "\t" << stats[FALSE_NEG] << "\t" << stats[FALSE_POS] << "\t" << stats[TRUE_NEG] << "\t\t" << numberGroundtruthFaces << "\t" << numberNegativeImages << "\n";
	std::cout << "\nPositive detection rate (#truePositives/#NumFaces): " << (double)stats[TRUE_POS]/(double)numberGroundtruthFaces << "\n";
	std::cout << "\nFalse positive rate (#falsePositives/#NumFaces): " << (double)stats[FALSE_POS]/(double)numberGroundtruthFaces << "\n";
	std::cout << "\nNegative detection rate (#trueNegatives/#NumNegatives): " << (double)stats[TRUE_NEG]/(double)numberNegativeImages << "\n";
	getchar();

	return ipa_Utils::RET_OK;
}