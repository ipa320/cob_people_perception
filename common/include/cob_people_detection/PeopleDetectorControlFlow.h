/// @file PeopleDetectorControlFlow.h
/// Interface to PeopleDetectorControlFlow
/// @author Daniel Seitz, modified Richard Bormann
/// @date Sep, 2009.

#ifndef __PEOPLEDETECTORCONTROLFLOW_H__
#define __PEOPLEDETECTORCONTROLFLOW_H__


#ifdef __LINUX__
	#include "cob_sensor_fusion/CameraSensorsControlFlow.h"
	#include "cob_vision_ipa_utils/MathUtils.h"
	#include "cob_people_detection/PeopleDetector.h"
#else
	#include "cob_vision/cob_sensor_fusion/common/include/cob_sensor_fusion/CameraSensorsControlFlow.h"
	#include "cob_vision/cob_vision_ipa_utils/common/include/cob_vision_ipa_utils/MathUtils.h"
	#include "cob_vision/cob_people_detection/common/include/cob_people_detection/PeopleDetector.h"
#endif

#include <iostream>
#include <cv.h>
#include <string>
#include <vector>
#include <cvaux.h>
#include <sstream>

#include "boost/filesystem/operations.hpp"
#include "boost/filesystem/convenience.hpp"
#include "boost/filesystem/path.hpp"

namespace fs = boost::filesystem;

namespace ipa_PeopleDetector {

/// Long description
class PeopleDetectorControlFlow : public CameraSensorsControlFlow
{
public:
	/// Constructor.
	PeopleDetectorControlFlow(void); ///< Constructor
	~PeopleDetectorControlFlow(void); ///< Destructor

	/// Initialization function.
	/// Creates an instance of a range imaging sensor (i.e. SwissRanger SR-3000) and an instance of 
	/// a color Camera (i.e. Imaging Source camera i.e. DBK 31AF03).
	/// @param directory Directory of camera sensors initialization file.
	/// @param colorCamera0 First color camera instance.
	/// @param colorCamera1 Second color camera instance.
	/// @param rangeImagingSensor Range imaging sensor instance.
	/// @return Return code
	unsigned long Init(std::string directory, 
		ipa_CameraSensors::AbstractColorCameraPtr* colorCamera0 = 0,
		ipa_CameraSensors::AbstractColorCameraPtr* colorCamera1 = 0,
		ipa_CameraSensors::AbstractRangeImagingSensorPtr* rangeImagingSensor = 0);

	/// Function to detect and verify faces
	/// @param pc ColoredPointClowed with images
	/// @return Return code
	unsigned long DetectFaces(ipa_SensorFusion::ColoredPointCloudPtr pc);

	/// Function to add a new face
	/// @param image ColorImage
	/// @param id Id of the new face
	/// @return Return code
	unsigned long AddFace(cv::Mat& image, std::string id);

	/// Function to Run the PCA algorithm and project the training images to the PCA subspace
	/// @return Return code
	unsigned long PCA();

	/// Function to Recognize faces
	/// The function recognize the faces
	/// @param pc ColoredPointClowed with images
	/// @param index Index of classified facespace in vector
	/// @return Return code
	unsigned long RecognizeFace(ipa_SensorFusion::ColoredPointCloudPtr pc, std::vector<int>& index);

	/// Function to save the training data
	/// @return Return code.
	unsigned long SaveTrainingData();

	/// Function to load the training data
	/// @return Return code.
	unsigned long LoadTrainingData();

	/// Function to show the eigenfaces. Only for debugging and development
	/// @param eigenface Eigenface
	/// @param index Index of the eigenface
	/// @return Return code.
	unsigned long GetEigenface(cv::Mat& eigenface, int index);

	/// Function to show the average image of all trained images
	/// @param avgImage The average image
	/// @return Return code.
	unsigned long ShowAVGImage(cv::Mat& avgImage);

	/// Function to extract images for training range classifier
	/// @param pc ColoredPointClowed with images
	/// @return Return code.
	unsigned long SaveRangeTrainImages(ipa_SensorFusion::ColoredPointCloudPtr pc);

	/// Function to load Parameters from Configuration File
	/// @param iniFileName Filename of the Configuration File
	/// @return Return code.
	unsigned long LoadParameters(const char* iniFileName);

	/// Returns the point cloud mode which was loaded from the .xml file
	/// @return point cloud mode
	ColoredPointCloudToolbox::t_PointCloudMode GetPCMode() { return m_PCMode; };

	ipa_SensorFusion::ColoredPointCloudToolbox* m_pcToolbox;

	std::vector<cv::Mat> m_faceImages;			///< Trained face images
	std::vector<std::string> m_id;				///< Id of learned faces
	std::vector<std::string> m_faceClasses_id;	///< Id of face Classes

	int m_nEigens;								///< Number of eigenvalues
	std::vector<cv::Mat> m_eigenVectArr;		///< Eigenvectors
	cv::Mat m_eigenValMat;						///< Eigenvalues
	cv::Mat m_avgImage;							///< Trained average Image
	cv::Mat m_projectedTrainFaceMat;			///< Projected training faces

	PeopleDetector* m_PeopleDetector;
	int m_threshold;							///< Threshold to detect unknown faces
	int m_threshold_FS;							///< Threshold to facespace
	std::vector<cv::Rect> m_colorFaces;			///< Vector with detected faces
	std::vector<cv::Rect> m_rangeFaces;			///< Vector with detected rangeFaces

	int m_OpenGL;								///< 1, if OpenGL viewer is used to vizualise point cloud
	int m_RangeCamIterations;					///< Number of images per range camera shot
												///< One median image is created out of all images
private:
	bool m_runPCA;

	ipa_CameraSensors::AbstractRangeImagingSensor* m_RangeImagingSensor;
	ipa_CameraSensors::AbstractColorCamera* m_ColorCamera0;
	ipa_CameraSensors::AbstractColorCamera* m_ColorCamera1;

	ipa_CameraSensors::CameraSensorToolbox* m_RangeSensorToolbox;	///< Camera toolbox
	ipa_CameraSensors::CameraSensorToolbox* m_ColorSensor0Toolbox;	///< Camera toolbox
	ipa_CameraSensors::CameraSensorToolbox* m_ColorSensor1Toolbox;	///< Camera toolbox

	int m_rangeSensorWidth;
	int m_rangeSensorHeight;
	int m_filname;
};

} // end namespace

#endif // __PEOPLEDETECTORCONTROLFLOW_H__
