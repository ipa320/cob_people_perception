/// @file PeopleDetector.h
/// Interface to PeopleDetector
/// @author Daniel Seitz
/// @date Sep, 2008.

#ifndef __PEOPLEDETECTOR_H__
#define __PEOPLEDETECTOR_H__

#ifdef __LINUX__
	#include "cob_vision_ipa_utils/MathUtils.h"
	#include "cob_sensor_fusion/ColoredPointCloud.h"
#else
	#include "cob_vision/cob_vision_ipa_utils/common/include/cob_vision_ipa_utils/MathUtils.h"
	#include "cob_vision/cob_sensor_fusion/common/include/cob_sensor_fusion/ColoredPointCloud.h"
#endif
#include <fstream>

namespace ipa_PeopleDetector {

/// Interface to Calibrate Head of Care-O-bot 3.
/// Long description
class PeopleDetector
{
public:

	/// Constructor.
	PeopleDetector(void); ///< Constructor
	~PeopleDetector(void); ///< Destructor

	/// Initialization function.
	/// Creates an instance of a range imaging sensor (i.e. SwissRanger SR-3000) and an instance of 
	/// @return Return code
	virtual unsigned long Init();

	/// Function to detect the faces on color image
	/// The function detects the faces in an given image
	/// @param img Image
	/// @param faceCoordinates Vector with the coordinates of detected faces in color image
	/// @return Return code
	virtual unsigned long DetectColorFaces(cv::Mat& img, std::vector<cv::Rect>& faceCoordinates);

	/// Function to detect the face on range image
	/// The function detects the face in an given range image
	/// @param img Image
	/// @param rangeFaceCoordinates Vector with the coordinates of detected heads in range image
	/// @param this parameter should be true if the kinect sensor is used (activates a filling method for black pixels)
	/// @return Return code
	virtual unsigned long DetectRangeFace(cv::Mat& img, std::vector<cv::Rect>& rangeFaceCoordinates, bool fromKinectSensor=false);

	/// Function to detect faces
	/// The function calls internally the functions DetectRangeFace() and DetectColorFaces()
	/// @param img Color image
	/// @param rangeImg Range image
	/// @param faceCoordinates Vector with the coordinates of detected faces on complete color image
	/// @param rangeFaceCoordinates Vector with the coordinates of heads on range image
	/// @param vFaceCoordinates Vector with the coordinates of correct detected faces
	/// @param this parameter should be true if the kinect sensor is used (activates a filling method for black pixels)
	/// @return Return code
	virtual unsigned long DetectFaces(cv::Mat& img, cv::Mat& rangeImg, std::vector<cv::Rect>& colorFaceCoordinates, std::vector<cv::Rect>& rangeFaceCoordinates, bool fromKinectSensor=false);

	/// Function to add a new face
	/// The function adds a new face to the trained images
	/// @param img Image
	/// @param face The face
	/// @param id Id of the new face
	/// @param images Vector with trained images
	/// @param ids Vector with trained ids
	/// @return Return code
	virtual unsigned long AddFace(cv::Mat& img, cv::Rect& face, std::string id, std::vector<cv::Mat>& images, std::vector<std::string>& ids);

	/// Function to Convert and Resizes a given image
	/// The function converts a 8U3 image from camera to an 8U1 image and resizes the face to 100x100 px.
	/// @param img Image from camera
	/// @param resized Resized image from face
	/// @param face The face
	/// @return Return code
	virtual unsigned long ConvertAndResize(cv::Mat& img, cv::Mat& resized, cv::Rect& face);

	/// Function to Run the PCA algorithm
	/// @param nEigens Number of eigenvalues
	/// @param eigenVectors Eigenvectors
	/// @param eigenValMat Eigenvalues
	/// @param avgImage Average image
	/// @param images Trained faces
	/// @param projectedTrainFaceMat Projected training faces (coefficients for the eigenvectors of the face subspace)
	/// @return Return code
	virtual unsigned long PCA(int* nEigens, std::vector<cv::Mat>& eigenVectors, cv::Mat& eigenValMat, cv::Mat& avgImage, std::vector<cv::Mat>& images, cv::Mat& projectedTrainFaceMat);

	/// Function to Recognize faces
	/// The function recognize the faces
	/// @param colorImage source color image
	/// @param faceCoordinates Detected faces
	/// @param nEigens Number of eigenvalues
	/// @param eigenVectArr Eigenvectors
	/// @param avgImage Average image
	/// @param projectedTrainFaceMat Projected training faces
	/// @param index Index of classified face in vector
	/// @param threshold The threshold to recognize unkown faces
	/// @param threshold_FS The threshold to the face space
	/// @param eigenValMat Eigenvalues
	/// @return Return code
	virtual unsigned long RecognizeFace(cv::Mat& colorImage, std::vector<cv::Rect>& colorFaces, int* nEigens, std::vector<cv::Mat>& eigenVectArr, cv::Mat& avgImage, cv::Mat& projectedTrainFaceMat,
																			std::vector<int>& index, int *threshold, int *threshold_FS, cv::Mat& eigenValMat);

	/// Function to find the closest face class
	/// The function calculates the distance of each sample image to the trained face class
	/// @param projectedTestFace The projected test face
	/// @param nearest Index of neares face, or -1 if the face is unknown
	/// @param nEigens Number of eigenvalues
	/// @param projectedTrainFaceMat The average eigenvalues from each face class
	/// @param threshold The threshold to recognize unkown faces
	/// @param eigenValMat Eigenvalues
	/// @return Return code
	virtual unsigned long ClassifyFace(float *projectedTestFace, int *nearest, int *nEigens, cv::Mat& projectedTrainFaceMat, int *threshold, cv::Mat& eigenValMat);

	/// Function to calculate the FaceClasses
	/// The function calculates the face classes.
	/// @param projectedTrainFaceMat The projected training faces
	/// @param id The ids of the training faces
	/// @param nEigens Number of eigenvalues
	/// @return Return code
	virtual unsigned long CalculateFaceClasses(cv::Mat& projectedTrainFaceMat, std::vector<std::string>& id, int *nEigens);

	double m_faces_increase_search_scale;		///< The factor by which the search window is scaled between the subsequent scans
	int m_faces_drop_groups;					///< Minimum number (minus 1) of neighbor rectangles that makes up an object.
	int m_faces_min_search_scale_x;				///< Minimum serach scale x
	int m_faces_min_search_scale_y;				///< Minimum serach scale y

	double m_range_increase_search_scale;		///< The factor by which the search window is scaled between the subsequent scans
	int m_range_drop_groups;					///< Minimum number (minus 1) of neighbor rectangles that makes up an object.
	int m_range_min_search_scale_x;				///< Minimum serach scale x
	int m_range_min_search_scale_y;				///< Minimum serach scale y

private:
	/// interpolates unassigned pixels in the depth image when using the kinect
	/// @param img depth image
	/// @return Return code
	unsigned long InterpolateUnassignedPixels(cv::Mat& img);

	CvMemStorage* m_storage;					///< Storage for face and eye detection
	CvHaarClassifierCascade* m_face_cascade;	///< Haar-Classifier for face-detection
	CvHaarClassifierCascade* m_range_cascade;	///< Haar-Classifier for range-detection
};

} // end namespace

#endif // __PEOPLEDETECTOR_H__
