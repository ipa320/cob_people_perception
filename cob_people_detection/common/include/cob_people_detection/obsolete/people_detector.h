/// @file PeopleDetector.h
/// Interface to PeopleDetector
/// @author Daniel Seitz
/// @date Sep, 2008.

#ifndef __PEOPLEDETECTOR_H__
#define __PEOPLEDETECTOR_H__

#ifdef __LINUX__
	//#include "cob_vision_ipa_utils/MathUtils.h"
	//#include "cob_sensor_fusion/ColoredPointCloud.h"
#else
	#include "cob_vision/cob_vision_ipa_utils/common/include/cob_vision_ipa_utils/MathUtils.h"
	#include "cob_vision/cob_sensor_fusion/common/include/cob_sensor_fusion/ColoredPointCloud.h"	// todo: necessary?
#endif
#include <fstream>
#include <set>
#include <opencv/ml.h>
#include <opencv/cv.h>

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
	/// @param directory The directory for data files
	/// @return Return code
	virtual unsigned long Init(std::string directory);

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
	/// @param fillUnassignedDepthValues this parameter should be true if the kinect sensor is used (activates a filling method for black pixels)
	/// @return Return code
	virtual unsigned long DetectRangeFace(cv::Mat& img, std::vector<cv::Rect>& rangeFaceCoordinates, bool fillUnassignedDepthValues=false);

	/// Function to detect faces
	/// The function calls internally the functions DetectRangeFace() and DetectColorFaces()
	/// @param img Color image
	/// @param rangeImg Range image
	/// @param faceCoordinates Vector with the coordinates of detected faces on complete color image
	/// @param rangeFaceCoordinates Vector with the coordinates of heads on range image
	/// @param colorToRangeFaceDependency stores the indices of range images that contain a face detection in the color image
	/// @param fillUnassignedDepthValues this parameter should be true if the kinect sensor is used (activates a filling method for black pixels)
	/// @return Return code
	virtual unsigned long DetectFaces(cv::Mat& img, cv::Mat& rangeImg, std::vector<cv::Rect>& colorFaceCoordinates, std::vector<cv::Rect>& rangeFaceCoordinates, std::set<size_t>& colorToRangeFaceDependency, bool fillUnassignedDepthValues=false);

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

	/// Applies some preprocessing to the grayscale face images to obtain a more robust identification.
	/// @param input_image Grayscale face image.
	/// @return Preprocessed image.
	virtual cv::Mat preprocessImage(cv::Mat& input_image);

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
	/// @param personClassifier A classifier for person identification. It is trained in this function. Can be left out if a simpler identification method is used.
	/// @return Return code
	virtual unsigned long RecognizeFace(cv::Mat& colorImage, std::vector<cv::Rect>& colorFaces, int* nEigens, std::vector<cv::Mat>& eigenVectArr, cv::Mat& avgImage, cv::Mat& projectedTrainFaceMat,
																			std::vector<int>& index, int *threshold, int *threshold_FS, cv::Mat& eigenValMat, cv::SVM* personClassifier = 0);

	/// Function to find the closest face class
	/// The function calculates the distance of each sample image to the trained face class
	/// @param projectedTestFace The projected test face
	/// @param nearest Index of nearest face, or -1 if the face is unknown
	/// @param nEigens Number of eigenvalues
	/// @param projectedTrainFaceMat The average factors from each face class originating from the eigenvector decomposition
	/// @param threshold The threshold to recognize unkown faces
	/// @param eigenValMat Eigenvalues
	/// @param personClassifier A classifier for person identification. It is trained in this function. Can be left out if a simpler identification method is used.
	/// @return Return code
	virtual unsigned long ClassifyFace(float *projectedTestFace, int *nearest, int *nEigens, cv::Mat& projectedTrainFaceMat, int *threshold, cv::Mat& eigenValMat, cv::SVM* personClassifier = 0);

	/// Function to calculate the FaceClasses
	/// The function calculates the average eigenvector decomposition factors for each face classes.
	/// @param projectedTrainFaceMat The projected training faces
	/// @param id The ids of the training faces
	/// @param nEigens Number of eigenvalues
	/// @param faceClassAvgProjections The average factors of the eigenvector decomposition from each face class
	/// @param idUnique A vector containing all different Ids from the training session exactly once (idUnique[i] stores the corresponding id to the average face coordinates in the face subspace in faceClassAvgProjections.row(i))
	/// @param personClassifier A classifier for person identification. It is trained in this function. Can be left out if a simpler identification method is used.
	/// @return Return code
	virtual unsigned long CalculateFaceClasses(cv::Mat& projectedTrainFaceMat, std::vector<std::string>& id, int *nEigens, cv::Mat& faceClassAvgProjections, std::vector<std::string>& idUnique, cv::SVM* personClassifier = 0);

	double m_faces_increase_search_scale;		///< The factor by which the search window is scaled between the subsequent scans
	int m_faces_drop_groups;					///< Minimum number (minus 1) of neighbor rectangles that makes up an object.
	int m_faces_min_search_scale_x;				///< Minimum search scale x
	int m_faces_min_search_scale_y;				///< Minimum search scale y

	double m_range_increase_search_scale;		///< The factor by which the search window is scaled between the subsequent scans
	int m_range_drop_groups;					///< Minimum number (minus 1) of neighbor rectangles that makes up an object.
	int m_range_min_search_scale_x;				///< Minimum search scale x
	int m_range_min_search_scale_y;				///< Minimum search scale y

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
