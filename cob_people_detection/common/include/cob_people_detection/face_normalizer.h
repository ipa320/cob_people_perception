#include <opencv/cv.h>
#include <opencv/cvaux.h>
#include <opencv/highgui.h>
#include <opencv/ml.h>

#include <iostream>
#include <boost/lexical_cast.hpp>

using namespace cv;

namespace FACE
{
enum FEATURE_TYPE
{
	LEFTEYE, RIGHTEYE, NOSE, MOUTH,
};

/// Class describes a set of facial features consisting of eyes, nose and mouth.
template<class T>
class FaceFeatures
{
public:

	/// Construcor for face features
	FaceFeatures<T>()
	{
	}
	;

	/// Destrucor for face features
	~FaceFeatures<T>()
	{
	}
	;

	T lefteye; ///< Coordinates of the left eye
	T righteye; ///< Coordinates of the right eye
	T nose; ///< Coordinates of the nose
	T mouth; ///< Coordinates of the mouth

	/// @brief Subtract offset from all features
	/// @param[in] ox Offset in x-direction
	/// @param[in] oy Offset in y-direction
	void sub_offset(int& ox, int& oy)
	{
		this->lefteye.x -= ox;
		this->lefteye.y -= oy;
		this->righteye.x -= ox;
		this->righteye.y -= oy;
		this->mouth.x -= ox;
		this->mouth.y -= oy;
		this->nose.x -= ox;
		this->nose.y -= oy;
	}

	/// @brief Add offset from all features
	/// @param[in] ox Offset in x-direction
	/// @param[in] oy Offset in y-direction
	void add_offset(int& ox, int& oy)
	{
		this->lefteye.x += ox;
		this->lefteye.y += oy;
		this->righteye.x += ox;
		this->righteye.y += oy;
		this->mouth.x += ox;
		this->mouth.y += oy;
		this->nose.x += ox;
		this->nose.y += oy;
	}

	/// @brief Scale set of features
	/// @param[in] scale Scale which is applied.
	void scale(double scale)
	{
		lefteye *= scale;
		righteye *= scale;
		nose *= scale;
		mouth *= scale;
	}

	/// @brief Create vector containing set of features
	// @return Vector containing lefteye,righteye,nose,mouth
	std::vector<T> as_vector()
	{
		std::vector<T> vec;
		vec.push_back(lefteye);
		vec.push_back(righteye);
		vec.push_back(nose);
		vec.push_back(mouth);
		return vec;
	}
	;

	/// @brief Check whether coordinates of features are valid numbers.
	bool valid()
	{
		if (std::isnan(lefteye.x) || std::isnan(lefteye.y))
			return false;
		if (std::isnan(righteye.x) || std::isnan(righteye.y))
			return false;
		if (std::isnan(nose.x) || std::isnan(nose.y))
			return false;
		//if(std::isnan(mouth.x)|| std::isnan(mouth.y)) return false;
		else
			return true;
	}
};

}
;

class FaceNormalizer
{

public:
	/// Configuration struct for the face normalizer.
	/// It determines the behavior of the normalization algorithm.
	/// It can be set during the initialization process.
	/// @brief Face normalizer configuration struct.
	/// @see init()
	struct FNConfig
	{
		bool eq_ill; ///< If flag is true, illumination normalization is performed.
		bool align; ///< If flag is true, geometrical alignment is performed.
		bool resize; ///< If flag is true, image is scaled to norm size.
		bool cvt2gray; ///< If flag is true, color images are converted to grayscale.
		bool extreme_illumination_condtions; ///< If flag is true, GammaDOG normalization is used instead of GammaDCT.
	};

	/// @brief Constructor for face normalizer.
	FaceNormalizer()
	{
	}
	;

	/// @brief Destructor for face normalizer.
	~FaceNormalizer();

	/// The function inititializes the face normalizer with default values.
	/// @brief Default initialization
	void init();

	/// The function initializes the face normalizer with given config.
	/// @brief Initialization with given config.
	/// @param[in] i_config Configuration for face normalizer.
	void init(FNConfig& i_config);

	/// The function initializes the face normalizer with directory for haar cascades and  given config.
	/// @brief Initialization with given config and path for haar classifier cascades.
	/// @param[in] i_classifier_directory Directory where haar classifier cascades are stored.
	/// @param[in] i_config Configuration for face normalizer.
	void init(std::string i_classifier_directory, FNConfig& i_config);

	/// The function initializes the face normalizer with all option.
	/// @brief Initialization with full control of behavior.
	/// @param[in] i_classifier_directory Directory where haar classifier cascades are stored.
	/// @param[in] i_storage_directory Directory where debug output is saved.
	/// @param[in] i_config Configuration for face normalizer.
	/// @param[in] i_epoch_ctr Starting number for input frames.
	/// @param[in] i_debug Flag whether debug output is printed.
	/// @param[in] i_record_scene Flag whether scene is saved automatically.
	void init(std::string i_classifier_directory, std::string i_storage_directory, FNConfig& i_config, int i_epoch_ctr, bool i_debug, bool i_record_scene);

	/// The function normalizes given color image with respect to illumination and size.
	/// @brief Function to normalize a color image
	/// @param[in,out] RGB Color image that is normalized.
	/// @param[in] norm_size Size the input image is scaled to.
	/// @return Return true/false whether normalization was successful.
	bool normalizeFace(cv::Mat & RGB, cv::Size& norm_size);

	/// The function normalizes given color image with the corresponding point cloud respect to illumination and size and recording perspective
	/// @brief Function to obtain normalized  color image and depth map 
	/// @param[in,out] RGB Color image that is normalized
	/// @param[in] XYZ Pointcloud corresponding to color image
	/// @param[in] norm_size Size the input image is scaled to
	/// @param[out] DM Depth map generated from normalized point cloud
	/// @return Return true/false whether normalization was successful
	bool normalizeFace(cv::Mat& RGB, cv::Mat& XYZ, cv::Size& norm_size, cv::Mat& DM);

	/// The function normalizes given color image with the corresponding point cloud respect to illumination and size and recording perspective.
	/// @brief Function to normalize a color image and point cloud.
	/// @param[in,out] RGB Color image that is normalized.
	/// @param[in] XYZ Pointcloud corresponding to color image.
	/// @param[in] norm_size Size the input image is scaled to.
	/// @return Return true/false whether normalization was successful.
	bool normalizeFace(cv::Mat & RGB, cv::Mat& XYZ, cv::Size& norm_size);

	//TODO documentation
	/// Function to synthetisize artificial poses from one image
	bool synthFace(cv::Mat &RGB, cv::Mat& XYZ, cv::Size& norm_size, std::vector<cv::Mat>& synth_images);
	bool synth_head_poses(cv::Mat& img, cv::Mat& depth, std::vector<cv::Mat>& synth_images);
	bool synth_head_poses_relative(cv::Mat& img, cv::Mat& depth, std::vector<cv::Mat>& synth_images);
	bool eliminate_background(cv::Mat& RGB, cv::Mat& XYZ, float background_thresh);
	bool isolateFace(cv::Mat& RGB, cv::Mat& XYZ);
	bool interpolate_head(cv::Mat& RGB, cv::Mat& XYZ);
	bool recordFace(cv::Mat&RGB, cv::Mat& XYZ);

	/// The function saves scene, consisting of color image and corresponding point cloud to given path.
	/// @brief Function to save scene.
	/// @param[in] RGB Color image that is normalized.
	/// @param[in] XYZ Pointcloud corresponding to color image.
	/// @return Return true/false whether saving was successful.
	bool save_scene(cv::Mat& RGB, cv::Mat& XYZ, std::string path);

	/// The function reads scene, consisting of color image and corresponding point cloud from given path.
	/// @brief Function to read scene.
	/// @param[out] RGB Color image that is normalized.
	/// @param[out] XYZ Pointcloud corresponding to color image.
	/// @return Return true/false whether reading was successful.
	bool read_scene(cv::Mat& RGB, cv::Mat& XYZ, std::string path);

	FNConfig config_; ///< Configuration of face normalizer
protected:

	/// The function normalizes image geometry using xyz information.
	/// @brief Function for geometric normalization.
	/// @param[in,out] img Color image that is normalized.
	/// @param[in,out] depth Pointcloud that is normalized.
	/// @return Return true/false whether geometric normalization was successful.
	bool normalize_geometry_depth(cv::Mat& img, cv::Mat& depth);

	/// The function manufactures artificial head poses
	/// @brief Function for artificial head poses.
	/// @param[in,out] img Color image that is normalized.
	/// @param[in,out] depth Pointcloud that is normalized.
	/// @return Return true/false whether rotation was successful.
	bool rotate_head(cv::Mat& img, cv::Mat& depth);

	/// The function detects facial features (nose, eyes) in color image.
	/// @brief Function detects facial features in color image.
	/// @param[in] img Color image containing facial features.
	/// @return Return true/false whether all features could be detected.
	bool features_from_color(cv::Mat& img);

	/// The function picks coordinates of facial features (nose, eyes) in point cloud.
	/// @brief Function picks 3D coordinates of facial features.
	/// @param[in] XYZ Input point cloud.
	/// @return Return true/false whether valid coordinates could be picked.
	bool features_from_depth(cv::Mat& depth);

	/// The function detects specific facial feature.
	/// @brief Function detects specified facial feature in color image.
	/// @param[in] img Color image containing facial features.
	/// @param[out] coords Image coordinates of detected facial feature.
	/// @param[in] type Feature type that is supposed t be detected.
	/// @return Return true/false whether feature could be detected.
	bool detect_feature(cv::Mat& img, cv::Point2f& coords, FACE::FEATURE_TYPE type);

	/// The function projects RGB and XYZ information of given image and point cloud to image plane.
	/// @brief Function projects point cloud on image plane.
	/// @param[in] RGB Color image.
	/// @param[in] XYZ Input point cloud.
	/// @param[out] img_res Resulting projected image.
	/// @param[out] depth_res Resulting projected depth image.
	/// @return Return true/false whether projection was successful.
	bool projectPointCloud(cv::Mat& RGB, cv::Mat& XYZ, cv::Mat& img_res, cv::Mat& depth_res);

	/// The function projects  XYZ information of point to image plane.
	/// @brief Function projects point to image plane.
	/// @param[in] xyz Input 3D point.
	/// @param[out] uv Output 2D image point.
	/// @param[out] depth_res Resulting projected depth image.
	/// @return Return true/false whether projection was successful.
	bool projectPoint(cv::Point3f& xyz, cv::Point2f& uv);

	/// The function creates normalized depth map from point cloud..
	/// @brief Function creates depth map from point cloud.
	/// @param[in] XYZ Input 3D point cloud.
	/// @param[out] DM Output normalized depth map.
	void create_DM(cv::Mat& XYZ, cv::Mat& DM);

	/// The function normalizes image illumination using GammaDCT or GammaDoG algorithms.
	/// @brief Function for illumination normalization.
	/// @param[in,out] img Color image that is normalized.
	/// @return Return true/false whether illumination normalization was successful.
	bool normalize_radiometry(cv::Mat& img);

	/// The function extracts the value channel in the HSV colorspace of a RGB image.
	/// @brief Function to extract value channel.
	/// @param[in] img Color image where value channel is extracted.
	/// @param[out] V Value channel of HSV representation of input image.
	/// @return true when process was successful
	void extractVChannel(cv::Mat& img, cv::Mat& V);

	/// The function substitutes the value channel in the HSV colorspace of a RGB image with a given image.
	/// @brief Function to substitute  value channel in color image.
	/// @param[in,out] img Color image where value channel is substituted.
	/// @param[in] V Value channel.
	void subVChannel(cv::Mat& img, cv::Mat& V);

	/// The function performs illumination normalization with the Gamma DCT method.
	///  This method is recommended for most cases.
	/// @brief Function for illumination normalization with Gamma DCT.
	/// @param[in,out] img Color image which is normalized.
	/// @return true when process was successful
	void GammaDCT(cv::Mat& img);

	/// The function performs illumination normalization with the Gamma DoG method.
	///  This method is recommended for extreme cases.
	///  It can be activated during initialization with configuration for extreme illumination conditions.
	/// @see FNConfig
	/// @brief Function for illumination normalization with Gamma DCT.
	/// @param[in,out] img Color image which is normalized.
	/// @return true when process was successful
	void GammaDoG(cv::Mat& img);

	/// This function is called to ensure  image return type consistency
	/// @param[in] in input image
	/// @param[out] out output image
	/// @return true when process was successful
	bool normalize_img_type(cv::Mat& in, cv::Mat& out);

	/// The function saves color image with marked detected facial features for debug puroposes.
	/// @brief Function to save image with detected features.
	/// @param[in,out] img Color image which is saved.
	void dump_features(cv::Mat& img);

	/// The function saves a given image under the specified path.
	/// @brief Function to save image under given path.
	/// @param[in,out] img Color image which is saved.
	void dump_img(cv::Mat& data, std::string name);

	bool initialized_; ///< Flag indicating if face normalizer is already initialized
	int epoch_ctr_; ///<  Counter for input frames
	bool debug_; ///< Flag activates debug outbput
	bool record_scene_; ///<  Flag indicates whether scene is recorded

	std::string storage_directory_; ///< Directory where debug information is saved
	std::string classifier_directory_; ///< Directory where haarcascade classifiers are stored

	//intrinsics
	cv::Mat cam_mat_; ///< Camera matrix used for projection to image plane.
	cv::Mat dist_coeffs_; ///< Distortion coefficients for camera model.

	CvHaarClassifierCascade* nose_cascade_; ///< OpenCv haarclassifier cascade for nose
	CvMemStorage* nose_storage_; ///< Pointer to OpenCv memory storage

	CvHaarClassifierCascade* eye_l_cascade_; ///< OpenCv haarclassifier cascade for left eye
	CvMemStorage* eye_l_storage_; ///< Pointer to OpenCv memory storage

	CvHaarClassifierCascade* eye_r_cascade_; ///< OpenCv haarclassifier cascade for right eye
	CvMemStorage* eye_r_storage_; ///< Pointer to OpenCv memory storage

	FACE::FaceFeatures<cv::Point2f> f_det_img_; ///< Facial features detected in color image.
	FACE::FaceFeatures<cv::Point3f> f_det_xyz_; ///< Facial features detected in point cloud.

	cv::Size norm_size_; ///< Norm size the image is scaled to.
	cv::Size input_size_; ///< Size of input image


	/// The uses interpolation to close gaps without color information.
	/// It can be used for images consisting of one or three channels.
	/// @brief Function fill gaps in image.
	/// @param[in] src Input image.
	/// @param[out dst Output image.
	template<class T>
	void despeckle(cv::Mat& src, cv::Mat& dst)
	{
		if (src.channels() == 1)
		{
			T* lptr = src.ptr<T>(1, 0);
			T* rptr = src.ptr<T>(1, 2);
			T* mptr = src.ptr<T>(1, 1);
			T* uptr = src.ptr<T>(0, 1);
			T* dptr = src.ptr<T>(2, 1);

			int normalizer = 4;

			for (int px = 2 * src.cols + 2; px < (dst.rows * src.cols); ++px)
			{
				if (*mptr == 0)
				{
					normalizer = 4;
					if (*lptr == 0)
						normalizer -= 1;
					if (*rptr == 0)
						normalizer -= 1;
					if (*uptr == 0)
						normalizer -= 1;
					if (*dptr == 0)
						normalizer -= 1;
					if (normalizer > 1)
						*mptr = (*lptr + *rptr + *uptr + *dptr) / normalizer;
					else
						*mptr = 75;
				}
				++lptr;
				++rptr;
				++mptr;
				++uptr;
				++dptr;
			}
		}

		if (src.channels() == 3)
		{
			cv::Vec<T, 3>* lptr = src.ptr<cv::Vec<T, 3> >(1, 0);
			cv::Vec<T, 3>* rptr = src.ptr<cv::Vec<T, 3> >(1, 2);
			cv::Vec<T, 3>* mptr = src.ptr<cv::Vec<T, 3> >(1, 1);
			cv::Vec<T, 3>* uptr = src.ptr<cv::Vec<T, 3> >(0, 1);
			cv::Vec<T, 3>* dptr = src.ptr<cv::Vec<T, 3> >(2, 1);

			int normalizer = 4;
			for (int px = 2 * src.cols + 2; px < (dst.rows * src.cols); ++px)
			{
				if ((*mptr)[0] == 0)
				{
					normalizer = 4;
					if ((*lptr)[0] == 0)
						normalizer -= 1;
					if ((*rptr)[0] == 0)
						normalizer -= 1;
					if ((*uptr)[0] == 0)
						normalizer -= 1;
					if ((*dptr)[0] == 0)
						normalizer -= 1;
					if (normalizer > 1)
						cv::divide((*lptr + *rptr + *uptr + *dptr), normalizer, *mptr);
					else
						*mptr = 75;
				}
				++lptr;
				++rptr;
				++mptr;
				++uptr;
				++dptr;
			}
		}
	}
};
