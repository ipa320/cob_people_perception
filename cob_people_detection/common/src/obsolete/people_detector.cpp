/// @file PeopleDetector.cpp

#ifdef __LINUX__
#include "cob_people_detection/people_detector.h"
#include "cob_vision_utils/GlobalDefines.h"
#else
#include "cob_vision/cob_people_detection/common/include/cob_people_detection/PeopleDetector.h"
#include "cob_common/cob_vision_utils/common/include/cob_vision_utils/GlobalDefines.h"
#endif

#include <opencv/cv.h>
#include <opencv/cvaux.h>

using namespace ipa_PeopleDetector;

PeopleDetector::PeopleDetector(void)
{
	m_face_cascade = 0;
	m_range_cascade = 0;
}

unsigned long PeopleDetector::Init(std::string directory)
{
	// Load Haar-Classifier for frontal face-, eyes- and body-detection
	std::string faceCascadePath = directory + "haarcascades/haarcascade_frontalface_alt2.xml";
	// todo:
	std::string rangeCascadePath = directory + "haarcascades/haarcascade_range_multiview_5p_bg.xml";
	//std::string rangeCascadePath = directory + "haarcascades/haarcascade_range_multiview_5p_bg+.xml";	// + "haarcascades/haarcascade_range.xml";
	m_face_cascade = (CvHaarClassifierCascade*)cvLoad(faceCascadePath.c_str(), 0, 0, 0); //"ConfigurationFiles/haarcascades/haarcascade_frontalface_alt2.xml", 0, 0, 0 );
	m_range_cascade = (CvHaarClassifierCascade*)cvLoad(rangeCascadePath.c_str(), 0, 0, 0);

	// Create Memory
	m_storage = cvCreateMemStorage(0);

	return ipa_Utils::RET_OK;
}

PeopleDetector::~PeopleDetector(void)
{
	// Release Classifiers and memory
	cvReleaseHaarClassifierCascade(&m_face_cascade);
	cvReleaseHaarClassifierCascade(&m_range_cascade);
	cvReleaseMemStorage(&m_storage);
}

unsigned long PeopleDetector::DetectColorFaces(cv::Mat& img, std::vector<cv::Rect>& faceCoordinates)
{
	IplImage imgPtr = (IplImage)img;
	CvSeq* faces = cvHaarDetectObjects(&imgPtr, m_face_cascade, m_storage, m_faces_increase_search_scale, m_faces_drop_groups, CV_HAAR_DO_CANNY_PRUNING,
			cvSize(m_faces_min_search_scale_x, m_faces_min_search_scale_y));

	cv::Size parentSize;
	cv::Point roiOffset;
	for (int i = 0; i < faces->total; i++)
	{
		cv::Rect* face = (cv::Rect*)cvGetSeqElem(faces, i);
		img.locateROI(parentSize, roiOffset);
		face->x += roiOffset.x; // todo: check what happens if the original matrix is used without roi
		face->y += roiOffset.y;
		faceCoordinates.push_back(*face);
	}

	return ipa_Utils::RET_OK;
}

unsigned long PeopleDetector::InterpolateUnassignedPixels(cv::Mat& img)
{
	CV_Assert( img.type() == CV_8UC3 )
		;

	cv::Mat temp = img.clone();

	uchar* data = img.data;
	uchar* data2 = temp.data;
	int stride = img.step;
	for (int repetitions = 0; repetitions < 10; repetitions++)
	{
		// each pixel with a value can propagate its value to black pixels in the 4 pixel neighborhoud
		for (int v = 1; v < img.rows - 1; v++)
		{
			for (int u = 1; u < img.cols - 1; u++)
			{
				// only pixels with a value can propagate their value
				int index = v * stride + 3 * u;
				if (data[index] != 0)
				{
					uchar val = data[index];
					if (data2[index - 3] == 0)
						for (int i = -3; i < 0; i++)
							data2[index + i] = val; // left
					if (data2[index + 3] == 0)
						for (int i = 3; i < 6; i++)
							data2[index + i] = val; // right
					if (data2[index - stride] == 0)
						for (int i = -stride; i < -stride + 3; i++)
							data2[index + i] = val; // up
					if (data2[index + stride] == 0)
						for (int i = stride; i < stride + 3; i++)
							data2[index + i] = val; // down
				}
			}
		}
		// copy back new data
		for (int i = 0; i < img.rows * stride; i++)
			data[i] = data2[i];
	}
	return ipa_Utils::RET_OK;
}

unsigned long PeopleDetector::DetectRangeFace(cv::Mat& img, std::vector<cv::Rect>& rangeFaceCoordinates, bool fillUnassignedDepthValues)
{
	rangeFaceCoordinates.clear();

	if (fillUnassignedDepthValues)
		InterpolateUnassignedPixels(img);
	//cv::namedWindow("depth image");
	//cv::imshow("depth image", img);
	//cv::waitKey(10);
	IplImage imgPtr = (IplImage)img;
	CvSeq* rangeFaces = cvHaarDetectObjects(&imgPtr, m_range_cascade, m_storage, m_range_increase_search_scale, m_range_drop_groups, CV_HAAR_DO_CANNY_PRUNING,
			cvSize(m_range_min_search_scale_x, m_range_min_search_scale_y));

	for (int i = 0; i < rangeFaces->total; i++)
	{
		cv::Rect *rangeFace = (cv::Rect*)cvGetSeqElem(rangeFaces, i);
		rangeFaceCoordinates.push_back(*rangeFace);
	}

	return ipa_Utils::RET_OK;
}

unsigned long PeopleDetector::DetectFaces(cv::Mat& img, cv::Mat& rangeImg, std::vector<cv::Rect>& colorFaceCoordinates, std::vector<cv::Rect>& rangeFaceCoordinates,
		std::set<size_t>& colorToRangeFaceDependency, bool fillUnassignedDepthValues)
{
	colorFaceCoordinates.clear();
	colorToRangeFaceDependency.clear();

	//######################################## Option1 ########################################
	DetectRangeFace(rangeImg, rangeFaceCoordinates, fillUnassignedDepthValues);
	for (int i = 0; i < (int)rangeFaceCoordinates.size(); i++)
	{
		cv::Rect rangeFace = rangeFaceCoordinates[i];

		rangeFace.y += (int)(rangeFace.height * 0.1);

		cv::Mat areaImg = img(rangeFace);

		// Detect color Faces and store the corresponding range face index if images were found
		size_t numberColorFacesBefore = colorFaceCoordinates.size();
		DetectColorFaces(areaImg, colorFaceCoordinates);
		if ((colorFaceCoordinates.size() - numberColorFacesBefore) != 0)
			colorToRangeFaceDependency.insert(i);
	}
	//######################################## /Option1 ########################################

	//######################################## Option2 ########################################
	/*DetectRangeFace(rangeImg, rangeFaceCoordinates);
	 IplImage imgPtr = (IplImage)img;
	 IplImage* areaImg = cvCloneImage(&imgPtr);
	 char* rowPtr = 0;
	 for (int row=0; row<areaImg->height; row++ )
	 {
	 rowPtr = (char*)(areaImg->imageData + row*areaImg->widthStep);
	 for (int col=0; col<areaImg->width; col++ )
	 {
	 bool inrect=false;
	 for(int i=0; i<(int)rangeFaceCoordinates->size(); i++)
	 {
	 cv::Rect rangeFace = rangeFaceCoordinates[i];

	 rangeFace.y += rangeFace.height*0.1;

	 if((col > rangeFace.x && col < (rangeFace.x + rangeFace.width)) && (row > rangeFace.y && row < (rangeFace.y + rangeFace.height)))
	 {
	 inrect=true;
	 break;
	 }
	 }

	 if(!inrect)
	 {
	 rowPtr[col*3] = 0;
	 rowPtr[col*3+1] = 0;
	 rowPtr[col*3+2] = 0;
	 }
	 }
	 }

	 // Detect color Faces
	 cv::Mat areaImgMat(areaImg);
	 DetectColorFaces(areaImgMat, colorFaceCoordinates);
	 areaImgMat.release();
	 cvReleaseImage(&areaImg);*/
	//######################################## /Option2 ########################################

	return ipa_Utils::RET_OK;
}

unsigned long PeopleDetector::AddFace(cv::Mat& img, cv::Rect& face, std::string id, std::vector<cv::Mat>& images, std::vector<std::string>& ids)
{
	//IplImage *resized_8U1 = cvCreateImage(cvSize(100, 100), 8, 1);
	cv::Mat resized_8U1(100, 100, CV_8UC1);
	ConvertAndResize(img, resized_8U1, face);

	// Save image
	images.push_back(resized_8U1);
	ids.push_back(id);

	return ipa_Utils::RET_OK;
}

unsigned long PeopleDetector::ConvertAndResize(cv::Mat& img, cv::Mat& resized, cv::Rect& face)
{
	cv::Mat temp;
	cv::cvtColor(img, temp, CV_BGR2GRAY);
	cv::Mat roi = temp(face);
	cv::resize(roi, resized, resized.size());

	return ipa_Utils::RET_OK;
}

cv::Mat PeopleDetector::preprocessImage(cv::Mat& input_image)
{
	// todo:
	return input_image;

	// do a modified census transform
	cv::Mat output(input_image.cols, input_image.rows, input_image.type());
	//cv::Mat smoothedImage = input_image.clone();
	//cv::GaussianBlur(smoothedImage, smoothedImage, cv::Size(3,3), 0, 0, cv::BORDER_REPLICATE);

	for (int v = 0; v < input_image.rows; v++)
	{
		uchar* srcPtr = input_image.ptr(v);
		//uchar* smoothPtr = smoothedImage.ptr(v);
		uchar* outPtr = output.ptr(v);
		for (int u = 0; u < input_image.cols; u++)
		{
			int ctOutcome = 0;
			int offset = -1;
			for (int dv = -1; dv <= 1; dv++)
			{
				for (int du = -1; du <= 1; du++)
				{
					if (dv == 0 && du == 0)
						continue;
					offset++;
					if (v + dv < 0 || v + dv >= input_image.rows || u + du < 0 || u + du >= input_image.cols)
						continue;
					//if (*smoothPtr < *(srcPtr+dv*input_image.step+du)) ctOutcome += 1<<offset;
					if (*srcPtr < *(srcPtr + dv * input_image.step + du))
						ctOutcome += 1 << offset;
				}
			}
			*outPtr = ctOutcome;

			srcPtr++;
			outPtr++;
		}
	}

	//	cv::imshow("census transform", output);
	//	cv::waitKey();

	return output;
}

unsigned long PeopleDetector::PCA(int* nEigens, std::vector<cv::Mat>& eigenVectors, cv::Mat& eigenValMat, cv::Mat& avgImage, std::vector<cv::Mat>& faceImages,
		cv::Mat& projectedTrainFaceMat)
{
	CvTermCriteria calcLimit;

	// Set the number of eigenvalues to use
	(*nEigens) = faceImages.size() - 1;

	// Allocate memory
	cv::Size faceImgSize(faceImages[0].cols, faceImages[0].rows);
	eigenVectors.resize(*nEigens, cv::Mat(faceImgSize, CV_32FC1));
	eigenValMat.create(1, *nEigens, CV_32FC1);
	avgImage.create(faceImgSize, CV_32FC1);

	// Set the PCA termination criterion
	calcLimit = cvTermCriteria(CV_TERMCRIT_ITER, (*nEigens), 1);

	// Convert vector to array
	IplImage** faceImgArr = (IplImage**)cvAlloc((int)faceImages.size() * sizeof(IplImage*));
	for (int j = 0; j < (int)faceImages.size(); j++)
	{
		// todo: preprocess
		cv::Mat preprocessedImage = preprocessImage(faceImages[j]);
		IplImage temp = (IplImage)preprocessedImage;
		faceImgArr[j] = cvCloneImage(&temp);
	}

	// Convert vector to array
	IplImage** eigenVectArr = (IplImage**)cvAlloc((int)eigenVectors.size() * sizeof(IplImage*));
	for (int j = 0; j < (int)eigenVectors.size(); j++)
	{
		IplImage temp = (IplImage)eigenVectors[j];
		eigenVectArr[j] = cvCloneImage(&temp);
	}

	// Compute average image, eigenvalues, and eigenvectors
	IplImage avgImageIpl = (IplImage)avgImage;
	cvCalcEigenObjects((int)faceImages.size(), (void*)faceImgArr, (void*)eigenVectArr, CV_EIGOBJ_NO_CALLBACK, 0, 0, &calcLimit, &avgImageIpl, (float*)(eigenValMat.data));

	// todo:
	cv::normalize(eigenValMat, eigenValMat, 1, 0, /*CV_L1*/CV_L2); //, 0);		0=bug?

	// Project the training images onto the PCA subspace
	projectedTrainFaceMat.create(faceImages.size(), *nEigens, CV_32FC1);
	for (int i = 0; i < (int)faceImages.size(); i++)
	{
		IplImage temp = (IplImage)faceImages[i];
		cvEigenDecomposite(&temp, *nEigens, eigenVectArr, 0, 0, &avgImageIpl, (float*)projectedTrainFaceMat.data + i * *nEigens); //attention: if image step of projectedTrainFaceMat is not *nEigens * sizeof(float) then reading functions which access with (x,y) coordinates might fail
	};

	// Copy back
	int eigenVectorsCount = (int)eigenVectors.size();
	eigenVectors.clear();
	for (int i = 0; i < (int)eigenVectorsCount; i++)
		eigenVectors.push_back(cv::Mat(eigenVectArr[i], true));

	// Clean
	for (int i = 0; i < (int)faceImages.size(); i++)
		cvReleaseImage(&(faceImgArr[i]));
	for (int i = 0; i < (int)eigenVectors.size(); i++)
		cvReleaseImage(&(eigenVectArr[i]));
	cvFree(&faceImgArr);
	cvFree(&eigenVectArr);

	return ipa_Utils::RET_OK;
}

unsigned long PeopleDetector::RecognizeFace(cv::Mat& colorImage, std::vector<cv::Rect>& colorFaceCoordinates, int* nEigens, std::vector<cv::Mat>& eigenVectors, cv::Mat& avgImage,
		cv::Mat& faceClassAvgProjections, std::vector<int>& index, int *threshold, int *threshold_FS, cv::Mat& eigenValMat, cv::SVM* personClassifier)
{
	float* eigenVectorWeights = 0;

	cv::Mat resized_8U1(100, 100, CV_8UC1); // = cvCreateImage(cvSize(100, 100), 8, 1);

	eigenVectorWeights = (float *)cvAlloc(*nEigens * sizeof(float));

	// Convert vector to array
	IplImage** eigenVectArr = (IplImage**)cvAlloc((int)eigenVectors.size() * sizeof(IplImage*));
	for (int j = 0; j < (int)eigenVectors.size(); j++)
	{
		IplImage temp = (IplImage)eigenVectors[j];
		eigenVectArr[j] = cvCloneImage(&temp);
	}

	for (int i = 0; i < (int)colorFaceCoordinates.size(); i++)
	{
		cv::Rect face = colorFaceCoordinates[i];
		ConvertAndResize(colorImage, resized_8U1, face);
		// todo: preprocess
		cv::Mat preprocessedImage = preprocessImage(resized_8U1);

		IplImage avgImageIpl = (IplImage)avgImage;

		// Project the test image onto the PCA subspace
		IplImage resized_8U1Ipl = (IplImage)resized_8U1;
		cvEigenDecomposite(&resized_8U1Ipl, *nEigens, eigenVectArr, 0, 0, &avgImageIpl, eigenVectorWeights);

		// Calculate FaceSpace Distance
		cv::Mat srcReconstruction = cv::Mat::zeros(eigenVectors[0].size(), eigenVectors[0].type());
		for (int i = 0; i < (int)eigenVectors.size(); i++)
			srcReconstruction += eigenVectorWeights[i] * eigenVectors[i];
		cv::Mat temp;

		// todo:
		//		cv::Mat reconstrTemp = srcReconstruction + avgImage;
		//		cv::Mat reconstr(eigenVectors[0].size(), CV_8UC1);
		//		reconstrTemp.convertTo(reconstr, CV_8UC1, 1);
		//		cv::imshow("reconstruction", reconstr);
		//		cv::waitKey();

		resized_8U1.convertTo(temp, CV_32FC1, 1.0 / 255.0);
		double distance = cv::norm((temp - avgImage), srcReconstruction, cv::NORM_L2);

		//######################################## Only for debugging and development ########################################
		//std::cout.precision( 10 );
		std::cout << "FS_Distance: " << distance << std::endl;
		//######################################## /Only for debugging and development ########################################

		// -2=distance to face space is too high
		// -1=distance to face classes is too high
		if (distance > *threshold_FS)
		{
			// No face
			index.push_back(-2);
			//index.push_back(-2); why twice? apparently makes no sense.
		}
		else
		{
			int nearest;
			ClassifyFace(eigenVectorWeights, &nearest, nEigens, faceClassAvgProjections, threshold, eigenValMat, personClassifier);
			if (nearest < 0)
				index.push_back(-1); // Face Unknown
			else
				index.push_back(nearest); // Face known, it's number nearest
		}
	}

	// Clear
	for (int i = 0; i < (int)eigenVectors.size(); i++)
		cvReleaseImage(&(eigenVectArr[i]));
	cvFree(&eigenVectorWeights);
	cvFree(&eigenVectArr);
	return ipa_Utils::RET_OK;
}

unsigned long PeopleDetector::ClassifyFace(float *eigenVectorWeights, int *nearest, int *nEigens, cv::Mat& faceClassAvgProjections, int *threshold, cv::Mat& eigenValMat,
		cv::SVM* personClassifier)
{
	double leastDistSq = DBL_MAX;
	//todo:
	int metric = 2; // 0 = Euklid, 1 = Mahalanobis, 2 = Mahalanobis Cosine


	for (int i = 0; i < faceClassAvgProjections.rows; i++)
	{
		double distance = 0;
		double cos = 0;
		double length_sample = 0;
		double length_projection = 0;
		for (int e = 0; e < *nEigens; e++)
		{
			if (metric < 2)
			{
				float d = eigenVectorWeights[e] - ((float*)(faceClassAvgProjections.data))[i * *nEigens + e];
				if (metric == 0)
					distance += d * d; //Euklid
				else
					distance += d * d /* / *// ((float*)(eigenValMat.data))[e]; //Mahalanobis
			}
			else
			{
				cos += eigenVectorWeights[e] * ((float*)(faceClassAvgProjections.data))[i * *nEigens + e] / ((float*)(eigenValMat.data))[e];
				length_projection += ((float*)(faceClassAvgProjections.data))[i * *nEigens + e] * ((float*)(faceClassAvgProjections.data))[i * *nEigens + e]
						/ ((float*)(eigenValMat.data))[e];
				length_sample += eigenVectorWeights[e] * eigenVectorWeights[e] / ((float*)(eigenValMat.data))[e];
			}
		}
		if (metric < 2)
			distance = sqrt(distance);
		else
		{
			length_sample = sqrt(length_sample);
			length_projection = sqrt(length_projection);
			cos /= (length_projection * length_sample);
			distance = -cos;
		}

		//######################################## Only for debugging and development ########################################
		//std::cout.precision( 10 );
		std::cout << "Distance_FC: " << distance << std::endl;
		//######################################## /Only for debugging and development ########################################

		if (distance < leastDistSq)
		{
			leastDistSq = distance;
			if (leastDistSq > *threshold)
				*nearest = -1;
			else
				*nearest = i;
		}
	}

	// todo:
	//	if (personClassifier != 0 && *nearest != -1)
	//	{
	//		cv::Mat temp(1, *nEigens, CV_32FC1, eigenVectorWeights);
	//		std::cout << "class. output: " << (int)personClassifier->predict(temp) << "\n";
	//		*nearest = (int)personClassifier->predict(temp);
	//	}

	return ipa_Utils::RET_OK;
}

unsigned long PeopleDetector::CalculateFaceClasses(cv::Mat& projectedTrainFaceMat, std::vector<std::string>& id, int *nEigens, cv::Mat& faceClassAvgProjections,
		std::vector<std::string>& idUnique, cv::SVM* personClassifier)
{
	std::cout << "PeopleDetector::CalculateFaceClasses ... ";

	// Look for face classes
	idUnique.clear();
	for (int i = 0; i < (int)id.size(); i++)
	{
		std::string face_class = id[i];
		bool class_exists = false;

		for (int j = 0; j < (int)idUnique.size(); j++)
		{
			if (!idUnique[j].compare(face_class))
			{
				class_exists = true;
			}
		}

		if (!class_exists)
		{
			idUnique.push_back(face_class);
		}
	}

	//id.clear();
	//cv::Mat faces_tmp = projectedTrainFaceMat.clone();
	cv::Mat temp = cv::Mat::zeros((int)idUnique.size(), *nEigens, projectedTrainFaceMat.type());
	temp.convertTo(faceClassAvgProjections, projectedTrainFaceMat.type());
	//for (int i=0; i<((int)idUnique.size() * *nEigens); i++)
	//	for (int i=0; i<((int)id.size() * *nEigens); i++)
	//	{
	//		((float*)(projectedTrainFaceMat.data))[i] = 0;
	//	}

	// Look for FaceClasses
	//	for(int i=0; i<(int)idUnique.size(); i++)
	//	{
	//		std::string face_class = idUnique[i];
	//		bool class_exists = false;
	//
	//		for(int j=0; j<(int)id.size(); j++)
	//		{
	//			if(!id[j].compare(face_class))
	//			{
	//				class_exists = true;
	//			}
	//		}
	//
	//		if(!class_exists)
	//		{
	//			id.push_back(face_class);
	//		}
	//	}

	//	cv::Size newSize(id.size(), *nEigens);
	//	projectedTrainFaceMat.create(newSize, faces_tmp.type());

	// Calculate FaceClasses
	for (int i = 0; i < (int)idUnique.size(); i++)
	{
		std::string face_class = idUnique[i];

		for (int e = 0; e < *nEigens; e++)
		{
			int count = 0;
			for (int j = 0; j < (int)id.size(); j++)
			{
				if (!(id[j].compare(face_class)))
				{
					((float*)(faceClassAvgProjections.data))[i * *nEigens + e] += ((float*)(projectedTrainFaceMat.data))[j * *nEigens + e];
					count++;
				}
			}
			((float*)(faceClassAvgProjections.data))[i * *nEigens + e] /= (float)count;
		}
	}

	// todo: machine learning technique for person identification
	if (personClassifier != 0)
	{
		//std::cout << "\n";
		// prepare ground truth
		cv::Mat data(id.size(), *nEigens, CV_32FC1);
		cv::Mat labels(id.size(), 1, CV_32SC1);
		std::ofstream fout("svm.dat", std::ios::out);
		for (int sample = 0; sample < (int)id.size(); sample++)
		{
			// copy data
			for (int e = 0; e < *nEigens; e++)
			{
				data.at<float>(sample, e) = ((float*)projectedTrainFaceMat.data)[sample * *nEigens + e];
				fout << data.at<float>(sample, e) << "\t";
			}
			// find corresponding label
			for (int i = 0; i < (int)idUnique.size(); i++) // for each person
				if (!(id[sample].compare(idUnique[i]))) // compare the labels
					labels.at<int>(sample) = i; // and assign the corresponding label's index from the idUnique list
			fout << labels.at<int>(sample) << "\n";
		}
		fout.close();

		// train the classifier
		cv::SVMParams svmParams(CvSVM::NU_SVC, CvSVM::RBF, 0.0, 0.001953125, 0.0, 0.0, 0.8, 0.0, 0, cv::TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 100, FLT_EPSILON));
		//personClassifier->train_auto(data, labels, cv::Mat(), cv::Mat(), svmParams, 10, cv::SVM::get_default_grid(CvSVM::C), CvParamGrid(0.001953125, 2.01, 2.0), cv::SVM::get_default_grid(CvSVM::P), CvParamGrid(0.0125, 1.0, 2.0));
		personClassifier->train(data, labels, cv::Mat(), cv::Mat(), svmParams);
		cv::SVMParams svmParamsOptimal = personClassifier->get_params();
		std::cout << "Optimal SVM params: gamma=" << svmParamsOptimal.gamma << "  nu=" << svmParamsOptimal.nu << "\n";
	}

	std::cout << "done\n";

	return ipa_Utils::RET_OK;
}

