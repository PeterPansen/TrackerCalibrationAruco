// BuiltIt.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "Calibration.h"

#include "HelperFunctions.h"

Ptr<aruco::CharucoBoard> _board;
Ptr<aruco::DetectorParameters> _detectorParams;
Ptr<aruco::Dictionary> _dictionary;

Mat _cameraMatrix, _distCoeffs;
vector<vector<int>> _allCharucoIds;
vector<vector<cv::Point2f>> _allCharucoCorners;

Size _Camera_img_size;

void Calibration::init(Size img_size, Ptr<aruco::Dictionary>& dic, Ptr<aruco::DetectorParameters>& params)
{
	_Camera_img_size = img_size;
	//Allocate space to one of the predefined aruco marker dictionaries
	_dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50);

	dic = _dictionary;

	_detectorParams = aruco::DetectorParameters::create();

	_detectorParams->cornerRefinementMethod = aruco::CornerRefineMethod::CORNER_REFINE_SUBPIX;

	params = _detectorParams;

	_board = aruco::CharucoBoard::create(5, 7, 0.036, 0.021, _dictionary);
}


/*
	Call this function to take a screenshot/snapshot of the currently active camera-image.
	It will check the images for the board defined in "init()" and use it to further refine our cameraMatrix and distCoefficients.
*/
void Calibration::makeCalibrationSnapShot(Mat inputImage, int camID)
{
	cout << "Received new calibration snapshot" << endl;
	vector<int> markerIds;
	vector<vector<Point2f>> markerCorners;


	aruco::detectMarkers(inputImage, _board->dictionary, markerCorners, markerIds);

	if (markerIds.size() > 4)
	{
		cout << "We have " << markerIds.size() << " markers and " << markerCorners.size() << " corners" << endl;
		vector<Point2f> charucoCorners;
		vector<int> charucoIds;

		aruco::interpolateCornersCharuco(markerCorners, markerIds, inputImage, _board, charucoCorners, charucoIds);

		_allCharucoCorners.push_back(charucoCorners);
		_allCharucoIds.push_back(charucoIds);

		vector<Mat> rvecs, tvecs;
		double repError = aruco::calibrateCameraCharuco(_allCharucoCorners, _allCharucoIds, _board, _Camera_img_size, _cameraMatrix, _distCoeffs, rvecs, tvecs);

		cout << "Intrinsic: " << endl << _cameraMatrix << endl << endl;

		stringstream ss;

		ss << "intrinsic_" << camID << ".txt";

		HelperFunctions::saveToFile(_cameraMatrix, ss.str());

		ss.str("");

		ss << "distCoeffs_" << camID << ".txt";

		HelperFunctions::saveToFile(_distCoeffs, ss.str());
	}
}

Mat& Calibration::createArucoMarker(int dICTType, int id, int sizeOfMarkerImage) 
{
	Mat markerImg;
	Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(dICTType);
	cv::aruco::drawMarker(dictionary, id, sizeOfMarkerImage, markerImg);

	return markerImg;
}

void Calibration::createMarkerSetOnHDD(int dICTType, int idFirst, int idLast,int sizeOfMarkerImage, string savepath)
{
	if (idFirst > idLast) 
	{
		cerr << "Error in createMarkerSetOnHDD. idFirst and idLast in conflict" << endl;
		return;
	}

	Mat markerImg;
	Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(dICTType);
	std::stringstream ss;

	for (int i = idFirst; i <= idLast; i++) 
	{
		cv::aruco::drawMarker(dictionary, i, sizeOfMarkerImage, markerImg);
		ss << savepath << "/markerID_" << i << ".jpg";
		imwrite(ss.str(), markerImg);
		ss.str("");
	}
}