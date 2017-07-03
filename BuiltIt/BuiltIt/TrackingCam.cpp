#include "stdafx.h"
#include "TrackingCam.h"

vector<int> markerIDs;
vector<Vec3d> tvecs, rvecs;

Mat img;

int camID;

VideoCapture cap;

Mat distortionCoeffs, cameraMatrix;

vector<vector<Point2f>> cornerPoints = { {} };
vector<vector<Point2f>> rejectedCandidates = { {} };

Ptr<aruco::Dictionary> _tcDictionary;
Ptr<aruco::DetectorParameters> _tcDetectorParams;

bool calibrated = false;

Calibration camCalib;

int init() 
{
	_tcDetectorParams->cornerRefinementMethod = aruco::CornerRefineMethod::CORNER_REFINE_SUBPIX;

	cap.open(camID);

	if (!cap.isOpened()) 
	{
		cerr << "Couldn't open cam with ID " << camID << endl;
		return -1;
	}
}

Mat& fetchCamStream() 
{
	cap >> img;

	if (calibrated) 
	{
		cv::undistort(img, img, cameraMatrix, distortionCoeffs);
	}

	return img;
}

void takeCalibrationSnapshot() 
{
	camCalib.init(img.size(), _tcDictionary, _tcDetectorParams);
	camCalib.makeCalibrationSnapShot(img, camID);
}

void loadCalibration(string intrinsicFile = "", string distortionCoeffsFile = "") 
{
	if (intrinsicFile == "" && distortionCoeffsFile == "")
	{
		stringstream ss;

		ss << "intrinsic_" << camID << ".txt";

		cameraMatrix = HelperFunctions::loadFromFile(ss.str(), 3, 3);

		ss.str("");

		ss << "distCoeffs_" << camID << ".txt";

		distortionCoeffs = HelperFunctions::loadFromFile(ss.str(), 1, 5);
	}

	else
	{
		cameraMatrix = HelperFunctions::loadFromFile(intrinsicFile, 3, 3);
		distortionCoeffs = HelperFunctions::loadFromFile(distortionCoeffsFile, 1, 5);
	}

	calibrated = true;
}

void searchMarkers() 
{
	if (calibrated)
	{
		aruco::detectMarkers(img, _tcDictionary, cornerPoints, markerIDs, _tcDetectorParams, rejectedCandidates,cameraMatrix,distortionCoeffs);
	}
	else 
	{
		aruco::detectMarkers(img, _tcDictionary, cornerPoints, markerIDs, _tcDetectorParams, rejectedCandidates);
	}
}

HelperFunctions::Pose getMarkerPose(int mID, float markerLength) 
{
	bool found = false;
	HelperFunctions::Pose result;

	searchMarkers();
	aruco::estimatePoseSingleMarkers(cornerPoints, markerLength, cameraMatrix, distortionCoeffs, rvecs, tvecs);

	for (int i = 0; i < markerIDs.size(); i++) 
	{
		//Here we found the corresponding marker
		if (markerIDs[i] == mID) 
		{
			result.quatRot = HelperFunctions::rotationVectorToQuaternion(rvecs[i]);
			result.trans = tvecs[i];
			found = true;
			break;
		}
	}

	if (found == false) 
	{
		cout << "ATTENTION!! Did not find the corresponding Marker! Returning empty Pose in getMarkerPose (TrackingCam.cpp)" << endl;
	}

	return result;
}