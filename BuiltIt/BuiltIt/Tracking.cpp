// BuiltIt.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "Tracking.h"



void Tracking::init(Ptr<aruco::DetectorParameters>& detectorParams, Ptr<aruco::Dictionary>& dictionary, Mat& cameraMatrix, Mat& distCoeffs)
{
	_mycameraMatrix = cameraMatrix;
	_mydistCoeffs = distCoeffs;
	_mydictionary = dictionary;
	_mydetectorParams = detectorParams;
}

void Tracking::track(Mat& image, bool undistorted, vector<vector<Point2f>>& cornerPoints, vector<Vec3d>& rvecs, vector<Vec3d>& tvecs,vector<int>& markerIDs) 
{
	vector<vector<Point2f>> rejectedCandidates = { {} };

	//If image is distorted
	if (undistorted == false)
	{
		aruco::detectMarkers(image, _mydictionary, cornerPoints, markerIDs, _mydetectorParams, rejectedCandidates);
		aruco::drawDetectedMarkers(image, cornerPoints, markerIDs);
	}
	//Else the image already comes in undistorted
	else
	{
		aruco::detectMarkers(image, _mydictionary, cornerPoints, markerIDs, _mydetectorParams, rejectedCandidates, _mycameraMatrix, _mydistCoeffs);

		if (markerIDs.size() > 0)
		{
			aruco::estimatePoseSingleMarkers(cornerPoints, 0.185f, _mycameraMatrix, _mydistCoeffs, rvecs, tvecs);

			for (int i = 0; i < rvecs.size(); i++)
			{
				aruco::drawAxis(image, _mycameraMatrix, _mydistCoeffs, rvecs[i], tvecs[i], 0.1);
			}
		}
	}
}

