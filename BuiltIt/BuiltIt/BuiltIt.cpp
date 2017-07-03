// HelperFunctions.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "BuiltIt.h"
#include "HelperFunctions.h"

#include "MarkerTracker.h"
#include "PoseEstimation.h"

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2\highgui.hpp>

Mat img_bgr_1, img_bgr_2, img_markers_c1, img_markers_c2, markerImage, img_undistorted;

Mat cameraMatrix_c1, cameraMatrix_c2, distCoeffs_c1, distCoeffs_c2;

Calibration _camCalib_c1, _camCalib_c2;

Ptr<aruco::Dictionary> dictionary;
Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();



vector<vector<Point2f>> cornerPoints_c1, cornerPoints_c2 = { {} };
vector<vector<Point2f>> rejectedCandidates_c1, rejectedCandidates_c2 = { {} };
vector<int> markerIDs_c1, markerIDs_c2;
vector< Vec3d > rvecs1, rvecs2, tvecs1, tvecs2;




void loadIntrinsics()
{
	cameraMatrix_c1 = HelperFunctions::loadFromFile("intrinsic_1.txt", 3, 3);
	distCoeffs_c1 = HelperFunctions::loadFromFile("distCoeffs_1.txt", 1, 5);

	cameraMatrix_c2 = HelperFunctions::loadFromFile("intrinsic_2.txt", 3, 3);
	distCoeffs_c2 = HelperFunctions::loadFromFile("distCoeffs_2.txt", 1, 5);
}

void calculateReprojectionError(Mat drawTo, Vec3d trans)
{
	circle(drawTo, Point2d(trans.val[0], trans.val[1]), 5, Scalar(255, 0, 0), -1);
	return;
}

/*
Errechnet die Pose von Camera 2 zu Camera 1
*/

HelperFunctions::Pose fetchPoseBetween2Cameras(Vec3d tv1, Vec3d tv2, Vec3d rv1, Vec3d rv2)
{
	Mat rv1Matrix, rv2Matrix;

	cv::Rodrigues(rv1, rv1Matrix);
	cv::Rodrigues(rv2, rv2Matrix);


	Vec4d rv1Quat = HelperFunctions::matrixIntoQuaternions(rv1Matrix.at<double>(0, 0), rv1Matrix.at<double>(0, 1), rv1Matrix.at<double>(0, 2), rv1Matrix.at<double>(1, 0), rv1Matrix.at<double>(1, 1), rv1Matrix.at<double>(1, 2), rv1Matrix.at<double>(2, 0), rv1Matrix.at<double>(2, 1), rv1Matrix.at<double>(2, 2));
	Vec4d rv2Quat = HelperFunctions::matrixIntoQuaternions(rv2Matrix.at<double>(0, 0), rv2Matrix.at<double>(0, 1), rv2Matrix.at<double>(0, 2), rv2Matrix.at<double>(1, 0), rv2Matrix.at<double>(1, 1), rv2Matrix.at<double>(1, 2), rv2Matrix.at<double>(2, 0), rv2Matrix.at<double>(2, 1), rv2Matrix.at<double>(2, 2));

	cout << "Cam1 to Marker: " << tv1 << endl;
	cout << "Cam2 to Marker: " << tv2 << endl;


	//Inversing the left side with Cam1 to Marker

	cout << "Cam1 To Marker: R:" << rv1Quat << " :T: " << tv1 << endl;

	Vec4d rv1_Inverse = HelperFunctions::inverse(rv1Quat);
	Vec3d tv1_Inverse = -(HelperFunctions::mul(rv1_Inverse, tv1));

	cout << "InverseResult: R:" << rv1_Inverse << " :T: " << tv1_Inverse << endl;

	//Here I inverted Camera1 To Marker to now read Marker to Camera1
	HelperFunctions::Pose markerToC1;
	markerToC1.quatRot = rv1_Inverse;
	markerToC1.trans = tv1_Inverse;

	//Multiplying the Pose from Camera2ToMarker with the pose from MarkerToCamera1 yields the offset between both cameras
	HelperFunctions::Pose C2ToC1;

	C2ToC1.quatRot = rv2Quat * markerToC1.quatRot;

	cout << endl << "Resulting translation is " << rv2Quat << " * " << markerToC1.trans << endl << "And all that finally added to " << tv2 << endl;

	C2ToC1.trans = (HelperFunctions::mul(rv2Quat, markerToC1.trans)) + tv2;

	cout << "Result: " << C2ToC1.trans << endl << endl;;

	return C2ToC1;
}


int main()
{
	namedWindow("ColorImage_C1", CV_WINDOW_NORMAL);
	namedWindow("ColorImage_C2", CV_WINDOW_NORMAL);

	VideoCapture cap_1, cap_2;
	cap_1.open(0);


	if (!cap_1.isOpened())
	{
		cerr << "Couldn't open first camera stream" << endl;
		return -1;
	}

	cap_2.open(1);
	if (!cap_2.isOpened())
	{
		cerr << "Couldn't open second camera stream" << endl;
		return -1;
	}
	cap_2.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
	cap_2.set(CV_CAP_PROP_FRAME_HEIGHT, 1080);


	cap_1 >> img_bgr_1;
	cap_2 >> img_bgr_2;

	_camCalib_c1.init(img_bgr_1.size(), dictionary, detectorParams);
	_camCalib_c2.init(img_bgr_2.size(), dictionary, detectorParams);


	loadIntrinsics();

	while (true)
	{
		cap_1 >> img_bgr_1;
		cap_2 >> img_bgr_2;


		imshow("ColorImage_C1", img_bgr_1);
		img_bgr_1.copyTo(img_markers_c1);

		imshow("ColorImage_C2", img_bgr_2);
		img_bgr_2.copyTo(img_markers_c2);



		if (!cameraMatrix_c1.empty() && !cameraMatrix_c2.empty())
		{
			undistort(img_bgr_1, img_markers_c1, cameraMatrix_c1, distCoeffs_c1);
			undistort(img_bgr_2, img_markers_c2, cameraMatrix_c2, distCoeffs_c2);

			aruco::detectMarkers(img_markers_c1, dictionary, cornerPoints_c1, markerIDs_c1, detectorParams, rejectedCandidates_c1, cameraMatrix_c1, distCoeffs_c1);
			aruco::detectMarkers(img_markers_c2, dictionary, cornerPoints_c2, markerIDs_c2, detectorParams, rejectedCandidates_c2, cameraMatrix_c2, distCoeffs_c2);

			aruco::estimatePoseSingleMarkers(cornerPoints_c1, 0.185f, cameraMatrix_c1, distCoeffs_c1, rvecs1, tvecs1);
			aruco::estimatePoseSingleMarkers(cornerPoints_c2, 0.185f, cameraMatrix_c2, distCoeffs_c2, rvecs2, tvecs2);

			aruco::drawDetectedMarkers(img_markers_c1, cornerPoints_c1, markerIDs_c1);
			aruco::drawDetectedMarkers(img_markers_c2, cornerPoints_c2, markerIDs_c2);



			int index1, index2;
			bool foundAll = false;

			for (int i = 0; i < markerIDs_c1.size(); i++)
			{
				if (markerIDs_c1[i] == 21)
				{
					index1 = i;
					//cout << tvecs1[i] << endl;
					foundAll = true;
					break;
				}
			}

			if (foundAll == true)
			{
				foundAll = false;
				for (int i = 0; i < markerIDs_c2.size(); i++)
				{
					if (markerIDs_c2[i] == 21)
					{
						index2 = i;
						//cout << tvecs2[i] << endl;
						foundAll = true;
						break;
					}
				}
			}

			if (foundAll)
			{
				HelperFunctions::Pose C2ToC1 = fetchPoseBetween2Cameras(tvecs1[index1], tvecs2[index2], rvecs1[index1], rvecs2[index2]);
				cout << "Pose between both cameras: " << C2ToC1.trans << endl;

				HelperFunctions::Pose C2ToMarker = HelperFunctions::Pose();
				C2ToMarker.quatRot = HelperFunctions::rotationVectorToQuaternion(rvecs2[index2]);
				C2ToMarker.trans = tvecs2[index2];

				//The Pose the marker has in C1's coordinate system using C2's information only

				HelperFunctions::Pose C1ToC2 = HelperFunctions::invertPose(C2ToC1);
				HelperFunctions::Pose C1ToMarkerViaC2 = HelperFunctions::poseMul(C1ToC2, C2ToMarker);


				Vec3d reproj1 = HelperFunctions::reprojectToImagePoint(cameraMatrix_c1, tvecs1[index1]);
				circle(img_markers_c1, Point2d(reproj1[0], reproj1[1]), 10, Scalar(255, 0, 0), -1);
				Vec3d reproj2 = HelperFunctions::reprojectToImagePoint(cameraMatrix_c1, C1ToMarkerViaC2.trans);
				circle(img_markers_c1, Point2d(reproj2[0], reproj2[1]), 10, Scalar(0, 0, 255), -1);
			}
		}


		aruco::drawDetectedMarkers(img_markers_c1, cornerPoints_c1, markerIDs_c1);
		aruco::drawDetectedMarkers(img_markers_c2, cornerPoints_c2, markerIDs_c2);



		namedWindow("Markers_C1", CV_WINDOW_NORMAL);
		imshow("Markers_C1", img_markers_c1);

		namedWindow("Markers_C2", CV_WINDOW_NORMAL);
		imshow("Markers_C2", img_markers_c2);

		int charKey = waitKey(10);

		//Escape-Key to end the application
		if (charKey == 27)
		{
			break;
		}

		//Space Key to take a snapshot of the CharuCo-Board
		if (charKey == 49)
		{
			_camCalib_c1.makeCalibrationSnapShot(img_bgr_1, 1);
		}

		//Left shift for second camera snapshot
		if (charKey == 50)
		{
			cout << "Taking calibration snapshot" << endl;
			_camCalib_c2.makeCalibrationSnapShot(img_bgr_2, 2);
		}

		//L Key to Load our intrinsics
		if (charKey == 108)
		{
			cameraMatrix_c1 = HelperFunctions::loadFromFile("intrinsic_1.txt", 3, 3);
			distCoeffs_c1 = HelperFunctions::loadFromFile("distCoeffs_1.txt", 1, 5);

			cameraMatrix_c2 = HelperFunctions::loadFromFile("intrinsic_2.txt", 3, 3);
			distCoeffs_c2 = HelperFunctions::loadFromFile("distCoeffs_2.txt", 1, 5);
		}
	}

	return 0;
}

