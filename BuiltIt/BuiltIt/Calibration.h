#pragma once
#include "stdafx.h"

#include <opencv2\core.hpp>
#include <opencv2\highgui.hpp>

#include <opencv2\aruco.hpp>
#include <opencv2\aruco\charuco.hpp>
#include <opencv2\imgproc.hpp>

#include<fstream>
#include <vector>

using namespace cv;
using namespace std;

#include <iostream>

class Calibration
{
public:
	void init(Size img_size, Ptr<aruco::Dictionary>& dic, Ptr<aruco::DetectorParameters>& params);
	void Calibration::makeCalibrationSnapShot(Mat inputImage, int camID);
	Mat& Calibration::createArucoMarker(int dICTType, int id, int sizeOfMarkerImage);
	void Calibration::createMarkerSetOnHDD(int dICTType, int idFirst, int idLast, int sizeOfMarkerImage, string savepath);
};