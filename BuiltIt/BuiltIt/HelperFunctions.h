#pragma once
#include "stdafx.h"

#include <opencv2\core.hpp>
#include <opencv2\highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <opencv2\aruco.hpp>
#include <opencv2\aruco\charuco.hpp>
#include <opencv2\imgproc.hpp>

#include<fstream>
#include <vector>

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

class HelperFunctions
{
public:
	struct Pose
	{
		Vec3d trans;
		Vec4d quatRot;
	};
	/*
	Saves a given matrix in a specified filePath. Remember to add the file extension to the filePath
	*/
	static void saveToFile(Mat m, string filePath)
	{
		cout << "Saving a matrix with size " << m.size() << endl;

		ofstream out(filePath);

		for (int i = 0; i < m.size().height; i++)
		{
			for (int j = 0; j < m.size().width; j++)
			{
				out << m.at<double>(i, j) << " ";
			}
			out << endl;
		}
	}
	/*
	Loads a matrix from any specified file. Remember to specify the amount of rows and columns used in the matrix
	*/
	static Mat loadFromFile(string filePath, int rows, int cols)
	{
		double m;
		Mat out = Mat::zeros(rows, cols, CV_64FC1);//Matrix to store values

		ifstream fileStream(filePath);
		int cnt = 0;//index starts from 0
		while (fileStream >> m)
		{
			int temprow = cnt / cols;
			int tempcol = cnt % cols;
			out.at<double>(temprow, tempcol) = m;
			cnt++;
		}

		cout << "Loaded: " << endl << out << endl;
		return out;
	}

	/*
	Converts a Vector3 from Euler-Angles to a Quaternion, represented as a Vec4d
	ATTENTION: Requires Radiants instead of Angles
	*/
	static Vec4d toQuaternion(double rollX, double pitchY, double yawZ)
	{
		Vec4d q;

		double t0 = std::cos(yawZ * 0.5);
		double t1 = std::sin(yawZ * 0.5);
		double t2 = std::cos(rollX * 0.5);
		double t3 = std::sin(rollX * 0.5);
		double t4 = std::cos(pitchY * 0.5);
		double t5 = std::sin(pitchY * 0.5);

		q.val[3] = t0 * t2 * t4 + t1 * t3 * t5;
		q.val[0] = t0 * t3 * t4 - t1 * t2 * t5;
		q.val[1] = t0 * t2 * t5 + t1 * t3 * t4;
		q.val[2] = t1 * t2 * t4 - t0 * t3 * t5;

		return q;
	}

	/* Converts a given Vector3 - Rotation - In Euler Angles to a real quaternion.
	ATTENTION!!!!! Requires Radiants to work!!!!
	Also: This will return a left-handed system, meaning the z-Axis will be negated
	*/
	static Vec4d toQuaternionUbi(Vec3d input)//double x, double y, double z) 
	{
		double x = input[0];
		double y = input[1];
		double z = input[2];

		double c1 = cos(y / 2);
		double s1 = sin(y / 2);
		double c2 = cos(x / 2);
		double s2 = sin(x / 2);
		double c3 = cos(z / 2);
		double s3 = sin(z / 2);
		double c1c2 = c1*c2;
		double s1s2 = s1*s2;

		double a = c1c2*c3 - s1s2*s3;
		double b = c1c2*s3 + s1s2*c3;
		double c = s1*c2*c3 + c1*s2*s3;
		double d = c1*s2*c3 - s1*c2*s3;

		Vec4d res;
		res.val[0] = a;
		res.val[1] = b;
		res.val[2] = c;
		res.val[3] = d;

		return res;
	}

	static Vec3d degreeToRadians(Vec3d input)
	{
		Vec3d res;

		res[0] = input[0] * (3.14159265359 / 180);
		res[1] = input[1] * (3.14159265359 / 180);
		res[2] = input[2] * (3.14159265359 / 180);

		return res;
	}

	static Vec3d radiansToDegree(Vec3d input)
	{
		Vec3d res;

		res[0] = input[0] * (180 / 3.14159265359);
		res[1] = input[1] * (180 / 3.14159265359);
		res[2] = input[2] * (180 / 3.14159265359);

		return res;

	}

	/*
	Multiply a given Quaternion with a vector3. This runs down to rotating the vector by this quaternion
	*/
	static Vec3d mul(Vec4d quat, Vec3d vec)
	{

		double x, y, z, w;
		x = quat[0];
		y = quat[1];
		z = quat[2];
		w = quat[3];
		Vec3d r;

		// precomputation of some values
		double xy = x * y;
		double xz = x * z;
		double yz = y * z;
		double ww = w * w;
		double wx = w * x;
		double wy = w * y;
		double wz = w * z;

		r(0) = vec(0) * (2 * (x*x + ww) - 1) + vec(1) * 2 * (xy - wz) + vec(2) * 2 * (wy + xz);
		r(1) = vec(0) * 2 * (xy + wz) + vec(1) * (2 * (y*y + ww) - 1) + vec(2) * 2 * (yz - wx);
		r(2) = vec(0) * 2 * (xz - wy) + vec(1) * 2 * (wx + yz) + vec(2) * (2 * (z*z + ww) - 1);

		return r;

	}



	static Vec4d inverse(Vec4d& ov)
	{
		ov.val[0] = -ov.val[0];
		ov.val[1] = -ov.val[1];
		ov.val[2] = -ov.val[2];
		ov.val[3] = ov.val[3];

		return ov;
	}

	static Vec3d inverse(Vec3d ov)
	{
		ov.val[0] = -ov.val[0];
		ov.val[1] = -ov.val[1];
		ov.val[2] = -ov.val[2];

		return ov;
	}

	static Vec3d reprojectToImagePoint(Mat intrinsic, Vec3f poseTrans)
	{
		Mat pT = Mat(poseTrans, CV_64F);
		intrinsic.convertTo(intrinsic, CV_64F);

		float tX = pT.at<float>(0, 0);
		float tY = pT.at<float>(0, 1);
		float tZ = pT.at<float>(0, 2);

		double a = intrinsic.at<double>(0, 0) * tX + intrinsic.at<double>(0, 1) * tY + intrinsic.at<double>(0, 2) * tZ;
		double b = intrinsic.at<double>(1, 0) * tX + intrinsic.at<double>(1, 1) * tY + intrinsic.at<double>(1, 2) * tZ;
		double c = intrinsic.at<double>(2, 0) * tX + intrinsic.at<double>(2, 1) * tY + intrinsic.at<double>(2, 2) * tZ;

		a /= c;
		b /= c;
		c /= c;

		return Vec3d(a, b, c);
	}

	/*
	Converts any given rotation matrix to quaternion
	*/
	static Vec4d matrixIntoQuaternions(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
	{
		double tr = m00 + m11 + m22;
		double qx, qy, qz, qw;

		if (tr > 0) {
			double S = sqrt(tr + 1.0) * 2; // S=4*qw 
			qw = 0.25 * S;
			qx = (m21 - m12) / S;
			qy = (m02 - m20) / S;
			qz = (m10 - m01) / S;
		}
		else if ((m00 > m11)&(m00 > m22)) {
			double S = sqrt(1.0 + m00 - m11 - m22) * 2; // S=4*qx 
			qw = (m21 - m12) / S;
			qx = 0.25 * S;
			qy = (m01 + m10) / S;
			qz = (m02 + m20) / S;
		}
		else if (m11 > m22) {
			double S = sqrt(1.0 + m11 - m00 - m22) * 2; // S=4*qy
			qw = (m02 - m20) / S;
			qx = (m01 + m10) / S;
			qy = 0.25 * S;
			qz = (m12 + m21) / S;
		}
		else {
			double S = sqrt(1.0 + m22 - m00 - m11) * 2; // S=4*qz
			qw = (m10 - m01) / S;
			qx = (m02 + m20) / S;
			qy = (m12 + m21) / S;
			qz = 0.25 * S;
		}

		return Vec4d(qx, qy, qz, qw);
	}

	static Pose poseMul(Pose firstP, Pose secondP)
	{
		Pose res;

		res.quatRot = firstP.quatRot * secondP.quatRot;
		res.trans = (HelperFunctions::mul(firstP.quatRot, secondP.trans)) + firstP.trans;

		return res;
	}

	static Pose invertPose(Pose original)
	{
		Pose result;

		result.quatRot = HelperFunctions::inverse(original.quatRot);
		result.trans = -(HelperFunctions::mul(result.quatRot, result.trans));

		return result;
	}

	static Vec4d rotationVectorToQuaternion(Vec3d rot)
	{
		Mat rv1Matrix;

		cv::Rodrigues(rot, rv1Matrix);

		Vec4d rv1Quat = HelperFunctions::matrixIntoQuaternions(rv1Matrix.at<double>(0, 0), rv1Matrix.at<double>(0, 1), rv1Matrix.at<double>(0, 2), rv1Matrix.at<double>(1, 0), rv1Matrix.at<double>(1, 1), rv1Matrix.at<double>(1, 2), rv1Matrix.at<double>(2, 0), rv1Matrix.at<double>(2, 1), rv1Matrix.at<double>(2, 2));

		return rv1Quat;
	}

	static double calculateEuclideanDistance(Vec2d a, Vec2d b)
	{
		cout << "Incoming: " << a << " " << b << endl;
		return sqrt(pow((a[0] - b[0]),2) + pow((a[1] - b[1]),2));
	}
};