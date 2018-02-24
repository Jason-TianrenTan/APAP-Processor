#include <iostream>
#include "SIFT_Matcher.h"
#include "Homography.h"
#include "APAP_Processor.h"
#include "RANSAC_Processor.h"
#include "MathUtils.h"
using namespace cv;
using namespace std;
using namespace Eigen;
const char* surface_path = "C:\\Users\\atsst\\Pictures\\Saved Pictures\\";
const char* pc_path = "C:\\Users\\Administrator\\Pictures\\Saved Pictures\\opencv\\";
string current_path = pc_path;
vector<Point2d> Left, Right;
vector<KeyPoint> left_keypoint, right_keypoint;
int main() {

	string imagename1 = current_path + "temple_01.JPG",
		imagename2 = current_path + "temple_02.JPG";
	Mat img_1 = imread(imagename1), img_2 = imread(imagename2);

	vector<DMatch> matches;
	//vector<KeyPoint> keypoints_1, keypoints_2;
	//vector<KeyPoint> keyPoints_obj, keyPoints_scene;
	vector<Point2d> obj, scene;//ÌØÕ÷µã


	cout << "SIFT Feature points..." << endl;
	/*
	findFeaturePointsWithSIFT(img_1, img_2, keypoints_1, keypoints_2, matches);
	cout << "Using RANSAC filtering matches..." << endl;
	RANSAC_filter(img_1, img_2, keypoints_1, keypoints_2, keyPoints_obj, keyPoints_scene, matches);*/
	ifstream fin("features.txt");
	double x1, y1, x2, y2;
	for (int i = 0; i < 135; i++) 
	{
		fin >> x1 >> y1 >> x2 >> y2;
		//cout << x1 << "," << y1 << "\t" << x2 << "," << y2 << endl;
		left_keypoint.emplace_back(KeyPoint(Point2f(x1, y1), 1.f));
		right_keypoint.emplace_back(KeyPoint(Point2f(x2, y2), 1.f));
	}

	MatrixXd A;
	Matrix3d H;
	vector<MatrixXd> Wi_Vector;
	vector<Matrix3d> H_vectors;
	cout << "Calculating homography..." << endl;
	//getHomography(keyPoints_scene, keyPoints_obj, scene, obj, H, A);
	getHomography(right_keypoint, left_keypoint, scene, obj, H, A);

	cout << "Calculating Weighted matrices..." << endl;
	calculate_Wi_Matrices(img_2, scene, Wi_Vector);
	cout << "testing..." << endl;
	H_vectors = calculate_CellHomography(Wi_Vector, A);
	Mat homography_target, display;
	cout << "Converting image 2..." << endl;
	ConvertImage(img_2, homography_target, H_vectors, C1, C2);

	warpImage(img_1, homography_target, display);
	return 0;
}
