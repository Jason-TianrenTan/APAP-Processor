#pragma once
#include "stdafx.h"
#include "GridBox.h"
void calculate_Wi_Matrices(Mat img, vector<Point2d>& obj, vector<MatrixXd>& vec);
vector<Matrix3d> calculate_CellHomography(vector<MatrixXd>& matrices, MatrixXd& A);
void ConvertImage(const Mat& img, Mat& target, vector<Matrix3d> H_vec, int C1, int C2);
void warpImage(const Mat& img_1, const Mat& img_2, Mat& target);