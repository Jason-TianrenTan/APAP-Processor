#pragma once
#include "stdafx.h"
#include "RANSAC_Processor.h"

//—∞’“Ãÿ’˜µ„
void findFeaturePointsWithSIFT(Mat img_1, Mat img_2, vector<KeyPoint>& key_p1, vector<KeyPoint>& key_p2, vector<DMatch>& matches);


//VLFeat∆•≈‰
//void findVLFeat_SIFT(const Mat& img_1, const Mat& img_2);

void drawMatch(Mat &img, const MatrixXf &match, const Matrix3f &inv_T1, const Matrix3f &inv_T2);

void normalizeMatch(MatrixXf &mat, Matrix3f &T1, Matrix3f &T2);

void combineMat(Mat &out, const Mat& left, const Mat& right);

void displayMat(const Mat& display);
