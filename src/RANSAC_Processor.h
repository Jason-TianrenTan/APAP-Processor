#pragma once
#include "stdafx.h"
#include "MathUtils.h"
void RANSAC_filter(Mat img1, Mat img2, vector<KeyPoint> keypoints_1, vector<KeyPoint> keypoints_2,
	vector<KeyPoint>& RR_keypoint01, vector<KeyPoint>& RR_keypoint02, vector<DMatch> raw_matches);