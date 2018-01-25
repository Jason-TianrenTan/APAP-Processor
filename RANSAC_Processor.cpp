#include "RANSAC_Processor.h"

void findGoodMatches(vector<DMatch> matches, vector<DMatch>& good_matches)
{
	double min_dist;
	for (size_t i = 0; i < matches.size(); i++)
	{
		double match_dist = matches[i].distance;
		if (i == 0)
			min_dist = match_dist;
		min_dist = match_dist < min_dist ? match_dist : min_dist;
	}

	for (size_t i = 0; i < matches.size(); i++)
		if (matches[i].distance <= Threshold * min_dist)
			good_matches.push_back(matches[i]);
}

//RANSACÌÞ³ýÎóÆ¥Åä
void RANSAC_filter(Mat img1, Mat img2, vector<KeyPoint> keypoints_1, vector<KeyPoint> keypoints_2,
	vector<KeyPoint>& RR_keypoint01, vector<KeyPoint>& RR_keypoint02, vector<DMatch> raw_matches)
{
	vector<KeyPoint> R_keypoint01, R_keypoint02;
	vector<DMatch> matches;
	findGoodMatches(raw_matches, matches);
	for (size_t i = 0; i < matches.size(); i++)
	{
		R_keypoint01.push_back(keypoints_1[matches[i].queryIdx]);
		R_keypoint02.push_back(keypoints_2[matches[i].trainIdx]);
	}

	//×ø±ê×ª»»
	vector<Point2d> p01, p02;//p01 -> object; p02 -> scene
	for (size_t i = 0; i < matches.size(); i++)
	{
		p01.push_back(R_keypoint01[i].pt);
		p02.push_back(R_keypoint02[i].pt);
	}

	//ÀûÓÃ»ù´¡¾ØÕóÌÞ³ýÎóÆ¥Åäµã
	vector<DMatch> goodMatches;
	vector<uchar> RansacStatus;
	Mat Fundamental = findHomography(p02, p01, RansacStatus, CV_FM_RANSAC);
	int index = 0;
	for (size_t i = 0; i < matches.size(); i++)
	{
		if (RansacStatus[i] != 0)
		{
			RR_keypoint01.push_back(R_keypoint01[i]);
			RR_keypoint02.push_back(R_keypoint02[i]);
			matches[i].queryIdx = index;
			matches[i].trainIdx = index;
			goodMatches.push_back(matches[i]);
			index++;
		}
	}
	cout << "good match count = " << index << endl;
	Mat display_match;
	drawMatches(img1, RR_keypoint01, img2, RR_keypoint02, goodMatches, display_match);
	imshow("matches", display_match);
}