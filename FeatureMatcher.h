#pragma once
#include "stdafx.h"
#include <opencv2/flann/miniflann.hpp>
class FeatureMatcher
{
public:
	
	FeatureMatcher(vector<Point2d> points);
	~FeatureMatcher();
	void FeatureMatch();
};

