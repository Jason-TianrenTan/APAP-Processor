#pragma once
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <limits.h>
#include <cmath>
#include <Eigen/Dense>
extern "C" {
#include <vl/generic.h>
#include <vl/slic.h>
#include <vl/sift.h>
}
#define  _USE_MATH_DEFINES
#include <math.h>
using namespace cv;
using namespace std;
using namespace Eigen;

const double gamma = 0.0015;
const double sigma = 12.0;
const int C1 = 50, C2 = 50;//Grid count
const int width = 1525, height = 893;//warp image
const double Threshold = 4.5;
const double MAXFLOAT = numeric_limits<float>::max();
const double GLOBAL_HOMOGRAPHY_MAX_INLIERS_DIST = 5.;
const double  LOCAL_HOMOGRAPHY_MAX_INLIERS_DIST = 3.;
const    int  LOCAL_HOMOGRAPHY_MIN_FEATURES_COUNT = 40;

const    int SIFT_LEVEL_COUNT = 4;
const    int SIFT_MINIMUM_OCTAVE_INDEX = 0;
const double SIFT_PEAK_THRESH = 0.;
const double SIFT_EDGE_THRESH = 10.;

const double INLIER_TOLERANT_STD_DISTANCE = 4.25;

const double GLOBAL_TRUE_PROBABILITY = 0.225;
const double LOCAL_TRUE_PROBABILITY = 0.2;
const double OPENCV_DEFAULT_CONFIDENCE = 0.995;

const double STRONG_CONSTRAINT = 1e4;

const int CRITERIA_MAX_COUNT = 1000;
const double CRITERIA_EPSILON = DBL_EPSILON;

const double TOLERANT_ANGLE = 1.5;