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
#include <cmath>
using namespace cv;
using namespace std;
using namespace Eigen;

const double gamma = 0.0025;
const double sigma = 12.0;
const int C1 = 50, C2 = 50;//Grid count
const int width = 2025, height = 1193;//warp image
const double Threshold = 4.5;