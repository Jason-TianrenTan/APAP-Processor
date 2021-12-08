#pragma once
#include "stdafx.h"
void getHomography(vector<KeyPoint> obj_keyP, vector<KeyPoint> scene_keyP, 
	vector<Point2d>& obj, vector<Point2d>& scene, Matrix3d& H, MatrixXd& A);