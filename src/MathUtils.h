#pragma once
#include "stdafx.h"
double getMultiply(Vector3d a, Vector3d b);
double getSqrDist(double x, double y, double u, double v);
void ConvertCoordinates(int x, int y, double& target_x, double& target_y, Matrix3d H);
Matrix3d rollVectorToH(VectorXd h);
void ConvertPoint(const Mat& img, int Width, int Height, double raw_x, double raw_y, uchar& b, uchar& g, uchar& r);

template <typename T>
void getMeanAndSTD(const vector<T> & _vec, double & _mean, double & _std) {
	getMeanAndVariance(_vec, _mean, _std);
	_std = sqrt(_std);
}


template <typename T>
void getMeanAndVariance(const vector<T> & _vec, double & _mean, double & _var) {
	_mean = 0, _var = 0;
	const int count = (int)_vec.size();
	for (int i = 0; i < count; ++i) {
		_mean += _vec[i];
	}
	_mean /= count;

	for (int i = 0; i < count; ++i)
		_var += (_vec[i] * _vec[i]);
	_var = (_var / count) - (_mean * _mean);

}