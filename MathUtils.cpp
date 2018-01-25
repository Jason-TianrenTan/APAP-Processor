#include "MathUtils.h"
using namespace std;
using namespace cv;
using namespace Eigen;
//向量点乘
double getMultiply(Vector3d a, Vector3d b)
{
	double ax = a[0], ay = a[1], az = a[2];
	double bx = b[0], by = b[1], bz = b[2];
	return ax*bx + ay*by + az*bz;
}

//距离的平方
double getSqrDist(double x, double y, double u, double v) {
	return (x - u) * (x - u) + (y - v) * (y - v);
}

//坐标转换
void ConvertCoordinates(int x, int y, double& target_x, double& target_y, Matrix3d H)
{
	Vector3d h1 = H.row(0), h2 = H.row(1), h3 = H.row(2);
	Vector3d xy_vec(x, y, 1);
	target_x = getMultiply(h1, xy_vec) / getMultiply(h3, xy_vec);
	target_y = getMultiply(h2, xy_vec) / getMultiply(h3, xy_vec);
}

//h转H
Matrix3d rollVectorToH(VectorXd h)
{
	Matrix3d H;
	H << h[0], h[1], h[2],
		h[3], h[4], h[5],
		h[6], h[7], h[8];
	H /= h[8];
	return H;
}


//转化坐标点
void ConvertPoint(int& x, int& y, int Width, int Height)
{
	x = x > 0 ? (x < Width ? x : Width - 1) : 0;
	y = y > 0 ? (y < Height ? y : Height - 1) : 0;
}


/**
 * 单点的rgb双线性插值转换
 * x y 所求点坐标
 * x1 y1 x2 y2 四个角的坐标
 * valij 对应(xi,yj)点的值
 */
void Bilinear_Interpolation(double x, double y, int x1, int y1, int x2, int y2,
	uchar& target, uchar val11, uchar val12, uchar val21, uchar val22)
{
	target = val11 * (x2 - x) * (y2 - y)
		+ val21 * (x - x1) * (y2 - y)
		+ val12 * (x2 - x) * (y - y1)
		+ val22 * (x - x1) * (y - y1);
}


void getRGB(const Mat& img, int x, int y, uchar& b, uchar& g, uchar& r)
{
	b = img.at<Vec3b>(y, x)[0];
	g = img.at<Vec3b>(y, x)[1];
	r = img.at<Vec3b>(y, x)[2];
}


/**
 * 双线插值
 * \param Width Height 图像长宽，临界值
 * \param raw_x raw_y 待转换坐标点
 * \param b g r 目标坐标r g b值
 */
void ConvertPoint(const Mat& img, int Width, int Height, double raw_x, double raw_y, uchar& b, uchar& g, uchar& r)
{
	int x1 = (int)raw_x, y1 = (int)raw_y;
	int x2 = x1 + 1, y2 = y1 + 1;
	ConvertPoint(x1, y1, Width, Height);
	ConvertPoint(x2, y2, Width, Height);
	/*
	 * 双线插值 
	 * Q11(x1, y1)
	 * Q12(x1, y2)
	 * Q21(x2, y1)
	 * Q22(x2, y2)
	 */
	uchar b11, b12, b21, b22;
	uchar g11, g12, g21, g22;
	uchar r11, r12, r21, r22;
	getRGB(img, x1, y1, b11, g11, r11);
	getRGB(img, x1, y2, b12, g12, r12);
	getRGB(img, x2, y1, b21, g21, r21);
	getRGB(img, x2, y2, b22, g22, r22);
	Bilinear_Interpolation(raw_x, raw_y, x1, y1, x2, y2, b, b11, b12, b21, b22);
	Bilinear_Interpolation(raw_x, raw_y, x1, y1, x2, y2, g, g11, g12, g21, g22);
	Bilinear_Interpolation(raw_x, raw_y, x1, y1, x2, y2, r, r11, r12, r21, r22);
}
