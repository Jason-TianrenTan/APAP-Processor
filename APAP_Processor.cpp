#include "APAP_Processor.h"
#include "MathUtils.h"
/*
 * \param points 特征点集
 * \param x y x*的坐标
 */
using namespace cv;
using namespace std;
using namespace Eigen;
MatrixXd calculate_Wi_forPoint(vector<Point2d> points, double x, double y)
{
	const double sigma_squared = sigma * sigma;
	MatrixXd Wi(2 * points.size(), 2 * points.size());
	Wi.setZero();
	for (size_t i = 0; i < points.size(); i++)
	{
		double u = (double)points[i].x, v = (double)points[i].y;
		double sqr_dist = getSqrDist(x, y, u, v);
		double candidate = exp(-sqr_dist / sigma_squared);
		double omega_i = max(candidate, gamma);
		Wi(i * 2, i * 2) = omega_i;
		Wi(i * 2 + 1, i * 2 + 1) = omega_i;
	}
	return Wi;
}

//计算每一个x*对应的*i
void calculate_Wi_Matrices(Mat img, vector<Point2d> obj, vector<MatrixXd>& vec)
{
	int Width = img.size().width, Height = img.size().height;
	ArrayXd heightArray = ArrayXd::LinSpaced(C1 + 1, 0, Height - 1),
		widthArray = ArrayXd::LinSpaced(C2 + 1, 0, Width - 1);//分割
	for (int i = 0; i < C1; i++) {
		double y = (heightArray(i) + heightArray(i + 1)) / 2;
		for (int j = 0; j < C2; j++) {
			double x = (widthArray(j) + widthArray(j + 1)) / 2;
			MatrixXd Wi = calculate_Wi_forPoint(obj, x, y);
			vec.push_back(Wi);
		}
	}

}

//每个分块的H*
vector<Matrix3d> calculate_CellHomography(vector<MatrixXd>& matrices, MatrixXd& A)
{
	vector<Matrix3d> H_vec;
	for (size_t i = 0; i < matrices.size(); i++)
	{
		MatrixXd WA = matrices[i] * A;
		Matrix3d H;
		JacobiSVD<MatrixXd> svd(WA, ComputeThinU | ComputeThinV);
		MatrixXd V = svd.matrixV();
		VectorXd h = V.col(V.cols() - 1);
		H << h[0], h[1], h[2],
			h[3], h[4], h[5],
			h[6], h[7], h[8];
		H_vec.push_back(H);
	}
	return H_vec;
}


GridBox** getIndex(const Mat& img, int C1, int C2, int& offset_x, int& offset_y, vector<Matrix3d> H_vec)
{
	GridBox** a = new GridBox*[C2 + 1];
	for (int i = 0; i < C2 + 1; i++)
		a[i] = new GridBox[C1 + 1];

	ArrayXf widthArray = ArrayXf::LinSpaced(C2 + 1, 0, img.size().width),
		heightArray = ArrayXf::LinSpaced(C1 + 1, 0, img.size().height); // 0 ~ C1 - 1, 0 ~ C2 - 1
	double min_x, min_y;
	for (int gy = 0; gy < C1; gy++)
		for (int gx = 0; gx < C2; gx++)
		{
			int H_index = gy * C2 + gx;
			double topleftx, toplefty,
				toprightx, toprighty,
				bottomleftx, bottomlefty,
				bottomrightx, bottomrighty;
			ConvertCoordinates(widthArray[gx], heightArray[gy], topleftx, toplefty, H_vec[H_index]);
			ConvertCoordinates(widthArray[gx + 1], heightArray[gy], toprightx, toprighty, H_vec[H_index]);
			ConvertCoordinates(widthArray[gx], heightArray[gy + 1], bottomleftx, bottomlefty, H_vec[H_index]);
			ConvertCoordinates(widthArray[gx + 1], heightArray[gy + 1], bottomrightx, bottomrighty, H_vec[H_index]);
			GridBox gbox = GridBox(Point2d(topleftx, toplefty),
									Point2d(toprightx, toprighty),
									Point2d(bottomleftx, bottomlefty),
									Point2d(bottomrightx, bottomrighty));
			a[gx][gy] = gbox;
		}
	return a;
}


void findGrid(int &gx, int &gy, double x, double y, GridBox** grids)
{
	for (int grid_x = 0; grid_x < C2; grid_x++)
		for (int grid_y = 0; grid_y < C1; grid_y++)
		{
			if (grids[grid_x][grid_y].contains(x, y))
			{
				gx = grid_x;
				gy = grid_y;
				return;
			}
		}
}


//横向(x)分成C2块 纵向(y)分成C1块
void ConvertImage(const Mat& img, Mat& target, vector<Matrix3d> H_vec, int C1, int C2)
{
	int Width = img.size().width, Height = img.size().height;
	int x_offset, y_offset;
	GridBox** grids = getIndex(img, C1, C2, x_offset, y_offset, H_vec);
	target = Mat::zeros(height, width, CV_8UC3);
	uchar b, g, r;
	int GridWidth = Width / C2, GridHeight = Height / C1;

	int min_x = 1000, min_y = 1000, max_x = -1, max_y = -1;

	bool debug = false;//debug
	bool appear[2500];
	for (int i = 0; i < 2500; i++)
		appear[i] = false;
	if (debug)
	{
		for (int y = 0; y < height; y++)
			for (int x = 0; x < width; x++)
			{
				int gx = x / C2, gy = y / C1;
				int H_index = gx + gy * C2;
				double d_nx, d_ny;
				ConvertCoordinates(x, y, d_nx, d_ny, H_vec[H_index].inverse());
				if (d_nx >= 0 && d_nx <= Width && d_ny >= 0 && d_ny <= Height)
				{
					ConvertPoint(img, Width, Height, d_nx, d_ny, b, g, r);
					int y_ = y, x_ = x;
					target.at<Vec3b>(y_, x_) = Vec3b(b, g, r);
				}
			}
	}
	else
	{
		for (int y = 0; y < height; y++)
			for (int x = 0; x < width; x++)
			{
				int grid_x = 0, grid_y = 0;
				findGrid(grid_x, grid_y, x, y, grids);
				int H_index = grid_x + grid_y * C2;
				appear[H_index] = true;
				double d_nx, d_ny;
				ConvertCoordinates(x, y, d_nx, d_ny, H_vec[H_index].inverse());
				if (d_nx >= 0 && d_nx <= Width && d_ny >= 0 && d_ny <= Height)
				{
					ConvertPoint(img, Width, Height, d_nx, d_ny, b, g, r);
					int y_ = y, x_ = x;
					target.at<Vec3b>(y_, x_) = Vec3b(b, g, r);
				}
			}

		for (int i = 0; i < 2500; i++)
			if (!appear[i])
				cout << "not appeared : " << i << endl;
	}
}


bool isBlack(const Mat& img, int x, int y, uchar& b, uchar& g, uchar& r)
{
	if (x >= img.size().width || y >= img.size().height)
		return true;
	Vec3b color = img.at<Vec3b>(y, x);
	b = color[0];
	g = color[1];
	r = color[2];
	if (b == 0 && g == 0 && r == 0)
		return true;
	return false;
}


uchar getWarpValue(uchar val1, uchar val2, int weight1, int weight2)
{
	return (val1 * weight1 + val2 * weight2) / (weight1 + weight2);
}


void warpImage(const Mat& image_1, const Mat& img_2, Mat& target)
{
	uchar b, g, r;
	uchar b1, g1, r1, b2, g2, r2;
	target = Mat::zeros(height, width, CV_8UC3);
	Mat img_1 = Mat::zeros(width, height, CV_8UC3);
	image_1.copyTo(img_1(Rect(0, 0, image_1.size().width, image_1.size().height)));
	for (int y = 0; y < height; y++)
		for (int x = 0; x < width; x++)
		{
			int weight_left = isBlack(img_1, x, y, b1, g1, r1) ? 0 : 1,
				weight_right = isBlack(img_2, x, y, b2, g2, r2) ? 0 : 1;
			if (weight_left + weight_right > 0)
			{
				b = getWarpValue(b1, b2, weight_left, weight_right);
				g = getWarpValue(g1, g2, weight_left, weight_right);
				r = getWarpValue(r1, r2, weight_left, weight_right);
				target.at<Vec3b>(y, x) = Vec3b(b, g, r);
			}
		}
	imshow("APAP target", target);
	waitKey(0);
}