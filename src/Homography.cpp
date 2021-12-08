#include "Homography.h"
#include "MathUtils.h"
using namespace std;
using namespace cv;
using namespace Eigen;

void getHomography(vector<KeyPoint> obj_keyP, vector<KeyPoint> scene_keyP, vector<Point2d>& obj, vector<Point2d>& scene, Matrix3d& H, MatrixXd& A)
{
	int N = obj_keyP.size();
	for (size_t i = 0; i <N; i++)
	{
		obj.push_back(obj_keyP[i].pt);
		scene.push_back(scene_keyP[i].pt);
	}

	//计算Ai
	/*
	 * -x -y -1 0 0 0 ux uy u
	 * 0 0 0 -x -y -1 vx vy v
	 * 对A进行SVD分解，最小奇异值对应的右奇异向量就是h;
	 * 重新排列h得到H
	 */
	A.resize(2 * N, 9); //2N * 9
	for (size_t i = 0; i < N; i++) {
		//	int x = scene[i].x, y = scene[i].y;
		//  int u = obj[i].x, v = obj[i].y;
		double x = obj[i].x, y = obj[i].y;
		double u = scene[i].x, v = scene[i].y;
		A.row(i * 2) << 0, 0, 0, -x, -y, -1, v*x, v*y, v;
		A.row(i * 2 + 1) << x, y, 1, 0, 0, 0, -u*x, -u*y, -u;
	}

	//奇异值分解
	JacobiSVD<MatrixXd> svd(A, ComputeThinU | ComputeThinV);
	MatrixXd V = svd.matrixV();
	//求H
	VectorXd h = V.col(V.cols() - 1);
	H = rollVectorToH(h);
}