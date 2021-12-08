#include "SIFT_Matcher.h"
using namespace cv;
using namespace std;
using namespace Eigen;

void findFeaturePointsWithSIFT(Mat img_1, Mat img_2, vector<KeyPoint>& key_p1, vector<KeyPoint>& key_p2, vector<DMatch>& matches) {
	Mat des1, des2;
	//SIFT寻找特征点
	Ptr<Feature2D> f2d = xfeatures2d::SIFT::create();

	f2d->detect(img_1, key_p1);
	f2d->detect(img_2, key_p2);
	//Calculate descriptors (feature vectors)
	f2d->compute(img_1, key_p1, des1);
	f2d->compute(img_2, key_p2, des2);

	//Matching descriptor vector using BFMatcher
	FlannBasedMatcher matcher;
	matcher.match(des1, des2, matches);
}


/*
 convert 2d vector -> Mat
*/
Mat getMatDescriptor(vector<vector<vl_uint8>>& descriptors)
{
	Mat mat = Mat(descriptors.size(), 128, CV_32FC1);
	for (size_t i = 0; i < descriptors.size(); i++)
	{
		for (size_t j = 0; j < descriptors[i].size(); j++)
			mat.at<vl_uint8>(i, j) = descriptors[i][j];
	}
	return mat;
}


/**使用VLFeat检测特征点
 * \param img 图像
 * \param keypoints 特征点
 * \param descriptor 描述子
 */
void detectPoints(const Mat& img, vector<Point2f>& keypoints, Mat& Descriptor)
{
	vector<vector<vl_uint8>> descriptors;
	//vector<KPoint> KVec;
	int width = img.size().width, height = img.size().height;
	vl_sift_pix *ImageData = new vl_sift_pix[width * height];
	for (int i = 0; i < height; i++)
	{
		const uchar* Pixel = img.ptr<uchar>(i);
		for (int j = 0; j < width; j++)
			ImageData[i * width + j] = Pixel[j];
	}
	int octave = -1, nlevel = 3, o_min = 0;
	VlSiftFilt* SiftFilt = vl_sift_new(width, height, octave, nlevel, o_min);
	const vl_sift_pix* vlData_const = (vl_sift_pix*)ImageData;
	double edge_threshold = 500, peak_threshold = 0;
	vl_sift_set_peak_thresh(SiftFilt, peak_threshold);
	vl_sift_set_edge_thresh(SiftFilt, edge_threshold);

	int reserved = 0;
	bool flag = true;
	while (true)
	{
		int err = 0;
		if (flag)
		{
			flag = false;
			err = vl_sift_process_first_octave(SiftFilt, vlData_const);
		}
		else err = vl_sift_process_next_octave(SiftFilt);
		if (err == VL_ERR_EOF)
			break;

		//获取特征点
		vl_sift_detect(SiftFilt);
		VlSiftKeypoint* key_arr = SiftFilt->keys;
		for (int i = 0; i < SiftFilt->nkeys; i++)
		{
			VlSiftKeypoint* temp = key_arr + i;
			double angles[4];
			int angleCount = vl_sift_calc_keypoint_orientations(SiftFilt, angles, temp);
			keypoints.emplace_back(temp->x, temp->y);
			//TODO:添加KPoint
			for (int j = 0; j < angleCount; j++)
			{
				float* buffer = new float[128];
				vl_sift_calc_keypoint_descriptor(SiftFilt, buffer, temp, angles[j]);
				//KPoint kPoint = KPoint(Point2f(temp->x, temp->y), temp->sigma, M_PI/2 - angles[j]);
			//	KVec.emplace_back(kPoint);
				vector<vl_uint8> temp_desc;
				for (int q = 0; q < 128; q++)
				{
					float x = 512.f * buffer[q];
					x = (x < 255.f) ? x : 255.f;
					temp_desc.emplace_back((vl_uint8) x);
				}
				descriptors.emplace_back(temp_desc);

				delete[] buffer;
				buffer = NULL;
			}

		}
	}
	Descriptor = getMatDescriptor(descriptors);
	vl_sift_delete(SiftFilt);
	free(ImageData);
}


/** Using VLFeat
 * 使用VLFeat的SIFT匹配
 */
void findVLFeat_SIFT(const Mat& img_1, const Mat& img_2)
{
	vector<Point2f> keyPoints1, keyPoints2;
	Mat desc1, desc2;
	cout << "detect points" << endl;
	detectPoints(img_1, keyPoints1, desc1),
	detectPoints(img_2, keyPoints2, desc2);
	
	FlannBasedMatcher matcher;
	vector<DMatch> matches;
	matcher.match(desc1, desc2, matches);
	cout << "match = " << matches.size() << endl;
}


//-----------------以下仅用于测试-----------------
void drawMatch(Mat &img, const MatrixXf &match, const Matrix3f &inv_T1, const Matrix3f &inv_T2) {
	int height = img.size[0];
	int width = img.size[1];
	cout << "width = " << width << ", height = " << height << endl;
	int reference = height;
	if (width < height)
		reference = width;
	float circleRadius = reference / 200.f;
	float strokeSize = reference / 600.f;
	MatrixXf pts1 = (inv_T1*match.block(0, 0, match.rows(), 3).transpose()).transpose();
	MatrixXf pts2 = (inv_T2*match.block(0, 3, match.rows(), 3).transpose()).transpose();

	srand(time(0));
	for (int i = 0; i < match.rows(); i++) {
		int r = rand() % 256, g = rand() % 256, b = rand() % 256;
		Scalar color(r, g, b);
		Point p1 = Point(pts1.row(i)[0], pts1.row(i)[1]);
		Point p2 = Point(pts2.row(i)[0] + img.size[1] / 2, pts2.row(i)[1]);
		line(img, p1, p2, color, strokeSize);
		circle(img, p1, 4, color, strokeSize);
		circle(img, p2, 4, color, strokeSize);
	}
}


void normalizePts(MatrixXf &mat, Matrix3f &T) {

	float cx = mat.col(0).mean();
	float cy = mat.col(1).mean();
	mat.array().col(0) -= cx;
	mat.array().col(1) -= cy;

	float sqrt_2 = sqrt(2);
	float meandist = (mat.array().col(0)*mat.array().col(0) + mat.array().col(1)*mat.array().col(1)).sqrt().mean();
	float scale = sqrt_2 / meandist;
	mat.leftCols<2>().array() *= scale;

	T << scale, 0, -scale*cx, 0, scale, -scale*cy, 0, 0, 1;
}


void normalizeMatch(MatrixXf &mat, Matrix3f &T1, Matrix3f &T2) {
	MatrixXf pts1 = mat.leftCols<3>();
	MatrixXf pts2 = mat.block(0, 3, mat.rows(), 3);
	normalizePts(pts1, T1);
	normalizePts(pts2, T2);
	mat.leftCols<3>() = pts1;
	mat.block(0, 3, mat.rows(), 3) = pts2;
}


void combineMat(Mat &out, const Mat& left, const Mat& right) {
	int height = left.size[0];
	int width = left.size[1];
	out = Mat(height, 2 * width, CV_8UC3);
	for (int i = 0; i < height; i++)
		for (int j = 0; j < width * 2; j++)
			if (j < width)
				out.at<Vec3b>(i, j) = left.at<Vec3b>(i, j);
			else
				out.at<Vec3b>(i, j) = right.at<Vec3b>(i, j - width);
}


void displayMat(const Mat& display) {
	imshow("img", display);
	waitKey(0);
}