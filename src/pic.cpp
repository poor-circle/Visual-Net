#include<opencv2/opencv.hpp>
#include<cstdio>
#define IMG_PARSE_DEBUG_MODE 1
namespace ImgParse
{
#ifdef IMG_PARSE_DEBUG_MODE
	int TestCaseNumber = 1;
#endif
	using namespace cv;
	using namespace std;
	constexpr float MaxQRBWRate = 2.0, MinQRBWRate = 0.5;//识别点黑白比例限制（理想1.0）
	constexpr int MinQRSize = 10, BlurSize = 3;         //最小识别点大小，模糊半径 
	constexpr float MaxQRScale = 0.25,MinQRXYRate=2.0/3.0,MaxQRXYRate=3.0/2.0;
			// 识别点长度占原图的最大比例，识别点的长宽比最小限制和最大限制
	Point RectCenter(vector<vector<Point> > contours, int i)
	{
		//找到所提取轮廓的中心点
		//在提取的中心小正方形的边界上每隔周长个像素提取一个点的坐标，求所提取四个点的平均坐标（即为小正方形的大致中心）
		int centerx = 0, centery = 0, n = contours[i].size();
		centerx = (contours[i][n / 4].x + contours[i][n * 2 / 4].x + contours[i][3 * n / 4].x + contours[i][n - 1].x) / 4;
		centery = (contours[i][n / 4].y + contours[i][n * 2 / 4].y + contours[i][3 * n / 4].y + contours[i][n - 1].y) / 4;
		return Point(centerx, centery);
	}
	double Cal3NumVariance(int a, int b, int c)
	{   //计算三个整数的方差
		double avg = (a + b + c) / 3.0;
		return (a - avg) * (a - avg) + (b - avg) * (b - avg) + (c - avg) * (c - avg);
		//其实应该除以三，但是都没除三也不影响比较大小。
	}
	Mat CropRect(const Mat& srcImg, RotatedRect& rotatedRect)
	{   //从图像中裁剪出一个矩形（可带角度）
		cv::Mat srcPoints, disImg;
		boxPoints(rotatedRect, srcPoints);//得到该矩阵的四点
		vector<Point2f> dis_points =
		{
			Point2f(0,rotatedRect.size.height - 1),
			Point2f(0,0),
			Point2f(rotatedRect.size.width - 1,0),
			Point2f(rotatedRect.size.width - 1,rotatedRect.size.height - 1)
		};//初始化目标矩阵的四点，其大小取决于初始矩阵（长宽不变）
		auto M = getPerspectiveTransform(srcPoints, dis_points); //计算变换矩阵
		warpPerspective(srcImg, disImg, M, rotatedRect.size); //进行透视变换以完成裁剪
#ifdef IMG_PARSE_DEBUG_MODE
		imshow("debug", disImg);
		waitKey(0);
#endif 
		return disImg;
	}
	bool IsQrBWRateLegal(float rate)
	{   //判断黑白比例是否合法
		return rate < MaxQRBWRate && rate > MinQRBWRate;
		// 理想情况rate=1.0，实际可能在一定范围内波动。
	}
	bool BWRatePreprocessing(Mat& image, vector<int>& vValueCount)
	{   //黑白条纹预处理函数
		int count = 0, nc = image.cols * image.channels(), nr = image.rows / 2;
		uchar lastColor = 0, * data = image.ptr<uchar>(nr);
		for (int i = 0; i < nc; i++)      //计算条纹数量以及各个条纹的像素数目
		{
			uchar color = data[i];
			if (color > 0)
				color = 255;
			if (i == 0)
			{
				lastColor = color;
				count++;
			}
			else
			{
				if (lastColor != color)
				{
					vValueCount.push_back(count);
					count = 0;
				}
				count++;
				lastColor = color;
			}
		}
		if (count) vValueCount.push_back(count);
		bool ans = vValueCount.size() >= 5;
#ifdef IMG_PARSE_DEBUG_MODE
		printf(ans ? "Preprocess Passed!" : "Preprocess Failed!");
#endif   
		return ans; //二维码未通过预处理（条纹不够黑白黑白黑的5条）
	}
	bool IsQrBWRateXLabel(Mat& image)
	{
		////计算二维码识别点的XLabel的黑白比例是否满足要求
#ifdef IMG_PARSE_DEBUG_MODE
		printf("%c Labels:", (TestCaseNumber & 1) ? ('Y') : ('X'));
#endif 
		vector<int> vValueCount;
		if (!BWRatePreprocessing(image, vValueCount)) //未通过预处理，不是识别点
			return false;
		//横向黑白比例1:1:3:1:1
		int index = -1, maxCount = -1;
		for (int i = 0; i < vValueCount.size(); i++)
		{
			if (i == 0)
			{
				index = i;
				maxCount = vValueCount[i];
			}
			else if (vValueCount[i] > maxCount)
			{
				index = i;
				maxCount = vValueCount[i];
			}
		}
		//左边 右边 都有两个值，才行
		if (index < 2 || (vValueCount.size() - index) < 3)
			return false;
		//黑白比例1:1:3:1:1测试

		float rate = ((float)maxCount) / 3.00;//以中间的比例3为基准
		bool checkTag = 1;
#ifdef IMG_PARSE_DEBUG_MODE
		printf("BWRate: "); //正常应该是1：1：3：1：1
#endif
		for (int i = -2; i < 3; ++i)
		{
			float rateNow = vValueCount[index + i] / rate;
#ifdef IMG_PARSE_DEBUG_MODE
			printf("%f ", rateNow);
#endif
			if (i) checkTag &= IsQrBWRateLegal(rateNow);
		}
		return checkTag;
	}
	Mat Rotation_90(const Mat& srcImg)
	{   //返回指定矩阵90度后的拷贝
		Mat tempImg;
		transpose(srcImg, tempImg);
		flip(tempImg, tempImg, 1);
		return tempImg;
	}
	bool IsQrBWRate(Mat& image)
	{   //计算二维码识别点的横纵黑白比例是否满足要求
#ifdef IMG_PARSE_DEBUG_MODE
		printf("\t");
#endif
		//计算X轴的比例
		bool xTest = IsQrBWRateXLabel(image);
		if (!xTest)
		{
#ifdef IMG_PARSE_DEBUG_MODE
			puts("\nFailed!");
#endif
			return false;
		}
#ifdef IMG_PARSE_DEBUG_MODE
		++TestCaseNumber;
		printf("\n\t");
#endif
		//矩阵旋转90度以计算Y轴比例
		Mat image_rotation_90 = Rotation_90(image);
		bool yTest = IsQrBWRateXLabel(image_rotation_90);
#ifdef IMG_PARSE_DEBUG_MODE
		puts(yTest ? "\nPass!" : "\nFailed!");
#endif
		return yTest;
	}
	bool IsQrSizeLegal(Size2f& qrSize, Size2f&& imgSize)
	{
		float xYScale = qrSize.height / qrSize.width;
#ifdef IMG_PARSE_DEBUG_MODE
		bool ans = qrSize.height >= MinQRSize && qrSize.width >= MinQRSize;
		ans &= qrSize.height / imgSize.height < MaxQRScale && qrSize.width / imgSize.width < MaxQRScale;
		ans &= xYScale <= MaxQRXYRate && xYScale >= MinQRXYRate;
		printf("Xsize:%.0f Ysize:%.0f XScale:%.2f%% YScale:%.2f%% xYScale:%.2f%% ",
			qrSize.height, qrSize.width,
			qrSize.height / imgSize.height * 100.0,
			qrSize.width / imgSize.width * 100.0,
			xYScale* 100.0
		);
		puts(ans?"Size test Passed!":"Size test Failed!");
		return ans;
#else
		if (qrSize.height < MinQRSize || qrSize.width < MinQRSize) //判断宽度和长度是否太小
			return false;
		if (qrSize.height / imgSize.height >= MaxQRScale || qrSize.width / imgSize.width >= MaxQRScale)
			return false;                                          //判断相对原图所占的比例是否太大
		if (xYScale < MinQRXYRate || xYScale > MaxQRXYRate)        //判断长宽比是否失衡
			return false;
#endif
	}
	bool IsQrPoint(vector<Point>& contour, const Mat& img)
	{   //本函数判断输入的vector<Point>是否是二维码识别点
#ifdef IMG_PARSE_DEBUG_MODE
		TestCaseNumber+=(TestCaseNumber&1)?(1):(2); //计数器控制
		printf("\nPOINT %d:\n\t", TestCaseNumber / 2);
#endif
		RotatedRect rotatedRect = minAreaRect(contour);
		//计算最小覆盖矩形
		cv::Mat cropImg = CropRect(img, rotatedRect);
		//将二维码从整个图上抠出来
		if (!IsQrSizeLegal(rotatedRect.size, img.size())) return false;
		//判断尺寸是否合法
		return IsQrBWRate(cropImg);
		//判断黑白比例是否合法
	}
	Mat ImgPreprocessing(const Mat& srcImg)
	{   //输入图像预处理函数
		Mat tmpImg;
		//彩色图转灰度图
		cvtColor(srcImg, tmpImg, COLOR_BGR2GRAY);
#ifdef IMG_PARSE_DEBUG_MODE
		imshow("Gray_output", tmpImg);
		waitKey(0);
#endif		
		//模糊全图，减少高频信息的干扰（尤其是摩尔纹）
		//实际上摩尔纹去除似乎还有更好的办法，考虑去掉一些高频率信息？
		blur(tmpImg, tmpImg, Size(BlurSize, BlurSize));
#ifdef IMG_PARSE_DEBUG_MODE
		imshow("Blur_output", tmpImg);
		waitKey(0);
#endif		
		//二值化
		threshold(tmpImg, tmpImg, 0, 255, THRESH_BINARY | THRESH_OTSU);
#ifdef IMG_PARSE_DEBUG_MODE
		imshow("Threshold_output", tmpImg);
		waitKey(0);
#endif
		return tmpImg;
	}
	bool ScreenQrPoint(const Mat& srcImg, vector<vector<Point>>& qrPoints)
	{
		//调用查找轮廓函数
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
		findContours(srcImg, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE, Point(0, 0));
		//通过黑色定位角作为父轮廓，有两个子轮廓的特点，筛选出三个定位角

		int parentIdx = -1;
		int ic = 0;

		for (int i = 0; i < contours.size(); i++)
		{
			if (hierarchy[i][2] != -1 && ic == 0)
			{
				parentIdx = i;
				ic++;
			}
			else if (hierarchy[i][2] != -1)
			{
				ic++;
			}
			else if (hierarchy[i][2] == -1)
			{
				ic = 0;
				parentIdx = -1;
			}
			if (ic >= 2)
			{
				bool isQrPoint = IsQrPoint(contours[parentIdx], srcImg);
				//保存找到的三个黑色定位角
				if (isQrPoint)
					qrPoints.push_back(contours[parentIdx]);
				ic = 0;
				parentIdx = -1;
			}
		}
#ifdef IMG_PARSE_DEBUG_MODE
		printf("Find %d Points!",(int)qrPoints.size());
		printf(qrPoints.size() < 3?"It's too less,Screen Failed!": "Screen Succeed!");
#endif // IMG_PARSE_DEBUG_MODE
		return qrPoints.size() >= 3;
	}
	void DumpExcessQrPoint(vector<vector<Point>>& qrPoints)
	{	//目前的实现：排序后计算面积最接近的三个点
		//可能以后有更好的实现？？
		sort(
			qrPoints.begin(), qrPoints.end(), 
			[](const vector<Point>& a, const vector<Point>& b) {return a.size() < b.size(); }
		);
		//按面积排序
		double mindis = INFINITY;
		int pos = -1;
		for (int i = 2; i < qrPoints.size(); ++i)
		{
			auto temp = Cal3NumVariance(qrPoints[i].size(), qrPoints[i - 1].size(), qrPoints[i - 2].size());
			if (mindis > temp)
			{
				mindis = temp;
				pos = i;
			}
		}
		//找到面积方差最小的三个点
		vector<vector<Point>> temp = { qrPoints[pos - 2],qrPoints[pos - 1],qrPoints[pos] };
		qrPoints.swap(temp);
		//清除多余的点
	}
	bool Find3QrPoint(const Mat& srcImg, vector<vector<Point>>& qrPoints)
	{
		//图像预处理,然后扫描定位点
		if (!ScreenQrPoint(ImgPreprocessing(srcImg), qrPoints)) return false;
		//如果定位点少于三个返回false，否则返回true
		if (qrPoints.size()!=3) DumpExcessQrPoint(qrPoints);
		//如果定位点多于三个，需要抛弃一些定位点
		return true;
	}
	void __DisPlayFind3QrPoint(const char* ImgPath)
	{
		Mat srcImg= imread(ImgPath, 1);
		imshow("原始", srcImg);
		vector<vector<Point>> qrPoints;
		if (Find3QrPoint(srcImg, qrPoints))
		{
			for (int i = 0; i < qrPoints.size(); i++)
				drawContours(srcImg, qrPoints, i, CV_RGB(0, 255, 0), -1);
			//填充定位点
			Point point[3];
			for (int i = 0; i < qrPoints.size(); i++)
				point[i] = RectCenter(qrPoints, i);
			line(srcImg, point[0], point[1], Scalar(0, 0, 255), 2);
			line(srcImg, point[1], point[2], Scalar(0, 0, 255), 2);
			line(srcImg, point[0], point[2], Scalar(0, 0, 255), 2);
			//连接定位点
			imshow("结果", srcImg);
		}
		waitKey(0);
	}
}
int main(int argc, char** argv[])
{
	ImgParse::__DisPlayFind3QrPoint("test.png");
	return 0;
}