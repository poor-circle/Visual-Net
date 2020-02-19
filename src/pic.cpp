#include<opencv2/opencv.hpp>
#include<cstdio>
//#define FIND_QRPOINT_DEBUG 1
namespace ImgParse
{

	using namespace cv;
	using namespace std;
	constexpr float MinRightAngel = 75.0, MaxRightAngel = 105.0;
	Mat Rotation_90(const Mat& srcImg)
	{   //返回指定矩阵90度后的拷贝
		Mat tempImg;
		transpose(srcImg, tempImg);
		flip(tempImg, tempImg, 1);
		return tempImg;
	}
	Point CalRectCenter(const vector<Point>& contours)
	{
		//找到所提取轮廓的中心点
		//在提取的中心小正方形的边界上每隔周长个像素提取一个点的坐标，求所提取四个点的平均坐标（即为小正方形的大致中心）
		int centerx = 0, centery = 0, n = contours.size();
		centerx = (contours[n / 4].x + contours[n * 2 / 4].x + contours[3 * n / 4].x + contours[n - 1].x) / 4;
		centery = (contours[n / 4].y + contours[n * 2 / 4].y + contours[3 * n / 4].y + contours[n - 1].y) / 4;
		return Point(centerx, centery);
	}
	bool IsClockWise(const Point& basePoint, const Point& point1, const Point& point2) 
	{   //判断point1和point2的顺逆时针关系
		float ax = point1.x - basePoint.x, ay = point1.y - basePoint.y;
		float bx = point2.x - basePoint.x, by = point2.y - basePoint.y;
#ifdef FIND_QRPOINT_DEBUG
		puts(((ax * by - bx * ay) > 0) ? "ClockWise" : "Anti-ClockWise");
#endif 
		return (ax * by - bx * ay) > 0;
		//如果点2位于点1的顺时针方向，返回真，否则返回假
	}
	float Cal3PointAngle(const Point& point0, const Point& point1, const Point& point2)
	{
		float dx1 = point1.x - point0.x, dy1 = point1.y - point0.y;
		float dx2 = point2.x - point0.x, dy2 = point2.y - point0.y;
		return acos((dx1 * dx2 + dy1 * dy2) / sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10f)) * 180.0f / 3.141592653f;
	}
	Mat CropRect(const Mat& srcImg, const RotatedRect& rotatedRect)
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
#ifdef FIND_QRPOINT_DEBUG
		imshow("debug", disImg);
		waitKey(0);
#endif 
		return disImg;
	}
	float distance(const Point& a, const Point& b)
	{
		return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
	}
	Point CalForthPoint(const array<Point, 3>& poi3)
	{
		return Point(poi3[2].x+poi3[1].x-poi3[0].x, poi3[2].y + poi3[1].y - poi3[0].y);
	}
	Mat CropParallelRect(const Mat& srcImg, vector<Point2f>& srcPoints, float bias=0.0)
	{   //从图像中裁剪出一个平行四边形
		cv::Mat disImg;

		float dis0 = distance(srcPoints[0], srcPoints[1]), dis1 = distance(srcPoints[1], srcPoints[3]);
		float rate = dis1 / dis0;
		float x1 = srcPoints[0].x - srcPoints[2].x, y1 = srcPoints[0].y- srcPoints[2].y;
		float x2 = (srcPoints[0].x - srcPoints[1].x)*rate, y2 = (srcPoints[0].y - srcPoints[1].y)*rate;
		float totx = x1 + x2, toty = y1 + y2, distot = sqrt(totx * totx + toty * toty);
		totx = totx / distot*bias ,toty= toty / distot *bias ;
		int id[4] = { 0,1,3,2 };
		//偏置向量计算
		for (int i = 0; i < 4; ++i)
		{
			Point temp = srcPoints[id[i]];
			srcPoints[id[i]].x += totx;
			srcPoints[id[i]].y += toty;
			std::swap(totx, toty);
			totx *= -1;
#ifdef CropParallelRect_DEBUG 
			line(srcImg, temp, srcPoints[id[i]], CV_RGB(255, 0, 0), 2);
			imshow("Debug", srcImg);
			waitKey(0);
#endif
		}
		
		Size size = Size(distance(srcPoints[0], srcPoints[1]), distance(srcPoints[1], srcPoints[3]));
		vector<Point2f> dis_points =
		{
			Point2f(0,0),
			Point2f(size.width - 1,0),
			Point2f(0,size.height - 1),
			Point2f(size.width - 1,size.height - 1)
		};//初始化目标矩阵的四点，其大小取决于初始矩阵（长宽不变）
		auto M = getPerspectiveTransform(srcPoints, dis_points); //计算变换矩阵
		warpPerspective(srcImg, disImg, M, size); //进行透视变换以完成裁剪
#ifdef FIND_QRPOINT_DEBUG
		imshow("debug", disImg);
		waitKey(0);
#endif 
		return disImg;
	}
	Mat CropParallelRect(const Mat& srcImg, const array<Point, 3>& poi3, float bias = 0.0)
	{
		vector<Point2f> srcPoints =
		{
			poi3[0],
			poi3[1],
			poi3[2],
			CalForthPoint(poi3)
		};
		return CropParallelRect(srcImg, srcPoints, bias);
	}
	bool isRightlAngle(float angle)
	{
		return MinRightAngel <= angle && MaxRightAngel >= angle;
	}
	namespace QrcodeParse
	{
#ifdef FIND_QRPOINT_DEBUG
		int TestCaseNumber = 1;
#endif
		struct ParseInfo
		{
			Point Center;
			RotatedRect Rect;
			ParseInfo(const vector<Point> &pointSet):
				Center(CalRectCenter(pointSet)),
				Rect(minAreaRect(pointSet)){}
			ParseInfo() = default;
		};
		constexpr float MaxQRBWRate = 2.22, MinQRBWRate = 0.45;//识别点黑白比例限制（理想1.0）
		constexpr int MinQRSize = 10, BlurSize = 1;//最小识别点大小，模糊半径 
		constexpr float MaxQRScale = 0.25, MinQRXYRate = 9.0 / 10.0, MaxQRXYRate = 10.0 / 9.0;
		double Cal3NumVariance(const int a, const int b, const int c)
		{   //计算三个整数的方差
			double avg = (a + b + c) / 3.0;
			return (a - avg) * (a - avg) + (b - avg) * (b - avg) + (c - avg) * (c - avg);
			//其实应该除以三，但是都没除三也不影响比较大小。
		}
		bool IsQrBWRateLegal(const float rate)
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
	#ifdef FIND_QRPOINT_DEBUG
			printf(ans ? "Preprocess Passed!" : "Preprocess Failed!");
	#endif   
			return ans; //二维码未通过预处理（条纹不够黑白黑白黑的5条）
		}
		bool IsQrBWRateXLabel(Mat& image)
		{
			////计算二维码识别点的XLabel的黑白比例是否满足要求
	#ifdef FIND_QRPOINT_DEBUG
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
	#ifdef FIND_QRPOINT_DEBUG
			printf("BWRate: "); //正常应该是1：1：3：1：1
	#endif
			for (int i = -2; i < 3; ++i)
			{
				float rateNow = vValueCount[index + i] / rate;
	#ifdef FIND_QRPOINT_DEBUG
				printf("%f ", rateNow);
	#endif
				if (i) checkTag &= IsQrBWRateLegal(rateNow);
			}
			return checkTag;
		}
		bool IsQrBWRate(Mat& image)
		{   //计算二维码识别点的横纵黑白比例是否满足要求
	#ifdef FIND_QRPOINT_DEBUG
			printf("\t");
	#endif
			//计算X轴的比例
			bool xTest = IsQrBWRateXLabel(image);
			if (!xTest)
			{
	#ifdef FIND_QRPOINT_DEBUG
				puts("\nFailed!");
	#endif
				return false;
			}
	#ifdef FIND_QRPOINT_DEBUG
			++TestCaseNumber;
			printf("\n\t");
	#endif
			//矩阵旋转90度以计算Y轴比例
			Mat image_rotation_90 = Rotation_90(image);
			bool yTest = IsQrBWRateXLabel(image_rotation_90);
	#ifdef FIND_QRPOINT_DEBUG
			puts(yTest ? "\nPass!" : "\nFailed!");
	#endif
			return yTest;
		}
		bool IsQrSizeLegal(const Size2f& qrSize, const Size2f&& imgSize)
		{
			float xYScale = qrSize.height / qrSize.width;
	#ifdef FIND_QRPOINT_DEBUG
			bool ans = qrSize.height >= MinQRSize && qrSize.width >= MinQRSize;
			ans &= qrSize.height / imgSize.height < MaxQRScale && qrSize.width / imgSize.width < MaxQRScale;
			ans &= xYScale <= MaxQRXYRate && xYScale >= MinQRXYRate;
			printf("Xsize:%.0f Ysize:%.0f XScale:%.2f%% YScale:%.2f%% xYScale:%.2f%% ",
				qrSize.height, qrSize.width,
				qrSize.height / imgSize.height * 100.0,
				qrSize.width / imgSize.width * 100.0,
				xYScale * 100.0
			);
			puts(ans ? "Size test Passed!" : "Size test Failed!");
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
		bool IsQrPoint(const vector<Point>& contour, const Mat& img)
		{   //本函数判断输入的vector<Point>是否是二维码识别点
	#ifdef FIND_QRPOINT_DEBUG
			TestCaseNumber += (TestCaseNumber & 1) ? (1) : (2); //计数器控制
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
	#ifdef FIND_QRPOINT_DEBUG
			imshow("Gray_output", tmpImg);
			waitKey(0);
	#endif		
			//模糊全图，减少高频信息的干扰（尤其是摩尔纹）
			//实际上摩尔纹去除似乎还有更好的办法，考虑去掉一些高频率信息？
			blur(tmpImg, tmpImg, Size(BlurSize, BlurSize));
	#ifdef FIND_QRPOINT_DEBUG
			imshow("Blur_output", tmpImg);
			waitKey(0);
	#endif		
			//二值化
			threshold(tmpImg, tmpImg, 0, 255, THRESH_BINARY | THRESH_OTSU);
	#ifdef FIND_QRPOINT_DEBUG
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
	#ifdef FIND_QRPOINT_DEBUG
			printf("Find %d Points!", (int)qrPoints.size());
			printf(qrPoints.size() < 3 ? "Points is too less,Screen Failed!" : "\n");
	#endif // FIND_QRPOINT_DEBUG
			return qrPoints.size() >= 3;
		}
		bool isRightAngleExist(const Point& point0, const Point& point1, const Point& point2)
		{
	#ifdef FIND_QRPOINT_DEBUG
			static int counter = 0;
			float angle0 = Cal3PointAngle(point0, point1, point2), angle1 = Cal3PointAngle(point1, point0, point2), angle2 = Cal3PointAngle(point2, point0, point1);
			bool flag2 = isRightlAngle(angle0) || isRightlAngle(angle1) || isRightlAngle(angle2);
			printf("\nDumpPoints: %d angle0: %.2f angle1: %.2f,angle2: %.2f %s\n", ++counter,angle0,angle1,angle2,flag2?"Passed!":"Failed!");
	#endif
			return isRightlAngle(Cal3PointAngle(point0, point1, point2)) ||
				isRightlAngle(Cal3PointAngle(point1, point0, point2)) ||
				isRightlAngle(Cal3PointAngle(point2, point0, point1));
		}
		bool DumpExcessQrPoint(vector<vector<Point>>& qrPoints)
		{	//目前的实现：排序后计算面积存在直角的方差最接近的三个点
			//可能以后有更好的实现？？
			sort(
				qrPoints.begin(), qrPoints.end(),
				[](const vector<Point>& a, const vector<Point>& b) {return a.size() < b.size(); }
			);
			//按面积排序
			double mindis = INFINITY;
			int pos = -1;
			Point Point0 = CalRectCenter(qrPoints[0]), Point1 = CalRectCenter(qrPoints[1]),Point2;
			for (int i = 2; i < qrPoints.size(); ++i)
			{
				bool tag = 0;
				if (!isRightAngleExist(Point2 = CalRectCenter(qrPoints[i]), Point1, Point0))
					tag = 1;
				if (!tag)
				{
					auto temp = Cal3NumVariance(qrPoints[i].size(), qrPoints[i - 1].size(), qrPoints[i - 2].size());
					if (mindis > temp)
					{
						mindis = temp;
						pos = i;
					}
				}
				Point0 = Point1;
				Point1 = Point2;
			}
			//如果pos==-1，则按大小排序后不存在夹角90度左右的识别点。
			if (pos == -1) return 0; 
			else
			{
				vector<vector<Point>> temp = { qrPoints[pos - 2],qrPoints[pos - 1],qrPoints[pos] };
				qrPoints.swap(temp);
				return 1;
			}
			//如不等于1，则找到成直角且面积方差最小的三个点
			//清除多余的点
		}
		void AdjustPointsOrder(array<vector<Point>, 3>& src3Points)
		{
			array<vector<Point>, 3> temp;
			Point p3[3] = { CalRectCenter(src3Points[0]),CalRectCenter(src3Points[1]),CalRectCenter(src3Points[2])};
			int index[3][3] = { { 0,1,2 },{1,0,2},{2,0,1} };
			for (int i = 0; i < 3; i++)
			{
				if (isRightlAngle(Cal3PointAngle(p3[index[i][0]], p3[index[i][1]], p3[index[i][2]])))
				{
					temp[0] = src3Points[index[i][0]];     //左上角的点位于0号 Red
					if (IsClockWise(p3[index[i][0]], p3[index[i][1]], p3[index[i][2]]))//判断1号点和2号点的顺逆时针关系
					{                                      
						temp[1] = src3Points[index[i][1]]; //右上角的点位于1号 Green
						temp[2] = src3Points[index[i][2]]; //左下角的点位于2号 Blue
					}
					else
					{
						temp[1] = src3Points[index[i][2]];
						temp[2] = src3Points[index[i][1]];
					}
				}
			}
			src3Points.swap(temp);
			return;
		}
		bool Get3Points(const Mat& srcImg, array<vector<Point>, 3> &qr3Points)
		{
			vector<vector<Point>> qrPoints;
			//图像预处理,然后扫描定位点
			if (!ScreenQrPoint(ImgPreprocessing(srcImg), qrPoints)) return 0;
			//如果定位点少于三个返回false，否则返回true
			if (qrPoints.size() > 3 && !DumpExcessQrPoint(qrPoints)) return 0;
			qr3Points = {qrPoints[0],qrPoints[1],qrPoints[2]};
			AdjustPointsOrder(qr3Points);
			return 1;
		}
		bool Get3Points(const Mat& srcImg, array<ParseInfo,3>& Points3Info)
		{
			array<vector<Point>, 3> qr3Points;
			if (!Get3Points(srcImg, qr3Points)) return 0;
			Points3Info = { qr3Points[0],qr3Points[1], qr3Points[2]};
			return 1;
		}
		bool Main(const Mat& srcImg, Mat& disImg, array<ParseInfo, 3>& Points3Info)
		{
			if (!Get3Points(srcImg, Points3Info)) return 1;
			auto poi3 = array<Point, 3>{Points3Info[0].Center, Points3Info[1].Center, Points3Info[2].Center};
			float avglen = 0.0;
			for (auto& e : Points3Info)
			{
				avglen += e.Rect.size.height + e.Rect.size.width;
			}
			avglen /= 6.0*sqrt(2.0)*0.8;

			disImg=CropParallelRect(srcImg,poi3, avglen);
			imshow("debug", disImg);
			waitKey(0);
			//if (!Get3Points(disImg, Points3Info)) return 1;
			//avglen = 0.0;
			//for (auto& e : Points3Info)
			//{
			//	avglen += e.Rect.size.height + e.Rect.size.width;
			//}
			//avglen /= 6.0 * sqrt(2.0);
			//poi3 = array<Point, 3>{Points3Info[0].Center, Points3Info[1].Center, Points3Info[2].Center};
			//disImg = CropParallelRect(disImg, poi3, avglen);
			//imshow("debug2", disImg);
			//waitKey(0);
			return 0;
		}
		// 识别点长度占原图的最大比例，识别点的长宽比最小限制和最大限制
		void __DisPlay(const char* ImgPath)
		{
			Mat srcImg = imread(ImgPath, 1);
			imshow("原始", srcImg);
			array<vector<Point>, 3> qrPoints;
			if (Get3Points(srcImg, qrPoints))
			{
				for (int i = 0, C = 0x00FF0000; i < qrPoints.size(); ++i,C>>=8)
					drawContours(srcImg, vector<vector<Point>>{qrPoints[i]}, 0, CV_RGB(C >> 16, (C >> 8) & 0xFF, C & 0xFF), -1);
				//填充定位点
				//左上角的点显示为红色，右上角的点显示为绿色，左下角的点显示为蓝色
				Point point[3];
				for (int i = 0; i < qrPoints.size(); i++)
					point[i] = CalRectCenter(qrPoints[i]);
				line(srcImg, point[0], point[1], Scalar(255, 255, 0), 2);
				line(srcImg, point[1], point[2], Scalar(0, 255, 255), 2);
				line(srcImg, point[0], point[2], Scalar(255, 0, 255), 2);
				//连接定位点
				imshow("结果", srcImg);
			}
			waitKey(0);
		}
		void __DisPlay2(const char* ImgPath)
		{
			Mat srcImg = imread(ImgPath, 1);
			imshow("原始", srcImg);
			array<ParseInfo, 3> qrPoints;
			if (Get3Points(srcImg, qrPoints))
			{
				line(srcImg, qrPoints[0].Center, qrPoints[1].Center, Scalar(0, 0, 255), 2);
				line(srcImg, qrPoints[1].Center, qrPoints[2].Center, Scalar(0, 255, 0), 2);
				line(srcImg, qrPoints[0].Center, qrPoints[2].Center, Scalar(255, 0, 0), 2);
				Point2f poi[4];
				for (auto& e : qrPoints)
				{
					e.Rect.points(poi);
					line(srcImg, poi[0], poi[1], Scalar(128, 128, 128), 2);
					line(srcImg, poi[1], poi[2], Scalar(128, 128, 128), 2);
					line(srcImg, poi[2], poi[3], Scalar(128, 128, 128), 2);
					line(srcImg, poi[3], poi[0], Scalar(128, 128, 128), 2);
				}
				//连接定位点
				imshow("结果", srcImg);
			}
			waitKey(0);
		}
		void __DisPlay3(const char* ImgPath)
		{
			Mat srcImg = imread(ImgPath, 1), disImg;
			array<ParseInfo, 3>  Points3Info;
			Main(srcImg, disImg, Points3Info);
		}
	}
	void CrapInfoArea(const array<Point, 3> &CenterPoints, const Mat& srcImg, Mat& disImg)
	{//TODO
		disImg = srcImg;
	}
	bool Main(const Mat& srcImg, Mat& disImg)
	{
		array<vector<Point>, 3> qrPoints;
		if (!QrcodeParse::Get3Points(srcImg, qrPoints)) return 0;
		array<Point, 3> CenterPoints= { CalRectCenter(qrPoints[0]), CalRectCenter(qrPoints[1]) , CalRectCenter(qrPoints[2]) };
		//AdjustPointsOrder(CenterPoints);
		CrapInfoArea(CenterPoints, srcImg,disImg);
		return 1;
	}
}
int main(int argc, char** argv[])
{
	cv::Mat srcImg,disImg;
	//srcImg = cv::imread("test7.jpg", 1);
	//imshow("原始", srcImg);

	ImgParse::QrcodeParse::__DisPlay3("test5.jpg");
	//ImgParse::Main(srcImg,disImg);
	return 0;
}