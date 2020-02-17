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
	constexpr float MaxQRBWRate = 2.0, MinQRBWRate = 0.5;//ʶ���ڰױ������ƣ�����1.0��
	constexpr int MinQRSize = 10, BlurSize = 3;         //��Сʶ����С��ģ���뾶 
	constexpr float MaxQRScale = 0.25,MinQRXYRate=2.0/3.0,MaxQRXYRate=3.0/2.0;
			// ʶ��㳤��ռԭͼ����������ʶ���ĳ������С���ƺ��������
	Point RectCenter(vector<vector<Point> > contours, int i)
	{
		//�ҵ�����ȡ���������ĵ�
		//����ȡ������С�����εı߽���ÿ���ܳ���������ȡһ��������꣬������ȡ�ĸ����ƽ�����꣨��ΪС�����εĴ������ģ�
		int centerx = 0, centery = 0, n = contours[i].size();
		centerx = (contours[i][n / 4].x + contours[i][n * 2 / 4].x + contours[i][3 * n / 4].x + contours[i][n - 1].x) / 4;
		centery = (contours[i][n / 4].y + contours[i][n * 2 / 4].y + contours[i][3 * n / 4].y + contours[i][n - 1].y) / 4;
		return Point(centerx, centery);
	}
	double Cal3NumVariance(int a, int b, int c)
	{   //�������������ķ���
		double avg = (a + b + c) / 3.0;
		return (a - avg) * (a - avg) + (b - avg) * (b - avg) + (c - avg) * (c - avg);
		//��ʵӦ�ó����������Ƕ�û����Ҳ��Ӱ��Ƚϴ�С��
	}
	Mat CropRect(const Mat& srcImg, RotatedRect& rotatedRect)
	{   //��ͼ���вü���һ�����Σ��ɴ��Ƕȣ�
		cv::Mat srcPoints, disImg;
		boxPoints(rotatedRect, srcPoints);//�õ��þ�����ĵ�
		vector<Point2f> dis_points =
		{
			Point2f(0,rotatedRect.size.height - 1),
			Point2f(0,0),
			Point2f(rotatedRect.size.width - 1,0),
			Point2f(rotatedRect.size.width - 1,rotatedRect.size.height - 1)
		};//��ʼ��Ŀ�������ĵ㣬���Сȡ���ڳ�ʼ���󣨳����䣩
		auto M = getPerspectiveTransform(srcPoints, dis_points); //����任����
		warpPerspective(srcImg, disImg, M, rotatedRect.size); //����͸�ӱ任����ɲü�
#ifdef IMG_PARSE_DEBUG_MODE
		imshow("debug", disImg);
		waitKey(0);
#endif 
		return disImg;
	}
	bool IsQrBWRateLegal(float rate)
	{   //�жϺڰױ����Ƿ�Ϸ�
		return rate < MaxQRBWRate && rate > MinQRBWRate;
		// �������rate=1.0��ʵ�ʿ�����һ����Χ�ڲ�����
	}
	bool BWRatePreprocessing(Mat& image, vector<int>& vValueCount)
	{   //�ڰ�����Ԥ������
		int count = 0, nc = image.cols * image.channels(), nr = image.rows / 2;
		uchar lastColor = 0, * data = image.ptr<uchar>(nr);
		for (int i = 0; i < nc; i++)      //�������������Լ��������Ƶ�������Ŀ
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
		return ans; //��ά��δͨ��Ԥ�������Ʋ����ڰ׺ڰ׺ڵ�5����
	}
	bool IsQrBWRateXLabel(Mat& image)
	{
		////�����ά��ʶ����XLabel�ĺڰױ����Ƿ�����Ҫ��
#ifdef IMG_PARSE_DEBUG_MODE
		printf("%c Labels:", (TestCaseNumber & 1) ? ('Y') : ('X'));
#endif 
		vector<int> vValueCount;
		if (!BWRatePreprocessing(image, vValueCount)) //δͨ��Ԥ��������ʶ���
			return false;
		//����ڰױ���1:1:3:1:1
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
		//��� �ұ� ��������ֵ������
		if (index < 2 || (vValueCount.size() - index) < 3)
			return false;
		//�ڰױ���1:1:3:1:1����

		float rate = ((float)maxCount) / 3.00;//���м�ı���3Ϊ��׼
		bool checkTag = 1;
#ifdef IMG_PARSE_DEBUG_MODE
		printf("BWRate: "); //����Ӧ����1��1��3��1��1
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
	{   //����ָ������90�Ⱥ�Ŀ���
		Mat tempImg;
		transpose(srcImg, tempImg);
		flip(tempImg, tempImg, 1);
		return tempImg;
	}
	bool IsQrBWRate(Mat& image)
	{   //�����ά��ʶ���ĺ��ݺڰױ����Ƿ�����Ҫ��
#ifdef IMG_PARSE_DEBUG_MODE
		printf("\t");
#endif
		//����X��ı���
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
		//������ת90���Լ���Y�����
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
		if (qrSize.height < MinQRSize || qrSize.width < MinQRSize) //�жϿ�Ⱥͳ����Ƿ�̫С
			return false;
		if (qrSize.height / imgSize.height >= MaxQRScale || qrSize.width / imgSize.width >= MaxQRScale)
			return false;                                          //�ж����ԭͼ��ռ�ı����Ƿ�̫��
		if (xYScale < MinQRXYRate || xYScale > MaxQRXYRate)        //�жϳ�����Ƿ�ʧ��
			return false;
#endif
	}
	bool IsQrPoint(vector<Point>& contour, const Mat& img)
	{   //�������ж������vector<Point>�Ƿ��Ƕ�ά��ʶ���
#ifdef IMG_PARSE_DEBUG_MODE
		TestCaseNumber+=(TestCaseNumber&1)?(1):(2); //����������
		printf("\nPOINT %d:\n\t", TestCaseNumber / 2);
#endif
		RotatedRect rotatedRect = minAreaRect(contour);
		//������С���Ǿ���
		cv::Mat cropImg = CropRect(img, rotatedRect);
		//����ά�������ͼ�Ͽٳ���
		if (!IsQrSizeLegal(rotatedRect.size, img.size())) return false;
		//�жϳߴ��Ƿ�Ϸ�
		return IsQrBWRate(cropImg);
		//�жϺڰױ����Ƿ�Ϸ�
	}
	Mat ImgPreprocessing(const Mat& srcImg)
	{   //����ͼ��Ԥ������
		Mat tmpImg;
		//��ɫͼת�Ҷ�ͼ
		cvtColor(srcImg, tmpImg, COLOR_BGR2GRAY);
#ifdef IMG_PARSE_DEBUG_MODE
		imshow("Gray_output", tmpImg);
		waitKey(0);
#endif		
		//ģ��ȫͼ�����ٸ�Ƶ��Ϣ�ĸ��ţ�������Ħ���ƣ�
		//ʵ����Ħ����ȥ���ƺ����и��õİ취������ȥ��һЩ��Ƶ����Ϣ��
		blur(tmpImg, tmpImg, Size(BlurSize, BlurSize));
#ifdef IMG_PARSE_DEBUG_MODE
		imshow("Blur_output", tmpImg);
		waitKey(0);
#endif		
		//��ֵ��
		threshold(tmpImg, tmpImg, 0, 255, THRESH_BINARY | THRESH_OTSU);
#ifdef IMG_PARSE_DEBUG_MODE
		imshow("Threshold_output", tmpImg);
		waitKey(0);
#endif
		return tmpImg;
	}
	bool ScreenQrPoint(const Mat& srcImg, vector<vector<Point>>& qrPoints)
	{
		//���ò�����������
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
		findContours(srcImg, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE, Point(0, 0));
		//ͨ����ɫ��λ����Ϊ�����������������������ص㣬ɸѡ��������λ��

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
				//�����ҵ���������ɫ��λ��
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
	{	//Ŀǰ��ʵ�֣��������������ӽ���������
		//�����Ժ��и��õ�ʵ�֣���
		sort(
			qrPoints.begin(), qrPoints.end(), 
			[](const vector<Point>& a, const vector<Point>& b) {return a.size() < b.size(); }
		);
		//���������
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
		//�ҵ����������С��������
		vector<vector<Point>> temp = { qrPoints[pos - 2],qrPoints[pos - 1],qrPoints[pos] };
		qrPoints.swap(temp);
		//�������ĵ�
	}
	bool Find3QrPoint(const Mat& srcImg, vector<vector<Point>>& qrPoints)
	{
		//ͼ��Ԥ����,Ȼ��ɨ�趨λ��
		if (!ScreenQrPoint(ImgPreprocessing(srcImg), qrPoints)) return false;
		//�����λ��������������false�����򷵻�true
		if (qrPoints.size()!=3) DumpExcessQrPoint(qrPoints);
		//�����λ�������������Ҫ����һЩ��λ��
		return true;
	}
	void __DisPlayFind3QrPoint(const char* ImgPath)
	{
		Mat srcImg= imread(ImgPath, 1);
		imshow("ԭʼ", srcImg);
		vector<vector<Point>> qrPoints;
		if (Find3QrPoint(srcImg, qrPoints))
		{
			for (int i = 0; i < qrPoints.size(); i++)
				drawContours(srcImg, qrPoints, i, CV_RGB(0, 255, 0), -1);
			//��䶨λ��
			Point point[3];
			for (int i = 0; i < qrPoints.size(); i++)
				point[i] = RectCenter(qrPoints, i);
			line(srcImg, point[0], point[1], Scalar(0, 0, 255), 2);
			line(srcImg, point[1], point[2], Scalar(0, 0, 255), 2);
			line(srcImg, point[0], point[2], Scalar(0, 0, 255), 2);
			//���Ӷ�λ��
			imshow("���", srcImg);
		}
		waitKey(0);
	}
}
int main(int argc, char** argv[])
{
	ImgParse::__DisPlayFind3QrPoint("test.png");
	return 0;
}