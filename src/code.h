#pragma once
#include<opencv2/opencv.hpp>

#include<cstdio>

namespace Code
{
	using namespace cv;
	using namespace std;
	enum class FrameType;
	uint16_t CalCheckCode(const char* info, int len);
	void BulidSafeArea(Mat& mat);
	void BulidQrPoint(Mat& mat);
	void BulidCheckCodeAndFrameNo(Mat& mat, uint16_t checkcode, uint8_t FrameNo);
	void BulidInfoRect(Mat& mat, const char* info, int len);
	void BulidFrameFlag(Mat& mat, FrameType frameType, int tailLen);
	Mat CodeFrame(FrameType frameType, const char* info, int tailLen, int FrameNo);
	void main(const char* info, int len,const char* savePath, const char* outputFormat);
}