#include"pic.h"
#include"code.h"
#include"ffmpeg.h"
#include"ImgDecode.h"
#define Show_Img(src) do\
{\
	cv::imshow("DEBUG", src);\
	cv::waitKey();\
}while (0);
void comp(cv::Mat & mat, const char* rawPath)
{
	cv::Mat rawImg = cv::imread(rawPath, 1);
	ImgParse::Resize(rawImg);
	//Show_Img(mat);
	int cnt = 0;
	for (int i = 0; i < 108; ++i)
	{
		for (int j = 0; j < 108; ++j)
		{
			auto temp = rawImg.at<cv::Vec3b>(i, j);
			auto temp2 = mat.at<cv::Vec3b>(i, j);
			if (temp != temp2)
			{
				printf("different NO.%d in %d,%d\n", ++cnt, i, j);
			}
		}
	}
}
int FileToVideo(const char* filePath, const char* videoPath)
{
	FILE* fp = fopen(filePath, "rb");
	if (fp == nullptr) return 1;
	fseek(fp, 0, SEEK_END);
	int size = ftell(fp);
	rewind(fp);
	char* temp = (char*)malloc(sizeof(char) * size);
	if (temp == nullptr) return 1;
	fread(temp, 1, size, fp);
	fclose(fp);
	//Code::Main(temp, size, "outputImg", "png");
	FFMPEG::ImagetoVideo("outputImg", "png", videoPath, 20, 60, 100000);
	free(temp);
	return 0;
}
int VideoToFile(const char* videoPath, const char* filePath)
{
	char imgName[256];
	if (FFMPEG::VideotoImage(videoPath, "inputImg", "jpg")) return 1;
	int precode = -1;
	std::vector<unsigned char> outputFile;
	bool hasStarted=0;
	int i = 0;
	for (i = 1;; ++i)
	{
		snprintf(imgName, 256, "inputImg\\%05d.jpg",i);
		FILE* fp  = fopen(imgName,"rb");
		if (fp == nullptr)
		{
			puts("eee");
			return 1;
		}
		fclose(fp);
		cv::Mat srcImg = cv::imread(imgName, 1),disImg;
		//cv::Mat disImg = cv::imread(imgName, 1);
		//ImgParse::Resize(disImg);
		if (ImgParse::Main(srcImg, disImg))
		{
			continue;
		}
	    //Show_Img(disImg);
		//comp(disImg, "00000.png");
		ImageDecode::ImageInfo imageInfo;
		bool ans = ImageDecode::Main(disImg, imageInfo);
		if (ans)
		{
			continue;
		}
		if (!hasStarted)
		{
			if (imageInfo.IsStart)
				hasStarted = 1;
			else continue;
		}
		if (precode == imageInfo.FrameBase) 
			continue;
		if (((precode + 1) & UINT16_MAX) != imageInfo.FrameBase)
			return 1;
		precode = (precode + 1) & UINT16_MAX;
		for (auto& e : imageInfo.Info)
			outputFile.push_back(e);
		if (imageInfo.IsEnd)
			break;
	}
	FILE*fp=fopen(filePath, "wb");
	if (fp == nullptr) return 1;
	outputFile.push_back('\0');
	fwrite(outputFile.data(),sizeof(unsigned char),outputFile.size()-1,fp);
	fclose(fp);
	return 0;
}
int main(int argc, char** argv[])
{
	//FileToVideo("jnsj.jpg", "output2.mp4");

	VideoToFile("phone.mp4", "jnsj2.jpg");
	//cv::Mat srcImg, disImg;
	//srcImg = cv::imread("00074.png", 1);
	//ImgParse::__DisPlay("00402.png");
	//ImgParse::Main(srcImg,disImg);
	//if (FileToVideo("jnsj.jpg", "output2.mp4")) return 1;
    //VideoToFile("phone.mp4", "");
	//ImageDecode::ImageInfo imageInfo;
	//bool ans=ImageDecode::Main(disImg,imageInfo);
	return 0;
}
