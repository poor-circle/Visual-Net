#include"pic.h"
#include"code.h"
#define Show_Img(src) do\
{\
	cv::imshow("DEBUG", src);\
	cv::waitKey();\
}while (0);
int main(int argc, char** argv[])
{
	//cv::Mat srcImg, disImg;
	//srcImg = cv::imread("test4.jpg", 1);
	//ImgParse::__DisPlay("test.jpg");
	//ImgParse::Main(srcImg,disImg);
	FILE *fp = fopen("jnsj.jpg", "rb");
	fseek(fp, 0, SEEK_END);
	int size = ftell(fp);
	rewind(fp);
	char* temp = (char*)malloc(sizeof(char) * size);
	fread(temp, 1, size, fp);
	Code::main(temp, size,"outputImg","png");
	free(temp);
	return 0;
}
