#include<opencv2/opencv.hpp>
#include<opencv2/core.hpp>
#include<opencv2/highgui.hpp>
#include <iostream>
using namespace std;
using namespace cv;
int main(){
    Mat a=imread("1.png");
	cout << "sda" << endl;
	imshow("1",a);
	waitKey();
	cin.get();
	return 0;
}