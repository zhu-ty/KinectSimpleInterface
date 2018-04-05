
#include "KinectSimple.h"
#include "opencv2/opencv.hpp"

using namespace cv;

int main()
{
	KinectSimple kinect;
	Mat showImageColor;
	Mat showImageDepth;
	kinect.init();
	while (1)
	{
		if (kinect.capture_depth_mat(showImageDepth) == 0)
			imshow("Depth", showImageDepth);
		if (kinect.capture_color_mat(showImageColor) == 0)
			imshow("Color", showImageColor);
		if (waitKey(1) >= 0)//按下任意键退出  
		{
			break;
		}
	}
	return 0;
}