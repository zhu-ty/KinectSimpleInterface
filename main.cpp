
#include "KinectSimple.h"
#include "opencv2/opencv.hpp"

using namespace cv;

int get_pt_depth_13(Mat &d, int x, int y)
{
	int d16 = 0;
	memcpy(&d16, d.data + (y * d.cols + x) * 2, 2);
	d16 = d16 & 0xfff8;
	d16 = d16 >> 3;
	return d16;
}

int main()
{
	KinectSimple kinect;
	Mat showImageColor;
	Mat showImageDepth;
	Mat realImageDepth;
	Mat real_calibrated_depth;
	Mat show_calibrated_depth;
	kinect.init();
	while (1)
	{
		if (kinect.capture_depth_mat(realImageDepth, showImageDepth) == 0 && kinect.capture_color_mat(showImageColor) == 0)
		{
			imshow("Depth", showImageDepth);
			imshow("Color", showImageColor);
			kinect.get_mapped_depth_in_color_space(realImageDepth, showImageColor, real_calibrated_depth, show_calibrated_depth);
			imshow("Mapped Color", show_calibrated_depth);
		}
		if (waitKey(1) >= 0)//按下任意键退出  
		{
			break;
		}
	}
	return 0;
}