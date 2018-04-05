/**
@brief KinectSimple.h
A super simple interface for Kinect and OpenCV
@author zhu-ty
@date Apr 5, 2018
*/

#ifndef __KINECT_SIMPLE__
#define __KINECT_SIMPLE__

// Windows Header Files
#include <windows.h>

// Kinect Header files
#include <Kinect.h>
#include "opencv2/opencv.hpp" 

#define SHOW_DEPTH_255


namespace SysUtil
{
    // Safe release for interfaces
    template<class Interface>
    inline void SafeRelease(Interface *& pInterfaceToRelease)
    {
        if (pInterfaceToRelease != NULL)
        {
            pInterfaceToRelease->Release();
            pInterfaceToRelease = NULL;
        }
    }
}


class KinectSimple
{
public:
	KinectSimple();
	~KinectSimple();

    int init();
    int capture_depth_mat(cv::Mat &dst);
    int capture_color_mat(cv::Mat &dst);
    int blend_color_with_depth(cv::Mat &depth, cv::Mat &color, cv::Mat &dst);
private:
	static const int cDepthWidth = 512;   //深度图的大小  
	static const int cDepthHeight = 424;

	static const int cColorWidth = 1920;   //彩色图的大小  
	static const int cColorHeight = 1080;
	IKinectSensor*          m_pKinectSensor;// Current Kinect  
	IDepthFrameReader*      m_pDepthFrameReader;// Depth reader    在需要的时候可以再添加IColorFrameReader,进行color reader  
	RGBQUAD*                m_pDepthRGBX;
	IColorFrameReader*      m_pColorFrameReader;// Color reader  
	RGBQUAD*                m_pColorRGBX;

	ICoordinateMapper* _mapper;
	ColorSpacePoint p;
};

#endif //__KINECT_SIMPLE__