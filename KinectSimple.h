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

//#define SHOW_DEPTH_255


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

//一个简单的Kinect和OpenCV使用类
class KinectSimple
{
public:
	KinectSimple();
	~KinectSimple();
	/**
	@brief 初始化Kinect
	@return int: 成功返回0，否则返回非零
	*/
    int init();

	/**
	@brief 抓取一帧深度数据，大小为512*424
	@param cv::Mat &dst: 用于计算的深度帧，格式为CV_16UC1，即16位深度，1维通道
	@param cv::Mat &dst_show: 用于显示查看的深度帧，将dst的低八位取出的结果，不可用于计算，格式为CV_8UC4，8位深度，4维通道，每一维数据均相同
	@return int: 成功返回0，否则返回非零
	*/
    int capture_depth_mat(cv::Mat &dst, cv::Mat &dst_show);

	/**
	@brief 抓取一帧彩色数据，大小为1920*1080
	@param cv::Mat &dst: 彩色帧，格式为CV_8UC4，第四维应该为全0或全255，前三维为RGB
	@return int: 成功返回0，否则返回非零
	*/
    int capture_color_mat(cv::Mat &dst);

	/**
	@brief 将深度图map到彩色1920*1080大小上
	@param cv::Mat &depth: 真实深度图，请传入CV_16UC1的深度
	@param cv::Mat &color: 对应的彩色图，格式为CV_8UC4
	@param cv::Mat &dst: mapping后的真实深度图，格式为CV_16UC1（无彩色信息），无对应点的地方的深度将被设为0
	@param cv::Mat &dst_show: 单纯为了绘制设计的低位深度/彩色混合图，若某点无映射则为彩色，若有映射则为该点的低八位深度信息，格式为CV_8UC4
	@return int: 成功返回0，否则返回非零
	*/
	int get_mapped_depth_in_color_space(cv::Mat &depth, cv::Mat &color, cv::Mat &dst, cv::Mat &dst_show);
    
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

	std::vector<DepthSpacePoint> _depth_space_calibrate;
};

#endif //__KINECT_SIMPLE__