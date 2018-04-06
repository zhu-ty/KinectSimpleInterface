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

//һ���򵥵�Kinect��OpenCVʹ����
class KinectSimple
{
public:
	KinectSimple();
	~KinectSimple();
	/**
	@brief ��ʼ��Kinect
	@return int: �ɹ�����0�����򷵻ط���
	*/
    int init();

	/**
	@brief ץȡһ֡������ݣ���СΪ512*424
	@param cv::Mat &dst: ���ڼ�������֡����ʽΪCV_16UC1����16λ��ȣ�1άͨ��
	@param cv::Mat &dst_show: ������ʾ�鿴�����֡����dst�ĵͰ�λȡ���Ľ�����������ڼ��㣬��ʽΪCV_8UC4��8λ��ȣ�4άͨ����ÿһά���ݾ���ͬ
	@return int: �ɹ�����0�����򷵻ط���
	*/
    int capture_depth_mat(cv::Mat &dst, cv::Mat &dst_show);

	/**
	@brief ץȡһ֡��ɫ���ݣ���СΪ1920*1080
	@param cv::Mat &dst: ��ɫ֡����ʽΪCV_8UC4������άӦ��Ϊȫ0��ȫ255��ǰ��άΪRGB
	@return int: �ɹ�����0�����򷵻ط���
	*/
    int capture_color_mat(cv::Mat &dst);

	/**
	@brief �����ͼmap����ɫ1920*1080��С��
	@param cv::Mat &depth: ��ʵ���ͼ���봫��CV_16UC1�����
	@param cv::Mat &color: ��Ӧ�Ĳ�ɫͼ����ʽΪCV_8UC4
	@param cv::Mat &dst: mapping�����ʵ���ͼ����ʽΪCV_16UC1���޲�ɫ��Ϣ�����޶�Ӧ��ĵط�����Ƚ�����Ϊ0
	@param cv::Mat &dst_show: ����Ϊ�˻�����Ƶĵ�λ���/��ɫ���ͼ����ĳ����ӳ����Ϊ��ɫ������ӳ����Ϊ�õ�ĵͰ�λ�����Ϣ����ʽΪCV_8UC4
	@return int: �ɹ�����0�����򷵻ط���
	*/
	int get_mapped_depth_in_color_space(cv::Mat &depth, cv::Mat &color, cv::Mat &dst, cv::Mat &dst_show);
    
private:
	static const int cDepthWidth = 512;   //���ͼ�Ĵ�С  
	static const int cDepthHeight = 424;

	static const int cColorWidth = 1920;   //��ɫͼ�Ĵ�С  
	static const int cColorHeight = 1080;
	IKinectSensor*          m_pKinectSensor;// Current Kinect  
	IDepthFrameReader*      m_pDepthFrameReader;// Depth reader    ����Ҫ��ʱ����������IColorFrameReader,����color reader  
	RGBQUAD*                m_pDepthRGBX;
	IColorFrameReader*      m_pColorFrameReader;// Color reader  
	RGBQUAD*                m_pColorRGBX;

	ICoordinateMapper* _mapper;

	std::vector<DepthSpacePoint> _depth_space_calibrate;
};

#endif //__KINECT_SIMPLE__