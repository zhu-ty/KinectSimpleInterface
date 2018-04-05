#include "KinectSimple.h"

KinectSimple::KinectSimple()
{
	m_pKinectSensor = NULL;
	m_pColorFrameReader = NULL;
	m_pDepthFrameReader = NULL;

	m_pDepthRGBX = new RGBQUAD[cDepthWidth * cDepthHeight];// create heap storage for color pixel data in RGBX format  ，开辟一个动态存储区域  
	m_pColorRGBX = new RGBQUAD[cColorWidth * cColorHeight];
}

KinectSimple::~KinectSimple()
{
	if (m_pDepthRGBX)
	{
		delete[] m_pDepthRGBX;            //删除动态存储区域  
		m_pDepthRGBX = NULL;
	}

	SysUtil::SafeRelease(m_pDepthFrameReader);// done with depth frame reader  

	if (m_pColorRGBX)
	{
		delete[] m_pColorRGBX;
		m_pColorRGBX = NULL;
	}

	SysUtil::SafeRelease(m_pColorFrameReader);// done with color frame reader  

	if (m_pKinectSensor)
	{
		m_pKinectSensor->Close();// close the Kinect Sensor  
	}
	SysUtil::SafeRelease(_mapper);
	SysUtil::SafeRelease(m_pKinectSensor);
}

int KinectSimple::init()
{
	HRESULT hr;                           //typedef long HRESULT  
	hr = GetDefaultKinectSensor(&m_pKinectSensor);      //获取默认的kinect，一般来说只有用一个kinect，所以默认的也就是唯一的那个
	m_pKinectSensor->get_CoordinateMapper(&_mapper);
	if (FAILED(hr))                                //Failed这个函数的参数小于0的时候返回true else 返回false  
	{
		printf("GetDefaultKinectSensor failed!\n");
		return -1;
	}
	if (m_pKinectSensor)
	{ 
		IDepthFrameSource* pDepthFrameSource = NULL;
		IColorFrameSource* pColorFrameSource = NULL;
		hr = m_pKinectSensor->Open();
		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_ColorFrameSource(&pColorFrameSource);
			hr = m_pKinectSensor->get_DepthFrameSource(&pDepthFrameSource);
		}
		if (SUCCEEDED(hr))
		{
			hr = pColorFrameSource->OpenReader(&m_pColorFrameReader);
			hr = pDepthFrameSource->OpenReader(&m_pDepthFrameReader);      //初始化Depth reader，传入该IDepthReader的地址，让该指针指向深度数据流  
		}
		SysUtil::SafeRelease(pColorFrameSource);
		SysUtil::SafeRelease(pDepthFrameSource);
	}
	if (!m_pKinectSensor || FAILED(hr))
	{
		printf("No ready Kinect found! \n");
		return E_FAIL;
	}
	if (SUCCEEDED(hr))
		return 0;
	else
		return -1;
}

int KinectSimple::capture_depth_mat(cv::Mat & dst)
{
	int ret = -1;
	if (!m_pDepthFrameReader)
	{
		return -1;
	}
	IDepthFrame* pDepthFrame = NULL;
	HRESULT hr = m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);//获得深度帧
	if (SUCCEEDED(hr))
	{
		IFrameDescription* pFrameDescription = NULL;
		int nWidth = 0;
		int nHeight = 0;
		USHORT nDepthMinReliableDistance = 0;
		USHORT nDepthMaxDistance = 0;
		UINT nBufferSize = 0;
		UINT16 *pBuffer = NULL;
		hr = pDepthFrame->get_FrameDescription(&pFrameDescription);
		hr = pFrameDescription->get_Width(&nWidth);
		hr = pFrameDescription->get_Height(&nHeight);
		hr = pDepthFrame->get_DepthMinReliableDistance(&nDepthMinReliableDistance);
		// In order to see the full range of depth (including the less reliable far field depth)  
		// we are setting nDepthMaxDistance to the extreme potential depth threshold  
		nDepthMaxDistance = USHRT_MAX;
		// Note:  If you wish to filter by reliable depth distance, uncomment the following line.  
		// hr = pDepthFrame->get_DepthMaxReliableDistance(&nDepthMaxDistance);  
		hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);
		//ProcessDepth(pBuffer, nWidth, nHeight, nDepthMinReliableDistance, nDepthMaxDistance);
		if (m_pDepthRGBX && pBuffer && (nWidth == cDepthWidth) && (nHeight == cDepthHeight))
		{
#ifdef SHOW_DEPTH_255
			RGBQUAD* pRGBX = m_pDepthRGBX;  
			const UINT16* pBufferEnd = pBuffer + (nWidth * nHeight);
			while (pBuffer < pBufferEnd)
			{
				//TODO:这个深度图的映射方式有待商榷
				USHORT depth = *pBuffer;
				// To convert to a byte, we're discarding the most-significant  
				// rather than least-significant bits.  
				// We're preserving detail, although the intensity will "wrap."  
				// Values outside the reliable depth range are mapped to 0 (black).  
				// Note: Using conditionals in this loop could degrade performance.  
				// Consider using a lookup table instead when writing production code.  
				BYTE intensity = static_cast<BYTE>((depth >= nDepthMinReliableDistance) && (depth <= nDepthMaxDistance) ? (depth % 256) : 0);
				pRGBX->rgbRed = intensity;
				pRGBX->rgbGreen = intensity;
				pRGBX->rgbBlue = intensity;
				++pRGBX;
				++pBuffer;
			}
			// Draw the data with OpenCV  
			cv::Mat DepthImage(nHeight, nWidth, CV_8UC4, m_pDepthRGBX);
#else
			cv::Mat DepthImage(nHeight, nWidth, CV_16UC1, pBuffer);
#endif
			dst = DepthImage.clone();
			ret = 0;
		}
		SysUtil::SafeRelease(pFrameDescription);
	}
	SysUtil::SafeRelease(pDepthFrame);
	return ret;
}

int KinectSimple::capture_color_mat(cv::Mat & dst)
{
	int ret = -1;
	if (!m_pColorFrameReader)
	{
		return ret;
	}
	IColorFrame* pColorFrame = NULL;
	HRESULT hr = m_pColorFrameReader->AcquireLatestFrame(&pColorFrame);
	if (SUCCEEDED(hr))
	{
		IFrameDescription* pFrameDescription = NULL;
		int nWidth = 0;
		int nHeight = 0;
		ColorImageFormat imageFormat = ColorImageFormat_None;
		UINT nBufferSize = 0;
		RGBQUAD *pBuffer = NULL;
		hr = pColorFrame->get_FrameDescription(&pFrameDescription);
		hr = pFrameDescription->get_Width(&nWidth);
		hr = pFrameDescription->get_Height(&nHeight);
		hr = pColorFrame->get_RawColorImageFormat(&imageFormat);
		if (imageFormat == ColorImageFormat_Bgra)
		{
			hr = pColorFrame->AccessRawUnderlyingBuffer(&nBufferSize, reinterpret_cast<BYTE**>(&pBuffer));
		}
		else if (m_pColorRGBX)
		{
			pBuffer = m_pColorRGBX;
			nBufferSize = cColorWidth * cColorHeight * sizeof(RGBQUAD);
			hr = pColorFrame->CopyConvertedFrameDataToArray(nBufferSize, reinterpret_cast<BYTE*>(pBuffer), ColorImageFormat_Bgra);
		}
		else
		{
			hr = E_FAIL;
		}
		//ProcessColor(pBuffer, nWidth, nHeight);
		if (pBuffer && (nWidth == cColorWidth) && (nHeight == cColorHeight))
		{
			// Draw the data with OpenCV  
			cv::Mat ColorImage(nHeight, nWidth, CV_8UC4, pBuffer);
			dst = ColorImage.clone();
			ret = 0;
		}
		SysUtil::SafeRelease(pFrameDescription);
	}
	SysUtil::SafeRelease(pColorFrame);
	return ret;
}

int KinectSimple::blend_color_with_depth(cv::Mat & depth, cv::Mat & color, cv::Mat & dst)
{
	return 0;
}
