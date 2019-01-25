//#include<opencv2/opencv.hpp>
//#include "vector"
//#include "iostream"
//#include "time.h"
//
//#include <stdio.h>
//#include <Kinect.h>
//#include <windows.h>
//
//
//
//using namespace cv;
//using namespace std;
//
//#define imageName "C:/Users/123/Desktop/picture/shipin.mp4"
//
//
//// 转换depth图像到cv::Mat
//Mat ConvertMat(const UINT16* pBuffer, int nWidth, int nHeight)
//{
//	Mat img(nHeight, nWidth, CV_8UC1);
//	uchar* p_mat = img.data;//指向头指针
//
//	const UINT16* pBufferEnd = pBuffer + (nWidth * nHeight);//指向最后一个元素的指针
//
//	while (pBuffer < pBufferEnd)//16位最大值为65536
//	{
//		*p_mat++ = *pBuffer++ / 65536.0 * 256;
//	}
//	return img;
//}
//int main()
//{
//	IKinectSensor*          m_pKinectSensor;
//	IDepthFrameReader*      m_pDepthFrameReader;
//	IDepthFrame* pDepthFrame = NULL;
//	IFrameDescription* pFrameDescription = NULL;
//	IDepthFrameSource* pDepthFrameSource = NULL;
//
//	HRESULT hr = GetDefaultKinectSensor(&m_pKinectSensor);//获取默认kinect传感器
//	assert(hr >= 0);
//	printf("打开kinect传感器成功\n");
//
//	hr = m_pKinectSensor->Open();//打开传感器
//	assert(hr >= 0);
//	hr = m_pKinectSensor->get_DepthFrameSource(&pDepthFrameSource);//获得深度信息传感器
//	assert(hr >= 0);
//	hr = pDepthFrameSource->OpenReader(&m_pDepthFrameReader);//打开深度信息帧读取器
//	assert(hr >= 0);
//
//	while (hr < 0 || pDepthFrame == NULL)
//		hr = m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);//由于有时候获取不到，因此循环获取最近的帧
//
//	assert(hr >= 0);
//	hr = pDepthFrame->get_FrameDescription(&pFrameDescription);//获取帧的像素信息（宽和高）
//	int depth_width, depth_height;
//	pFrameDescription->get_Width(&depth_width);
//	pFrameDescription->get_Height(&depth_height);
//	printf("width=%d height=%d\n", depth_width, depth_height);
//
//	USHORT nDepthMinReliableDistance = 0;//获取最大、最小深度距离信息
//	USHORT nDepthMaxReliableDistance = 0;
//	assert(hr >= 0);
//	hr = pDepthFrame->get_DepthMinReliableDistance(&nDepthMinReliableDistance);
//	assert(hr >= 0);
//	hr = pDepthFrame->get_DepthMaxReliableDistance(&nDepthMaxReliableDistance);
//
//	printf("nDepthMinReliableDistance=%d nDepthMaxReliableDistance=%d\n", nDepthMinReliableDistance, nDepthMaxReliableDistance);
//
//	UINT nBufferSize_depth = 0;
//	UINT16 *pBuffer_depth = NULL;
//	pDepthFrame->AccessUnderlyingBuffer(&nBufferSize_depth, &pBuffer_depth);//获取图像像素个数和指向图像的指针
//
//
//																			//转换为MAT格式
//	Mat depthImg_show = ConvertMat(pBuffer_depth, depth_width, depth_height);//转换为8位的mat
//
//
//	equalizeHist(depthImg_show, depthImg_show);//均衡化，为了提高显示效果
//
//	imwrite("MyFirstKinectImg.jpg", depthImg_show);//保存图片
//												   //用opencv显示
//
//	namedWindow("display");
//
//	imshow("display", depthImg_show);
//
//	if (27 == waitKey(0))
//		return 0;
//}
//
//
//
