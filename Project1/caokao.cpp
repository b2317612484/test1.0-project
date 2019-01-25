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
//// ת��depthͼ��cv::Mat
//Mat ConvertMat(const UINT16* pBuffer, int nWidth, int nHeight)
//{
//	Mat img(nHeight, nWidth, CV_8UC1);
//	uchar* p_mat = img.data;//ָ��ͷָ��
//
//	const UINT16* pBufferEnd = pBuffer + (nWidth * nHeight);//ָ�����һ��Ԫ�ص�ָ��
//
//	while (pBuffer < pBufferEnd)//16λ���ֵΪ65536
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
//	HRESULT hr = GetDefaultKinectSensor(&m_pKinectSensor);//��ȡĬ��kinect������
//	assert(hr >= 0);
//	printf("��kinect�������ɹ�\n");
//
//	hr = m_pKinectSensor->Open();//�򿪴�����
//	assert(hr >= 0);
//	hr = m_pKinectSensor->get_DepthFrameSource(&pDepthFrameSource);//��������Ϣ������
//	assert(hr >= 0);
//	hr = pDepthFrameSource->OpenReader(&m_pDepthFrameReader);//�������Ϣ֡��ȡ��
//	assert(hr >= 0);
//
//	while (hr < 0 || pDepthFrame == NULL)
//		hr = m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);//������ʱ���ȡ���������ѭ����ȡ�����֡
//
//	assert(hr >= 0);
//	hr = pDepthFrame->get_FrameDescription(&pFrameDescription);//��ȡ֡��������Ϣ����͸ߣ�
//	int depth_width, depth_height;
//	pFrameDescription->get_Width(&depth_width);
//	pFrameDescription->get_Height(&depth_height);
//	printf("width=%d height=%d\n", depth_width, depth_height);
//
//	USHORT nDepthMinReliableDistance = 0;//��ȡ�����С��Ⱦ�����Ϣ
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
//	pDepthFrame->AccessUnderlyingBuffer(&nBufferSize_depth, &pBuffer_depth);//��ȡͼ�����ظ�����ָ��ͼ���ָ��
//
//
//																			//ת��ΪMAT��ʽ
//	Mat depthImg_show = ConvertMat(pBuffer_depth, depth_width, depth_height);//ת��Ϊ8λ��mat
//
//
//	equalizeHist(depthImg_show, depthImg_show);//���⻯��Ϊ�������ʾЧ��
//
//	imwrite("MyFirstKinectImg.jpg", depthImg_show);//����ͼƬ
//												   //��opencv��ʾ
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
