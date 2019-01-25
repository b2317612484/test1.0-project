#include <Ws2tcpip.h>
#include <WinSock2.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <Windows.h>
#include <Kinect.h>
#include<iostream>
#include<time.h>
#include<math.h> 
#include <corecrt_math_defines.h>
#include <ctime> 
#include <thread>
#include "MyDeter.h"


using namespace std;
using namespace cv;
SOCKET serverSocket;

#pragma comment(lib,"ws2_32.lib")/*#pragma:设定编译器的状态或指示编译器完成一些特定的动作。comment(lib,"ws2_32.lib")：告诉链接器链接的时候要找ws2_32.lib库。*/
#define SERVER_ADDRESS "192.168.4.1" /*宏定义*/

Joint joint[JointType::JointType_Count];
Joint newjoint[JointType::JointType_Count];

/*
	Kinect 需要的变量
*/
IKinectSensor* pSensor;
HRESULT hResult = S_OK;
IColorFrameSource* pColorSource;
IBodyFrameSource* pBodySource;
IColorFrameReader* pColorReader;
IBodyFrameReader* pBodyReader;
IFrameDescription* pDescription;
ICoordinateMapper* pCoordinateMapper;
int width = 0;
int height = 0;

int ang;
//释放接口需要自己定义
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL) {
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}


void DrawBone(Mat& SkeletonImage, CvPoint pointSet[], const Joint* pJoints, int whichone, JointType joint0, JointType joint1);

void drawSkeleton(Mat& SkeletonImage, CvPoint pointSet[], const Joint* pJoints, int whichone);

inline void CalculateLeftArm(Joint *jointPaream, int first, int mid, int last);
inline void CalculateRightArm(Joint *jointPaream, int first, int mid, int last);
inline void CalculateLeftLeg(Joint *jointPaream0, int first, int mid, int last, int last1);
inline void CalculateRightLeg(Joint *jointPaream0, int first, int mid, int last, int last1);

inline void CalculateHead(Joint *jointPaream, int first, int last);
inline void CalculateHand(Joint *jointPaream, int first, int mid, int last);

double rollFlag, pitchFlag, Ri_Should_Elbow;
double selectPitch[7], selectRoll[7];
int arm_0_AngleFlag = 0, arm_0_AngleFlag21 = 0, arm_180_AngleFlag = 180;
int arm_0_AngleFlagR = 0, arm_0_AngleFlag21R = 0, arm_180_AngleFlagR = 180;
int arm_0_AngleFlagFL, arm_0_AngleFlag21FL, arm_180_AngleFlagFL = 180;
int arm_0_AngleFlagFR, arm_0_AngleFlag21FR, arm_180_AngleFlagFR = 180;

//----连接
char senduff[1024];

char str11[1024];
char str44[1024];

// 两条腿的
char str77[1024];
char str1111[1024];

//头和手的
char head77[1024];
char handleft[3];
char endnew[3] = { ']','\0' };


#define JOINT_CONFIDENCE (0.35)

struct MyVector3D
{
	double X;
	double Y;
	double Z;
};

typedef struct XnSkeletonJointPosition

{
	Joint              joit;
	JointType          jointType;
	CameraSpacePoint   position;
	int  fConfidence;

} XnSkeletonJointPosition;



void initSocket() {
	//1 请求版本
	WSADATA wsaData;   /*这个结构被用来存储被WSAStartup函数调用后返回的Windows Sockets数据*/
	WSAStartup(MAKEWORD(2, 2), &wsaData);  /*MAKEWORD(2, 2)：声明调用不同的Winsock版本*/
										   //版本号中间有一个不是2
	if (LOBYTE(wsaData.wVersion) != 2 || HIBYTE(wsaData.wVersion) != 2) /*LOBYTE（）：得到一个16bit数最低（最右边）那个字节。HIBYTE（）：得到一个16bit数最高（最左边）那个字节*/
	{
		printf("请求版本失败!\n");

	}
	else
		printf("请求版本成功!\n");

	//2 创建socket 套接字
	//                           通信协议   载体         保护方式
	serverSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (serverSocket == INVALID_SOCKET) {
		printf("创建socket失败!\n");

	}
	else
		printf("创建socket成功!\n");

	//3 创建协议地址族    确定主机和程序
	SOCKADDR_IN addr;
	addr.sin_family = AF_INET;//必须和socket第一个参数一致
	inet_pton(AF_INET, SERVER_ADDRESS, &addr.sin_addr.S_un.S_addr);
	addr.sin_port = htons(8086);//端口号

								//4 连接服务器
	int r = connect(serverSocket, (sockaddr*)&addr, sizeof addr);
	if (r == INVALID_SOCKET) {
		printf("connect失败!\n");

	}
	printf("connect成功!\n");

}
DWORD WINAPI work_thread(LPVOID lpParameter)
{
	cout << "thread function Fun1Proc!\n";
	initSocket();
	return 0;
}

// 转换depth图像到cv::Mat
Mat ConvertMat(const UINT16* pBuffer, int nWidth, int nHeight)
{
Mat img(nHeight, nWidth, CV_8UC1);
uchar* p_mat = img.data;//指向头指针

const UINT16* pBufferEnd = pBuffer + (nWidth * nHeight);//指向最后一个元素的指针

while (pBuffer < pBufferEnd)//16位最大值为65536
{
	*p_mat++ = *pBuffer++ / 65536.0 * 256;
}
return img;
}

// 边缘检测
Mat Sobel(Mat &img) {
	Mat out;
	Mat grad_x, grad_y;
	Mat abs_grad_x, abs_grad_y;

	//X方向
	Sobel(img, grad_x, CV_16S, 1, 0, 3, 1, 1, BORDER_DEFAULT);
	convertScaleAbs(grad_x, abs_grad_x);
	/*Sobel(img, img, CV_16S, 1, 0, 3, 1, 1, BORDER_DEFAULT);
	convertScaleAbs(img, out);*/

	//Y方向
	Sobel(img, grad_y, CV_16S, 0, 1, 3, 1, 1, BORDER_DEFAULT);
	convertScaleAbs(grad_y, abs_grad_y);
	//convertScaleAbs(img, out);

	//合并
	addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, out);

	return out;
}


//----初始化kinect
void initKinect(void) {
	
	hResult = GetDefaultKinectSensor(&pSensor);
	hResult = pSensor->Open();
	if (FAILED(hResult)) {
		std::cerr << "Error : IKinectSensor::Open()" << std::endl;
	}
	//Source
	hResult = pSensor->get_ColorFrameSource(&pColorSource);
	if (FAILED(hResult)) {
		std::cerr << "Error : IKinectSensor::get_ColorFrameSource()" << std::endl;
	}

	hResult = pSensor->get_BodyFrameSource(&pBodySource);
	if (FAILED(hResult)) {
		std::cerr << "Error : IKinectSensor::get_BodyFrameSource()" << std::endl;
	}

	// Reader
	hResult = pColorSource->OpenReader(&pColorReader);
	if (FAILED(hResult)) {
		std::cerr << "Error : IColorFrameSource::OpenReader()" << std::endl;
	}

	hResult = pBodySource->OpenReader(&pBodyReader);
	if (FAILED(hResult)) {
		std::cerr << "Error : IBodyFrameSource::OpenReader()" << std::endl;
	}

	// Description
	hResult = pColorSource->get_FrameDescription(&pDescription);
	if (FAILED(hResult)) {
		std::cerr << "Error : IColorFrameSource::get_FrameDescription()" << std::endl;

	}

	hResult = pSensor->get_CoordinateMapper(&pCoordinateMapper);
	if (FAILED(hResult)) {
		std::cerr << "Error : IKinectSensor::get_CoordinateMapper()" << std::endl;
	}

}

/*	
	------------图像处理函数
*/
Mat pictureProcessing(Mat &matsrc) {

	Mat matGauss(height, width, CV_8UC4);
	Mat matGary(height, width, CV_8UC4);
	Mat matSobel(height, width, CV_8UC4);
	Mat matThold(height, width, CV_8UC4);
	Mat matmorphologyEx(height, width, CV_8UC4);

	GaussianBlur(matsrc, matGauss, Size(3, 3), 0, 0, BORDER_DEFAULT);
	cvtColor(matGauss, matGary, CV_RGB2GRAY);
	matSobel = Sobel(matGary);
	threshold(matGary, matThold, 123, 255, CV_THRESH_OTSU + CV_THRESH_BINARY);

	Mat element = getStructuringElement(MORPH_RECT, Size(15, 15));
	morphologyEx(matThold, matmorphologyEx, cv::MORPH_OPEN, element);
	return matmorphologyEx;
}
int main(int argc, char **argv[])
{

	// 人体颜色变量定义
	cv::Vec3b color[BODY_COUNT];

	//纯黑
	/*color[0] = cv::Vec3b(0, 0, 0);
	color[1] = cv::Vec3b(0, 0, 0);
	color[2] = cv::Vec3b(0, 0, 0);
	color[3] = cv::Vec3b(0, 0, 0);
	color[4] = cv::Vec3b(0, 0, 0);
	color[5] = cv::Vec3b(0, 0, 0);*/

	//纯白
	color[0] = cv::Vec3b(255, 255, 255);
	color[1] = cv::Vec3b(255, 255, 255);
	color[2] = cv::Vec3b(255, 255, 255);
	color[3] = cv::Vec3b(255, 255, 255);
	color[4] = cv::Vec3b(255, 255, 255);
	color[5] = cv::Vec3b(255, 255, 255);
	float Skeletons[6][25][3];

	//initSocket();  //wifi开启
	//OpenCV中开启CPU的硬件指令优化功能函数
	setUseOptimized(true);
	// 初始化Kinect
	initKinect();
	pDescription->get_Width(&width); // 1920
	pDescription->get_Height(&height); // 1080
	unsigned int bufferSize = width * height * 4 * sizeof(unsigned char);

	// 定义一些中间变量
	cv::Mat bufferMat(height, width, CV_8UC4);
	cv::Mat bufferMatChuli(height, width, CV_8UC4);
	cv::Mat bodyMat(height / 2, width / 2, CV_8UC4);
	cv::namedWindow("Body");

	while (1)
	{	
		for (int i = 0; i <= 3; i++)
		{
		// Frame
		IColorFrame* pColorFrame = nullptr;
		hResult = pColorReader->AcquireLatestFrame(&pColorFrame);
		if (SUCCEEDED(hResult)) {
			hResult = pColorFrame->CopyConvertedFrameDataToArray(bufferSize, reinterpret_cast<BYTE*>(bufferMatChuli.data), ColorImageFormat::ColorImageFormat_Bgra);
			bufferMat = pictureProcessing(bufferMatChuli);
		}

		//更新骨骼帧
		IBodyFrame* pBodyFrame = nullptr;
		hResult = pBodyReader->AcquireLatestFrame(&pBodyFrame);

		if (SUCCEEDED(hResult)) {

			IBody* pBody[BODY_COUNT] = { 0 };
			//更新骨骼数据
			hResult = pBodyFrame->GetAndRefreshBodyData(BODY_COUNT, pBody);
			if (SUCCEEDED(hResult)) {
				// 6个人
				for (int count = 0; count < 6; count++) {
					BOOLEAN bTracked = false;
					hResult = pBody[count]->get_IsTracked(&bTracked);
					// 追踪到了
					if (SUCCEEDED(hResult) && bTracked) {

						// 获取关键节点
						hResult = pBody[count]->GetJoints(JointType::JointType_Count, joint);//joint
						if (SUCCEEDED(hResult)) {
							// Left Hand State

							HandState leftHandState = HandState::HandState_Unknown;
							hResult = pBody[count]->get_HandLeftState(&leftHandState);
							if (SUCCEEDED(hResult)) {
								ColorSpacePoint colorSpacePoint = { 0 };
								hResult = pCoordinateMapper->MapCameraPointToColorSpace(joint[JointType::JointType_HandLeft].Position, &colorSpacePoint);
								if (SUCCEEDED(hResult)) {
									int x = static_cast<int>(colorSpacePoint.X);
									int y = static_cast<int>(colorSpacePoint.Y);

									if ((x >= 0) && (x < width) && (y >= 0) && (y < height)) {
										if (leftHandState == HandState::HandState_Open) {
											cv::circle(bufferMat, cv::Point(x, y), 75, cv::Scalar(0, 128, 0), 10, CV_AA);
											handleft[0] = '1';
											
										}
										else if (leftHandState == HandState::HandState_Closed) {
											cv::circle(bufferMat, cv::Point(x, y), 75, cv::Scalar(0, 0, 128), 10, CV_AA);
											
											handleft[0] = '0';


										}
										else if (leftHandState == HandState::HandState_Lasso) {
											cv::circle(bufferMat, cv::Point(x, y), 75, cv::Scalar(128, 128, 0), 10, CV_AA);
											
										}
									}
								}
							}

							HandState rightHandState = HandState::HandState_Unknown;
							ColorSpacePoint colorSpacePoint = { 0 };
							hResult = pBody[count]->get_HandRightState(&rightHandState);
							if (SUCCEEDED(hResult)) {
								hResult = pCoordinateMapper->MapCameraPointToColorSpace(joint[JointType::JointType_HandRight].Position, &colorSpacePoint);
								if (SUCCEEDED(hResult)) {
									int x = static_cast<int>(colorSpacePoint.X);
									int y = static_cast<int>(colorSpacePoint.Y);

									if ((x >= 0) && (x < width) && (y >= 0) && (y < height)) {
										if (rightHandState == HandState::HandState_Open) {
											cv::circle(bufferMat, cv::Point(x, y), 75, cv::Scalar(0, 128, 0), 10, CV_AA);
										}
										else if (rightHandState == HandState::HandState_Closed) {
											cv::circle(bufferMat, cv::Point(x, y), 75, cv::Scalar(0, 0, 128), 10, CV_AA);

										}
										else if (rightHandState == HandState::HandState_Lasso) {
											cv::circle(bufferMat, cv::Point(x, y), 75, cv::Scalar(128, 128, 0), 10, CV_AA);
										}
									}
								}
							}
							CvPoint skeletonPoint[BODY_COUNT][JointType_Count] = { cvPoint(0,0) };

							// Joint  获取关键点的信息

							for (int type = 0; type < JointType::JointType_Count; type++) {
								ColorSpacePoint colorSpacePoint = { 0 };
								pCoordinateMapper->MapCameraPointToColorSpace(joint[type].Position, &colorSpacePoint);
								int x = static_cast<int>(colorSpacePoint.X);
								int y = static_cast<int>(colorSpacePoint.Y);
								skeletonPoint[count][type].x = x;
								skeletonPoint[count][type].y = y;
								if ((x >= 0) && (x < width) && (y >= 0) && (y < height)) {
									cv::circle(bufferMat, cv::Point(x, y), 15, static_cast<cv::Scalar>(color[count]), -1, CV_AA);
								}
							}
							for (int i = 0; i < 25; i++) {
								Skeletons[count][i][0] = joint[i].Position.X;
								Skeletons[count][i][1] = joint[i].Position.Y;
								Skeletons[count][i][2] = joint[i].Position.Z;

							}
							//UINT16 *depthData = new UINT16[424 * 512];
							if (i == 3)
							{
								//printf("===========\r\n");
								CalculateLeftArm(joint, JointType_ShoulderLeft, JointType_ElbowLeft, JointType_WristLeft);
								CalculateRightArm(joint, JointType_ShoulderRight, JointType_ElbowRight, JointType_WristRight);
								strcat_s(str11, str44);
								send(serverSocket, str11, strlen(str11), NULL);
								// 腿部
								CalculateLeftLeg(joint, JointType_HipLeft, JointType_KneeLeft, JointType_AnkleLeft, JointType_FootLeft);
								CalculateRightLeg(joint, JointType_HipRight, JointType_KneeRight, JointType_AnkleRight, JointType_FootRight);
								//脑袋和手
								CalculateHead(joint, JointType_Head, JointType_Neck);

								strcat_s(str77, str1111);
								strcat_s(str77, head77);
								send(serverSocket, str77, strlen(str77), NULL);
								printf("===str11===%s\r\n", str11);
								printf("===str77===%s\r\n", str77);
						
								memset(str44, 0, sizeof(str44));
								memset(str11, 0, sizeof(str11));
								memset(str1111, 0, sizeof(str1111));
								memset(head77, 0, sizeof(head77));
								memset(str77, 0, sizeof(str77));
								i = 0;	
							}
					// 画骨骼线
					drawSkeleton(bufferMat, skeletonPoint[count], joint, count);
						}
					}
				}
				cv::resize(bufferMat, bodyMat, cv::Size(), 0.5, 0.5);
			}
			for (int count = 0; count < BODY_COUNT; count++) {
				SafeRelease(pBody[count]);
			}
		}

		SafeRelease(pColorFrame);
		SafeRelease(pBodyFrame);

		waitKey(1);
		//
		cv::imshow("Body", bodyMat);

		}
		

	}
	SafeRelease(pColorSource);
	SafeRelease(pColorReader);
	SafeRelease(pDescription);
	SafeRelease(pBodySource);
	// done with body frame reader
	SafeRelease(pBodyReader);

	SafeRelease(pDescription);
	// done with coordinate mapper
	SafeRelease(pCoordinateMapper);

	if (pSensor) {
		pSensor->Close();
	}
	SafeRelease(pSensor);
	//CloseHandle(hThread1);

	return 0;
}




void DrawBone(Mat& SkeletonImage, CvPoint pointSet[], const Joint* pJoints, int whichone, JointType joint0, JointType joint1)
{
	TrackingState joint0State = pJoints[joint0].TrackingState;
	TrackingState joint1State = pJoints[joint1].TrackingState;

	// If we can't find either of these joints, exit
	if ((joint0State == TrackingState_NotTracked) || (joint1State == TrackingState_NotTracked))
	{
		return;
	}

	// Don't draw if both points are inferred
	if ((joint0State == TrackingState_Inferred) && (joint1State == TrackingState_Inferred))
	{
		return;
	}


	CvScalar color;
	switch (whichone) //跟踪不同的人显示不同的颜色   
	{
	case 0:
		color = cvScalar(255);
		break;
	case 1:
		color = cvScalar(255, 255);
		break;
	case 2:
		color = cvScalar(255, 255, 255);
		break;
	case 3:
		color = cvScalar(255, 255, 255);
		break;
	case 4:
		color = cvScalar(255, 255, 255);
		break;
	case 5:
		color = cvScalar(255, 255, 255);
		break;
	}


	// We assume all drawn bones are inferred unless BOTH joints are tracked
	if ((joint0State == TrackingState_Tracked) && (joint1State == TrackingState_Tracked))
	{
		line(SkeletonImage, pointSet[joint0], pointSet[joint1], color, 10);
	}
	else
	{
		line(SkeletonImage, pointSet[joint0], pointSet[joint1], color, 10);
	}
}

void drawSkeleton(Mat& SkeletonImage, CvPoint pointSet[], const Joint* pJoints, int whichone)
{

	// Draw the bones

	// Torso
	DrawBone(SkeletonImage, pointSet, pJoints, whichone, JointType_Head, JointType_Neck);
	DrawBone(SkeletonImage, pointSet, pJoints, whichone, JointType_Neck, JointType_SpineShoulder);
	DrawBone(SkeletonImage, pointSet, pJoints, whichone, JointType_SpineShoulder, JointType_SpineMid);
	DrawBone(SkeletonImage, pointSet, pJoints, whichone, JointType_SpineMid, JointType_SpineBase);
	DrawBone(SkeletonImage, pointSet, pJoints, whichone, JointType_SpineShoulder, JointType_ShoulderRight);
	DrawBone(SkeletonImage, pointSet, pJoints, whichone, JointType_SpineShoulder, JointType_ShoulderLeft);
	DrawBone(SkeletonImage, pointSet, pJoints, whichone, JointType_SpineBase, JointType_HipRight);
	DrawBone(SkeletonImage, pointSet, pJoints, whichone, JointType_SpineBase, JointType_HipLeft);

	// Right Arm    
	DrawBone(SkeletonImage, pointSet, pJoints, whichone, JointType_ShoulderRight, JointType_ElbowRight);
	DrawBone(SkeletonImage, pointSet, pJoints, whichone, JointType_ElbowRight, JointType_WristRight);
	DrawBone(SkeletonImage, pointSet, pJoints, whichone, JointType_WristRight, JointType_HandRight);
	DrawBone(SkeletonImage, pointSet, pJoints, whichone, JointType_HandRight, JointType_HandTipRight);
	DrawBone(SkeletonImage, pointSet, pJoints, whichone, JointType_WristRight, JointType_ThumbRight);

	// Left Arm
	DrawBone(SkeletonImage, pointSet, pJoints, whichone, JointType_ShoulderLeft, JointType_ElbowLeft);
	DrawBone(SkeletonImage, pointSet, pJoints, whichone, JointType_ElbowLeft, JointType_WristLeft);
	DrawBone(SkeletonImage, pointSet, pJoints, whichone, JointType_WristLeft, JointType_HandLeft);
	DrawBone(SkeletonImage, pointSet, pJoints, whichone, JointType_HandLeft, JointType_HandTipLeft);
	DrawBone(SkeletonImage, pointSet, pJoints, whichone, JointType_WristLeft, JointType_ThumbLeft);

	// Right Leg
	DrawBone(SkeletonImage, pointSet, pJoints, whichone, JointType_HipRight, JointType_KneeRight);
	DrawBone(SkeletonImage, pointSet, pJoints, whichone, JointType_KneeRight, JointType_AnkleRight);
	DrawBone(SkeletonImage, pointSet, pJoints, whichone, JointType_AnkleRight, JointType_FootRight);

	// Left Leg
	DrawBone(SkeletonImage, pointSet, pJoints, whichone, JointType_HipLeft, JointType_KneeLeft);
	DrawBone(SkeletonImage, pointSet, pJoints, whichone, JointType_KneeLeft, JointType_AnkleLeft);
	DrawBone(SkeletonImage, pointSet, pJoints, whichone, JointType_AnkleLeft, JointType_FootLeft);
}


int compare1024(int parm) 
{
	int a=0;
	if (parm > 999) {
		a = 999;
	}
	else
	{
		a = parm;
	}
	return a;
}

inline void CalculateLeftArm(Joint *jointPaream, int first, int mid, int last)
{

	MyVector3D vector1;
	MyVector3D vector2;
	MyVector3D normal1;
	MyVector3D normal2;
	double deltaNormal1;
	double deltaNormal2;
	double deltaVector1;
	double deltaVector2;
	double cosAngle;
	char str1[1024] = { '#','0','1','\0' };
	char str2[1024] = { '#','0','2','\0' };
	char str3[1024] = { '#','0','3','\0' };

	// 左手
	char str16[1024] = { '#','1','6','\0' };


	Joint shoulder, elbow, hand;
	shoulder = jointPaream[first];
	elbow = jointPaream[mid];
	hand = jointPaream[last];
	//协助小臂转动的肩膀舵机的角度
	if (1)
	{
		 /*vector1      -> shoulder - elbow  
		 vector2      -> hand - elbow  */
		vector1.X = shoulder.Position.X - elbow.Position.X;
		vector1.Y = shoulder.Position.Y - elbow.Position.Y;
		vector1.Z = elbow.Position.Z - shoulder.Position.Z;

		vector2.X = hand.Position.X - elbow.Position.X;
		vector2.Y = hand.Position.Y - elbow.Position.Y;
		vector2.Z = elbow.Position.Z - hand.Position.Z;

		// normal1 = vector1 x vector2  

		normal1.X = vector1.Y * vector2.Z - vector1.Z * vector2.Y;
		normal1.Y = vector1.Z * vector2.X - vector1.X * vector2.Z;
		normal1.Z = vector1.X * vector2.Y - vector1.Y * vector2.X;

		normal2.X = 0.0;
		normal2.Y = -150.0;
		normal2.Z = 0.0;
		// 取模
		deltaNormal1 = sqrt(normal1.X * normal1.X + normal1.Y * normal1.Y + normal1.Z * normal1.Z);
		deltaNormal2 = sqrt(normal2.X * normal2.X + normal2.Y * normal2.Y + normal2.Z * normal2.Z);
		if (deltaNormal1 * deltaNormal2 > 0.0)
		{
			//printf("=========\r\n");
			cosAngle = (normal1.X * normal2.X + normal1.Y * normal2.Y + normal1.Z * normal2.Z) / (deltaNormal1 * deltaNormal2);
			if (cosAngle < 0) {
				//printf("发生错误\r\n");
			}
			else
			{
				if (shoulder.Position.Y > hand.Position.Y)
				{
					ang = int(90 - 180 * (acos(cosAngle)) / 3.14);
				}
				else
				{   
					// 手举起来
					ang = int(90 + 180 * (acos(cosAngle)) / 3.14);
					//char str1[1024] = { '#','0','1','\0' };
				}
				if (ang > 150)
				{
					ang=0;
				}
				else
				{
					ang =(150-ang)*1024/300;
				}
				ang = compare1024(ang);
				_itoa_s(ang, senduff, 10);
				char end[3] = { ']','\0' };
				strcat_s(str1, senduff);
				strcat_s(str1, end);
				printf("左胳膊向前放下角度==%d\r\n", ang);

			}
			
		}
		else
			//g_controlRobot.RobotJointAngle[ROBOT_LEFT_SHOULDER_VERTICAL] = INVALID_JOINT_VALUE;
			printf("14有错误22\r\n");
		//大臂的上下摆动角度
		vector1.X = elbow.Position.X - shoulder.Position.X;
		vector1.Y = elbow.Position.Y - shoulder.Position.Y;
		vector1.Z = 0.0;

		vector2.X = 0.0;
		vector2.Y = 100;
		vector2.Z = 0.0;

		deltaVector1 = sqrt(vector1.X * vector1.X + vector1.Y * vector1.Y + vector1.Z * vector1.Z);
		deltaVector2 = sqrt(vector2.X * vector2.X + vector2.Y * vector2.Y + vector2.Z * vector2.Z);

		if (deltaVector1 * deltaVector2 > 0.0)
		{
			cosAngle = (vector1.X * vector2.X + vector1.Y * vector2.Y + vector1.Z * vector2.Z) / (deltaVector1 * deltaVector2);
			if (cosAngle < 0) {
				ang = int(180 - 180 * (acos(cosAngle)) / 3.14);
			}
			else {
				ang = int( 180 * (acos(cosAngle)) / 3.14);
			}
			ang = (150 - ang) * 1024 / 300;
			ang = compare1024(ang);
			_itoa_s(ang, senduff, 10);
			//char str2[1024] = { '#','0','2','\0' };
			char end[3] = { ']','\0' };
			strcat_s(str2, senduff);
			strcat_s(str2, end);
			printf("左胳膊张开角度==%d\r\n", ang);
		}
		else {
			//g_controlRobot.RobotJointAngle[ROBOT_LEFT_SHOULDER_HORIZEN] = INVALID_JOINT_VALUE;
			printf("23ROBOT_LEFT_SHOULDER_HORIZEN==有错误\r\n");
		}
		//肘关节角度
		vector1.X = shoulder.Position.X - elbow.Position.X;
		vector1.Y = shoulder.Position.Y - elbow.Position.Y;
		vector1.Z = elbow.Position.Z - shoulder.Position.Z;

		vector2.X = hand.Position.X - elbow.Position.X;
		vector2.Y = hand.Position.Y - elbow.Position.Y;
		vector2.Z = elbow.Position.Z - hand.Position.Z;

		deltaVector1 = sqrt(vector1.X * vector1.X + vector1.Y * vector1.Y + vector1.Z * vector1.Z);
		deltaVector2 = sqrt(vector2.X * vector2.X + vector2.Y * vector2.Y + vector2.Z * vector2.Z);
		if (deltaVector1 * deltaVector2 > 0.0)
		{
			cosAngle = (vector1.X * vector2.X + vector1.Y * vector2.Y + vector1.Z * vector2.Z)
				/ (deltaVector1 * deltaVector2);
			ang = int(180 * (acos(cosAngle)) / 3.14);
			ang =(150+(180 - ang)) * 1024 / 300;
			ang = compare1024(ang);
			_itoa_s(ang, senduff, 10);
			char end[3] = { ']','\0' };
			strcat_s(str3, senduff);
			strcat_s(str3, end);
			printf("左胳膊肘关节张合角度==%d\r\n", ang);
		
		}
		else {
			printf("LeftArm肘关节角度==有错误\r\n");
		}
		

	}
	strcat_s(str11,str1);
	strcat_s(str11, str2);
	strcat_s(str11, str3);

	// 手部
	strcat_s(str16, handleft);
	strcat_s(str16, endnew);
	strcat_s(str11, str16);

	//printf("2222222--str1====%s\r\n", str11);
	/*return str1[1024];*/

#ifdef DEBUG_MSG_LEFT_ARM
	char bufferLeftArm[200];
	snprintf(bufferLeftArm, sizeof(bufferLeftArm),
		"LEFT_SHOULDER_VERTICAL = %4d  LEFT_SHOULDER_HORIZEN = %4d  LEFT_ELBOW = %4d",
		g_controlRobot.RobotJointAngle[ROBOT_LEFT_SHOULDER_VERTICAL],
		g_controlRobot.RobotJointAngle[ROBOT_LEFT_SHOULDER_HORIZEN],
		g_controlRobot.RobotJointAngle[ROBOT_LEFT_ELBOW]);
	std::cout << bufferLeftArm << std::endl;
	NsLog()->info(bufferLeftArm);
#endif
}
//右臂运动
inline void CalculateRightArm(Joint *jointPaream, int first, int mid, int last)
{
	MyVector3D vector1;
	MyVector3D vector2;
	MyVector3D normal1;
	MyVector3D normal2;
	double deltaNormal1;
	double deltaNormal2;
	double deltaVector1;
	double deltaVector2;
	double cosAngle;
	char str4[1024] = { '#','0','4','\0' };
	char str5[1024] = { '#','0','5','\0' };
	char str6[1024] = { '#','0','6','\0' };

	//controlRobot g_controlRobot;
	Joint shoulder, elbow, hand;
	shoulder = jointPaream[first];
	elbow = jointPaream[mid];
	hand = jointPaream[last];

	//协助小臂转动的肩膀舵机的角度
	if (1)
	{
		// vector1      -> shoulder - elbow  
		// vector2      -> hand - elbow  
		//右肩关节舵机转动角度
		vector1.X = shoulder.Position.X - elbow.Position.X;
		vector1.Y = shoulder.Position.Y - elbow.Position.Y;
		vector1.Z = elbow.Position.Z - shoulder.Position.Z;

		/*vector1.x = 0.0;
		vector1.y = elbow.Position.Y - shoulder.Position.Y;
		vector1.z = shoulder.Position.Z - elbow.Position.Z;*/

		vector2.X = hand.Position.X - elbow.Position.X;
		vector2.Y = hand.Position.Y - elbow.Position.Y;
		vector2.Z = elbow.Position.Z - hand.Position.Z;

		// normal1 = vector1 x vector2  

		normal1.X = vector1.Y * vector2.Z - vector1.Z * vector2.Y;
		normal1.Y = vector1.Z * vector2.X - vector1.X * vector2.Z;
		normal1.Z = vector1.X * vector2.Y - vector1.Y * vector2.X;

		normal2.X = 0.0;
		normal2.Y = -150.0;
		normal2.Z = 0.0;
		//deltaNormal1 = sqrt(vector1.x *vector1.x + vector1.y *vector1.y + vector1.z * vector1.z);
		deltaNormal1 = sqrt(normal1.X * normal1.X + normal1.Y * normal1.Y + normal1.Z * normal1.Z);
		deltaNormal2 = sqrt(normal2.X * normal2.X + normal2.Y * normal2.Y + normal2.Z * normal2.Z);



		if (deltaNormal1 * deltaNormal2 > 0.0)
		{

			cosAngle = (normal1.X * normal2.X + normal1.Y * normal2.Y + normal1.Z * normal2.Z) / (deltaNormal1 * deltaNormal2);

			if (cosAngle > 0) {
				//printf("发生错误\r\n");
				ang = 0;
			}
			/*else
			{*/

				if (shoulder.Position.Y < hand.Position.Y) {

					ang = int(360 - (90 + 180 * (acos(cosAngle)) / 3.14)); // 手举起来
					
				}
				else
				{
					ang = int(-(90 - 180 * (acos(cosAngle)) / 3.14)); // 手放下

				}
				if (ang > 150)
				{
					ang = 1024;
				}
				else
				{
					ang = (150 + ang) * 1024 / 300;
				}
				ang = compare1024(ang);
				_itoa_s(ang, senduff, 10);
				char end[3] = { ']','\0' };
				strcat_s(str4, senduff);
				strcat_s(str4, end);
				printf("右胳膊向前放下角度==%d\r\n", ang);

			//}
		}
		else

			printf("14有错误22\r\n");
		//右大臂的上下摆动角度
		vector1.X = elbow.Position.X - shoulder.Position.X;
		vector1.Y = elbow.Position.Y - shoulder.Position.Y;
		vector1.Z = 0.0;

		vector2.X = 0.0;
		vector2.Y = 100;
		vector2.Z = 0.0;

		deltaVector1 = sqrt(vector1.X * vector1.X + vector1.Y * vector1.Y + vector1.Z * vector1.Z);
		deltaVector2 = sqrt(vector2.X * vector2.X + vector2.Y * vector2.Y + vector2.Z * vector2.Z);

		if (deltaVector1 * deltaVector2 > 0.0)
		{
			cosAngle = (vector1.X * vector2.X + vector1.Y * vector2.Y + vector1.Z * vector2.Z) / (deltaVector1 * deltaVector2);

			if (cosAngle < 0) {
				ang = int(180 - 180 * (acos(cosAngle)) / 3.14);
			}
			else {
				ang = int(180 * (acos(cosAngle)) / 3.14);
			}
			ang = (150 + ang) * 1024 / 300;
			ang = compare1024(ang);
			_itoa_s(ang, senduff, 10);
			char end[3] = { ']','\0' };
			strcat_s(str5, senduff);
			strcat_s(str5, end);
			printf("右胳膊张开角度==%d\r\n", ang);
		}
		else {
			printf("23右ROBOT_LEFT_SHOULDER_HORIZEN==有错误\r\n");
		}
		//右肘关节角度
		vector1.X = shoulder.Position.X - elbow.Position.X;
		vector1.Y = shoulder.Position.Y - elbow.Position.Y;
		vector1.Z = elbow.Position.Z - shoulder.Position.Z;

		vector2.X = hand.Position.X - elbow.Position.X;
		vector2.Y = hand.Position.Y - elbow.Position.Y;
		vector2.Z = elbow.Position.Z - hand.Position.Z;

		deltaVector1 = sqrt(vector1.X * vector1.X + vector1.Y * vector1.Y + vector1.Z * vector1.Z);
		deltaVector2 = sqrt(vector2.X * vector2.X + vector2.Y * vector2.Y + vector2.Z * vector2.Z);

		if (deltaVector1 * deltaVector2 > 0.0)
		{
			cosAngle = (vector1.X * vector2.X + vector1.Y * vector2.Y + vector1.Z * vector2.Z) / (deltaVector1 * deltaVector2);
			ang = int(180 * (acos(cosAngle)) / 3.14); 
			ang = (150-(180 - ang)) * 1024 / 300;
			ang = compare1024(ang);
			_itoa_s(ang, senduff, 10);
			char end[3] = { ']','\0' };
			strcat_s(str6, senduff);
			strcat_s(str6, end);
			printf("右胳膊肘关节张合角度==%d\r\n", ang);
		}
		else {
			printf("RightArm右肘关节角度==有错误\r\n");
			//g_controlRobot.RobotJointAngle[ROBOT_LEFT_ELBOW] = INVALID_JOINT_VALUE;
		}

	}
	strcat_s(str44, str4);
	strcat_s(str44, str5);
	strcat_s(str44, str6);
	//return str4[1024];
}
//控制左腿关节的旋转角度
inline void CalculateLeftLeg(Joint *jointPaream0, int first, int mid, int last, int last1)
{
	MyVector3D vector1;
	MyVector3D vector2;
	MyVector3D normal1;
	MyVector3D normal2;
	double deltaNormal1;
	double deltaNormal2;
	double deltaVector1;
	double deltaVector2;
	double cosAngle;
	Joint hip, knee, ankle, foot;
	hip = jointPaream0[first];
	knee = jointPaream0[mid];
	ankle = jointPaream0[last];
	foot = jointPaream0[last1];

	char str7[1024] = { '#','0','7','\0' };
	char str8[1024] = { '#','0','8','\0' };
	char str9[1024] = { '#','0','9','\0' };
	char str10[1024] = { '#','1','0','\0' };

	if (1)
	{
		//胯关节左右摆动
		vector1.X = knee.Position.X - hip.Position.X;
		vector1.Y = knee.Position.Y - hip.Position.Y;
		vector1.Z = 0.0;

		vector2.X = 0.0;
		vector2.Y = -100;
		vector2.Z = 0.0;

		deltaVector1 = sqrt(vector1.X * vector1.X + vector1.Y * vector1.Y + vector1.Z * vector1.Z);
		deltaVector2 = sqrt(vector2.X * vector2.X + vector2.Y * vector2.Y + vector2.Z * vector2.Z);

		// 此角度范围是0-90度
		if (deltaVector1 * deltaVector2 > 0.0)
		{
			cosAngle = (vector1.X * vector2.X + vector1.Y * vector2.Y + vector1.Z * vector2.Z) / (deltaVector1 * deltaVector2);

			if (cosAngle < 0) {
				//printf("发生错误\r\n");
			}

		
			else {
				ang = int(180 * (acos(cosAngle)) / 3.14);
				ang = (150 + ang) * 1024 / 300;
				ang = compare1024(ang);
				_itoa_s(ang, senduff, 10);
				
				char end[3] = { ']','\0' };
				strcat_s(str7, senduff);
				strcat_s(str7, end);
			}

		}
		//胯关节前后摆动
		vector1.X = 0.0;
		vector1.Y = knee.Position.Y - hip.Position.Y;
		vector1.Z = hip.Position.Z - knee.Position.Z;

		vector2.X = 0.0;
		vector2.Y = -100;
		vector2.Z = 0.0;

		deltaVector1 = sqrt(vector1.X * vector1.X + vector1.Y * vector1.Y + vector1.Z * vector1.Z);
		deltaVector2 = sqrt(vector2.X * vector2.X + vector2.Y * vector2.Y + vector2.Z * vector2.Z);
		if (deltaVector1 * deltaVector2 > 0.0)
		{
			cosAngle = (vector1.X * vector2.X + vector1.Y * vector2.Y + vector1.Z * vector2.Z) / (deltaVector1 * deltaVector2);
			if (cosAngle < 0) {
				//printf("发生错误\r\n");
			}
			else
			{
				ang = int(180 * (acos(cosAngle)) / 3.14);
				ang = (150 + ang) * 1024 / 300;
				ang = compare1024(ang);
				_itoa_s(ang, senduff, 10);
				char end[3] = { ']','\0' };
				strcat_s(str8, senduff);
				strcat_s(str8, end);

			}

		}
		//膝盖旋转角度
		vector1.X = hip.Position.X - knee.Position.X;
		vector1.Y = hip.Position.Y - knee.Position.Y;
		vector1.Z = knee.Position.Z - hip.Position.Z;

		vector2.X = ankle.Position.X - knee.Position.X;
		vector2.Y = ankle.Position.Y - knee.Position.Y;
		vector2.Z = knee.Position.Z - ankle.Position.Z;


		deltaVector1 = sqrt(vector1.X * vector1.X + vector1.Y * vector1.Y + vector1.Z * vector1.Z);
		deltaVector2 = sqrt(vector2.X * vector2.X + vector2.Y * vector2.Y + vector2.Z * vector2.Z);

		if (deltaVector1 * deltaVector2 > 0.0)
		{
			cosAngle = (vector1.X * vector2.X + vector1.Y * vector2.Y + vector1.Z * vector2.Z) / (deltaVector1 * deltaVector2);
			ang = int(180 * (acos(cosAngle)) / 3.14);
			ang = (150+(180 - ang)) * 1024 / 300;
			ang = compare1024(ang);
			_itoa_s(ang, senduff, 10);
			char end[3] = { ']','\0' };
			strcat_s(str9, senduff);
			strcat_s(str9, end);
		}

		////脚踝旋转角度
		//vector1.X = knee.Position.X - ankle.Position.X;
		//vector1.Y = knee.Position.Y - ankle.Position.Y;
		//vector1.Z = ankle.Position.Z - knee.Position.Z;

		//vector2.X = foot.Position.X - ankle.Position.X;
		//vector2.Y = foot.Position.Y - ankle.Position.Y;
		//vector2.Z = ankle.Position.Z - foot.Position.Z;


		//deltaVector1 = sqrt(vector1.X * vector1.X + vector1.Y * vector1.Y + vector1.Z * vector1.Z);
		//deltaVector2 = sqrt(vector2.X * vector2.X + vector2.Y * vector2.Y + vector2.Z * vector2.Z);

		//

		//if (deltaVector1 * deltaVector2 > 0.0)
		//{
		//	cosAngle = (vector1.X * vector2.X + vector1.Y * vector2.Y + vector1.Z * vector2.Z) / (deltaVector1 * deltaVector2);
		//	if (cosAngle < 0) {
		//		printf("发生错误\r\n");
		//	}
		//	else
		//	{
		//		ang = int(180 * (acos(cosAngle)) / 3.14);
		//		ang = (150 - (115 - ang)) * 1024 / 300;
		//		ang = compare1024(ang);
		//		_itoa_s(ang, senduff, 10);				
		//		char end[3] = { ']','\0' };
		//		strcat_s(str10, senduff);
		//		strcat_s(str10, end);
		//		printf("----------leftAnkle==%s\r\n", str10);

		//	}
		//}

	}
	strcat_s(str77, str7);
	strcat_s(str77, str8);
	strcat_s(str77, str9);
	//strcat_s(str77, str10);
}
//控制右腿关节的旋转角度
inline void CalculateRightLeg(Joint *jointPaream0, int first, int mid, int last, int last1)
{
	MyVector3D vector1;
	MyVector3D vector2;
	MyVector3D normal1;
	MyVector3D normal2;
	double deltaNormal1;
	double deltaNormal2;
	double deltaVector1;
	double deltaVector2;
	double cosAngle;
	//controlRobot g_controlRobot;
	Joint hip, knee, ankle, foot;
	hip = jointPaream0[first];
	knee = jointPaream0[mid];
	ankle = jointPaream0[last];
	foot = jointPaream0[last1];

	char str11[1024] = { '#','1','1','\0' };
	char str12[1024] = { '#','1','2','\0' };
	char str13[1024] = { '#','1','3','\0' };
	char str14[1024] = { '#','1','4','\0' };

	if (1)
	{
		//胯关节左右摆动
		vector1.X = knee.Position.X - hip.Position.X;
		vector1.Y = knee.Position.Y - hip.Position.Y;
		vector1.Z = 0.0;

		vector2.X = 0.0;
		vector2.Y = -100;
		vector2.Z = 0.0;

		deltaVector1 = sqrt(vector1.X * vector1.X + vector1.Y * vector1.Y + vector1.Z * vector1.Z);
		deltaVector2 = sqrt(vector2.X * vector2.X + vector2.Y * vector2.Y + vector2.Z * vector2.Z);

		if (deltaVector1 * deltaVector2 > 0.0)
		{
			cosAngle = (vector1.X * vector2.X + vector1.Y * vector2.Y + vector1.Z * vector2.Z) / (deltaVector1 * deltaVector2);
			if (cosAngle < 0) {
				//printf("发生错误\r\n");
			}
			else {
				ang = int(180 * (acos(cosAngle)) / 3.14);
				ang = (150 - ang) * 1024 / 300;
				ang = compare1024(ang);
				_itoa_s(ang, senduff, 10);
				char end[3] = { ']','\0' };
				strcat_s(str11, senduff);
				strcat_s(str11, end);
			}
		}
		//胯关节前后摆动
		vector1.X = 0.0;
		vector1.Y = knee.Position.Y - hip.Position.Y;
		vector1.Z = hip.Position.Z - knee.Position.Z;

		vector2.X = 0.0;
		vector2.Y = -100;
		vector2.Z = 0.0;

		deltaVector1 = sqrt(vector1.X * vector1.X + vector1.Y * vector1.Y + vector1.Z * vector1.Z);
		deltaVector2 = sqrt(vector2.X * vector2.X + vector2.Y * vector2.Y + vector2.Z * vector2.Z);
		if (deltaVector1 * deltaVector2 > 0.0)
		{
			cosAngle = (vector1.X * vector2.X + vector1.Y * vector2.Y + vector1.Z * vector2.Z) / (deltaVector1 * deltaVector2);
			if (cosAngle < 0) {
				//printf("发生错误\r\n");
			}
			else
			{
				ang = int(180 * (acos(cosAngle)) / 3.14);
				ang = (150 + ang) * 1024 / 300;
				ang = compare1024(ang);
				_itoa_s(ang, senduff, 10);
				char end[3] = { ']','\0' };
				strcat_s(str12, senduff);
				strcat_s(str12, end);

			}
		}
		//膝盖旋转角度
		vector1.X = hip.Position.X - knee.Position.X;
		vector1.Y = hip.Position.Y - knee.Position.Y;
		vector1.Z = knee.Position.Z - hip.Position.Z;

		vector2.X = ankle.Position.X - knee.Position.X;
		vector2.Y = ankle.Position.Y - knee.Position.Y;
		vector2.Z = knee.Position.Z - ankle.Position.Z;


		deltaVector1 = sqrt(vector1.X * vector1.X + vector1.Y * vector1.Y + vector1.Z * vector1.Z);
		deltaVector2 = sqrt(vector2.X * vector2.X + vector2.Y * vector2.Y + vector2.Z * vector2.Z);

		if (deltaVector1 * deltaVector2 > 0.0)
		{
			cosAngle = (vector1.X * vector2.X + vector1.Y * vector2.Y + vector1.Z * vector2.Z) / (deltaVector1 * deltaVector2);
			ang = int(180 * (acos(cosAngle)) / 3.14);
			ang = (150 - (180 - ang)) * 1024 / 300;
			ang = compare1024(ang);

			_itoa_s(ang, senduff, 10);
			char end[3] = { ']','\0' };
			strcat_s(str13, senduff);
			strcat_s(str13, end);

		}

		////脚踝旋转角度
		//vector1.X = knee.Position.X - ankle.Position.X;
		//vector1.Y = knee.Position.Y - ankle.Position.Y;
		//vector1.Z = ankle.Position.Z - knee.Position.Z;

		//vector2.X = foot.Position.X - ankle.Position.X;
		//vector2.Y = foot.Position.Y - ankle.Position.Y;
		//vector2.Z = ankle.Position.Z - foot.Position.Z;


		//deltaVector1 = sqrt(vector1.X * vector1.X + vector1.Y * vector1.Y + vector1.Z * vector1.Z);
		//deltaVector2 = sqrt(vector2.X * vector2.X + vector2.Y * vector2.Y + vector2.Z * vector2.Z);

		//

		//if (deltaVector1 * deltaVector2 > 0.0)
		//{
		//	cosAngle =(vector1.X * vector2.X + vector1.Y * vector2.Y + vector1.Z * vector2.Z) / (deltaVector1 * deltaVector2);

		//	if (cosAngle < 0) {
		//		printf("发生错误\r\n");
		//	}
		//	else
		//	{
		//		ang = int(180 * (acos(cosAngle)) / 3.14);
		//		ang = (150 + (115 - ang)) * 1024 / 300;
		//		ang = compare1024(ang);
		//		_itoa_s(ang, senduff, 10);
		//		char end[3] = { ']','\0' };
		//		strcat_s(str14, senduff);
		//		strcat_s(str14, end);
		//		memset(senduff, 0, 1024);
		//	}
		//}

	}
	strcat_s(str1111, str11);
	strcat_s(str1111, str12);
	strcat_s(str1111, str13);
	//strcat_s(str1111, str14);
}
inline void CalculateHead(Joint *jointPaream, int first, int last)
{
	MyVector3D vector1;
	MyVector3D vector2;

	double deltaVector1;
	double deltaVector2;
	double cosAngle;
	char headArr[1024] = { '#','1','5','\0' };

	Joint head, neck;
	head = jointPaream[first];
	neck = jointPaream[last];


	//协助小臂转动的肩膀舵机的角度
	if (1) 
	{
		//胯关节左右摆动
		vector1.X = head.Position.X - neck.Position.X;
		vector1.Y = head.Position.Y - neck.Position.Y;
		vector1.Z = 0.0;

		vector2.X = 100;
		vector2.Y = 0.0;
		vector2.Z = 0.0;

		deltaVector1 = sqrt(vector1.X * vector1.X + vector1.Y * vector1.Y + vector1.Z * vector1.Z);
		deltaVector2 = sqrt(vector2.X * vector2.X + vector2.Y * vector2.Y + vector2.Z * vector2.Z);

		if (deltaVector1 * deltaVector2 > 0.0)
		{
			cosAngle = (vector1.X * vector2.X + vector1.Y * vector2.Y + vector1.Z * vector2.Z) / (deltaVector1 * deltaVector2);
			ang = int(180 * (acos(cosAngle)) / 3.14);
			if (ang < 75) 
			{
				ang = 344;
			}
			else if (75 <= ang < 95) 
			{
				ang = 512;
			}
			else
			{
				ang = 680;
			}
			_itoa_s(ang, senduff, 10);
			char end[3] = { ']','\0' };
			strcat_s(headArr, senduff);
			strcat_s(headArr, end);
		}
	}
	strcat_s(head77, headArr);
}

