#include <Ws2tcpip.h>
#include <WinSock2.h>
#include <iostream>

#pragma comment(lib,"ws2_32.lib")/*#pragma:设定编译器的状态或指示编译器完成一些特定的动作。comment(lib,"ws2_32.lib")：告诉链接器链接的时候要找ws2_32.lib库。*/
#define SERVER_ADDRESS "192.168.4.1" /*宏定义*/
SOCKET myinitClient();

SOCKET myinitClient()
{
	//1 请求版本
	WSADATA wsaData;   /*这个结构被用来存储被WSAStartup函数调用后返回的Windows Sockets数据*/
	WSAStartup(MAKEWORD(2, 2), &wsaData);  /*MAKEWORD(2, 2)：声明调用不同的Winsock版本*/
										   //版本号中间有一个不是2
	if (LOBYTE(wsaData.wVersion) != 2 || HIBYTE(wsaData.wVersion) != 2) /*LOBYTE（）：得到一个16bit数最低（最右边）那个字节。HIBYTE（）：得到一个16bit数最高（最左边）那个字节*/
	{
		printf("请求版本失败!\n");
		return -1;
	}
	else
		printf("请求版本成功!\n");

	//2 创建socket 套接字
	//                           通信协议   载体         保护方式
	SOCKET serverSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (serverSocket == INVALID_SOCKET) {
		printf("创建socket失败!\n");
		return -1;
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
		return -1;
	}
	printf("connect成功!\n");


	////5 通信
	//char buff[1024] = "hello,i am client\n";
	//char recvBuff[1024];
	//while (1) {
	//	/*memset(buff, 0, 1024);*/
	//	memset(recvBuff, 0, 1024);

	//	send(serverSocket, buff, strlen(buff), NULL);

	//	r = recv(serverSocket, recvBuff, 1024, NULL);
	//	if (r > 0)
	//		printf("来自服务器的回信:%s\n", recvBuff);
	//}

	//while (1);
	//system("pause");
	return serverSocket;
}

