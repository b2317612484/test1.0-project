#include <Ws2tcpip.h>
#include <WinSock2.h>
#include <iostream>

#pragma comment(lib,"ws2_32.lib")/*#pragma:�趨��������״̬��ָʾ���������һЩ�ض��Ķ�����comment(lib,"ws2_32.lib")���������������ӵ�ʱ��Ҫ��ws2_32.lib�⡣*/
#define SERVER_ADDRESS "192.168.4.1" /*�궨��*/
SOCKET myinitClient();

SOCKET myinitClient()
{
	//1 ����汾
	WSADATA wsaData;   /*����ṹ�������洢��WSAStartup�������ú󷵻ص�Windows Sockets����*/
	WSAStartup(MAKEWORD(2, 2), &wsaData);  /*MAKEWORD(2, 2)���������ò�ͬ��Winsock�汾*/
										   //�汾���м���һ������2
	if (LOBYTE(wsaData.wVersion) != 2 || HIBYTE(wsaData.wVersion) != 2) /*LOBYTE�������õ�һ��16bit����ͣ����ұߣ��Ǹ��ֽڡ�HIBYTE�������õ�һ��16bit����ߣ�����ߣ��Ǹ��ֽ�*/
	{
		printf("����汾ʧ��!\n");
		return -1;
	}
	else
		printf("����汾�ɹ�!\n");

	//2 ����socket �׽���
	//                           ͨ��Э��   ����         ������ʽ
	SOCKET serverSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (serverSocket == INVALID_SOCKET) {
		printf("����socketʧ��!\n");
		return -1;
	}
	else
		printf("����socket�ɹ�!\n");

	//3 ����Э���ַ��    ȷ�������ͳ���
	SOCKADDR_IN addr;
	addr.sin_family = AF_INET;//�����socket��һ������һ��
	inet_pton(AF_INET, SERVER_ADDRESS, &addr.sin_addr.S_un.S_addr);
	addr.sin_port = htons(8086);//�˿ں�

								//4 ���ӷ�����
	int r = connect(serverSocket, (sockaddr*)&addr, sizeof addr);
	if (r == INVALID_SOCKET) {
		printf("connectʧ��!\n");
		return -1;
	}
	printf("connect�ɹ�!\n");


	////5 ͨ��
	//char buff[1024] = "hello,i am client\n";
	//char recvBuff[1024];
	//while (1) {
	//	/*memset(buff, 0, 1024);*/
	//	memset(recvBuff, 0, 1024);

	//	send(serverSocket, buff, strlen(buff), NULL);

	//	r = recv(serverSocket, recvBuff, 1024, NULL);
	//	if (r > 0)
	//		printf("���Է������Ļ���:%s\n", recvBuff);
	//}

	//while (1);
	//system("pause");
	return serverSocket;
}

