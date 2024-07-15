//#include "api/global_constants.h"
#include "SimpleTCPSocket.h"

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
//#define snprintf _snprintf

#else  //Unix includes
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <netinet/tcp.h>
#include <unistd.h>
#include <iostream>

#define closesocket close
#endif //_WIN32

#include <cassert>
#include <cstring>
#include <cstdio>
//using namespace std;

#ifndef SOCKET_ERROR
#define SOCKET_ERROR -1
#endif

#ifndef INVALID_SOCKET
#define INVALID_SOCKET -1
#endif

#if !defined SD_BOTH && defined SHUT_RDWR
#define SD_BOTH SHUT_RDWR
#endif

unsigned int SimpleTCPSocket::nactive_=0;

SimpleTCPSocket::SimpleTCPSocket(){
    //init socket varibles
    socket_=INVALID_SOCKET;

    if(nactive_ == 0){
        init();
    }
    nactive_++;
}

SimpleTCPSocket::~SimpleTCPSocket(){
    close();
    nactive_--;
    if(nactive_ == 0){
        cleanup();
    }
}

void SimpleTCPSocket::init(){
#ifdef _WIN32
    WSADATA wsaData;
    WSAStartup(MAKEWORD(2,2), &wsaData);
#endif
    return;
}

void SimpleTCPSocket::cleanup(){
#ifdef _WIN32
    WSACleanup();
#endif
    return;
}

int SimpleTCPSocket::send(const char* buf, const int& size) const{
//    assert(socket_ != INVALID_SOCKET);
	if (socket_ != INVALID_SOCKET)
		return ::send(socket_, buf, size, MSG_NOSIGNAL);
	return SOCKET_ERROR;
}

int SimpleTCPSocket::recv(char* buf, const int& size, int flags) const{
//    assert(socket_ != INVALID_SOCKET);
	if (socket_ != INVALID_SOCKET) {
		int ret = ::recv(socket_, buf, size, flags);
		int errorcode;
		if (ret == SOCKET_ERROR) {
			//errorcode = WSAGetLastError();
		}
		return ret;
	}
	return SOCKET_ERROR;
}

bool SimpleTCPSocket::close(){ // close socket
    if(socket_ == INVALID_SOCKET){
        return true;
    }

    int ret = ::shutdown(socket_, SD_BOTH);
    if(ret == SOCKET_ERROR ){
        return false;
    }

    ret = ::closesocket(socket_);
    if(ret == SOCKET_ERROR ){
        return false;
    }
    socket_ = INVALID_SOCKET;

    return true;
}

SimpleTCPServerSocket::SimpleTCPServerSocket(){
    serversocket_=INVALID_SOCKET;
}

SimpleTCPServerSocket::~SimpleTCPServerSocket(){
    closeserver();
}


bool SimpleTCPServerSocket::bind(const unsigned int portnum){

    struct sockaddr_in my_addr;    // my address information
    int yes=1;
	int keep_alive = 1;

    if ((serversocket_ = socket(PF_INET, SOCK_STREAM, 0)) == SOCKET_ERROR) {
        perror("serversocket");
        return false;

    }

	struct timeval tv;
	tv.tv_sec = 1;
	tv.tv_usec = 0;

	if ((setsockopt(serversocket_, SOL_SOCKET, SO_RCVTIMEO, (struct timeval *)&tv, sizeof(struct timeval))) == SOCKET_ERROR) {
		return false;
	}

    if (setsockopt(serversocket_,SOL_SOCKET,SO_REUSEADDR,&yes,sizeof(int)) 
        == SOCKET_ERROR) {
        perror("setsockopt");
        closeserver();
        return false;
    }

	if (setsockopt(serversocket_, SOL_SOCKET, SO_KEEPALIVE, &keep_alive, sizeof(int))
		== SOCKET_ERROR) {
		perror("setsockopt");
		closeserver();
		return false;
	}
    
    my_addr.sin_family = AF_INET;         // host byte order
    my_addr.sin_port = htons(portnum);     // short, network byte order
    my_addr.sin_addr.s_addr = INADDR_ANY; // automatically fill with my IP
    memset(&(my_addr.sin_zero), 0, 8); // zero the rest of the struct

    if (::bind(serversocket_, (struct sockaddr*)&my_addr, 
               sizeof(struct sockaddr))
        == SOCKET_ERROR){
        perror("bind");
        closeserver();
        return false;
    }
    return true;
}

bool SimpleTCPServerSocket::listen(){
    int ret = ::listen(serversocket_, 1);
    if(ret !=0){
        return false;
    }
    return true;
}

bool SimpleTCPServerSocket::accept(){
    socket_ = ::accept(serversocket_, NULL, NULL);
    if(socket_ == INVALID_SOCKET){
        return false;
    }
	struct timeval tv;
	tv.tv_sec = 1;
	tv.tv_usec = 0;

	if ((setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, (struct timeval *)&tv, sizeof(struct timeval))) == SOCKET_ERROR) {
		return false;
	}
    return true;
}

bool SimpleTCPServerSocket::closeserver(){ //close socket
    if(serversocket_ == INVALID_SOCKET){
        return true;
    }

    int ret = ::closesocket(serversocket_);
    if(ret == SOCKET_ERROR ){
        return false;
    }
    serversocket_ = INVALID_SOCKET;
    return true;
}

bool SimpleTCPClientSocket::connect(const char* host,
                                    const unsigned int portnum, int recv_timeout) {

//     struct sockaddr_in their_addr; // connector's address information
//     char yes=1;
//     struct hostent *he;

//     if ((he=gethostbyname(host)) == NULL) {  // get the host info 
//         close();
//         perror("gethostbyname");
//         return false;
//     }

//     if ((socket_ = socket(PF_INET, SOCK_STREAM, 0)) == SOCKET_ERROR) {
//         perror("socket");
//         return false;

//     }

//    /* if (setsockopt(socket_,SOL_SOCKET,SO_SNDTIMEO,
//                    (char*)(&send_timeout),sizeof(int)) == SOCKET_ERROR) {
//         perror("setsockopt");
//         close();
//         return false;
//     }*/

// 	if (setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO,
// 		(char*)(&recv_timeout), sizeof(int)) == SOCKET_ERROR) {
// 		perror("setsockopt");
// 		close();
// 		return false;
// 	}

//     int flag=1;
//     if (setsockopt(socket_,IPPROTO_TCP,TCP_NODELAY,
//                    (char*)(&flag),sizeof(int)) == SOCKET_ERROR) {
//         perror("setsockopt");
//         close();
//         return false;
//     }

//     their_addr.sin_family = AF_INET;    // host byte order 
//     their_addr.sin_port = htons(portnum);  // short, network byte order 
//     their_addr.sin_addr = *((struct in_addr *)he->h_addr);
//     memset(&(their_addr.sin_zero), 0, 8);  // zero the rest of the struct 

//     if (::connect(socket_, (struct sockaddr*)&their_addr,
//                   sizeof(struct sockaddr))== SOCKET_ERROR){
//         perror("connect");
//         close();
//         return false;
//     }

//     return true;

    struct sockaddr_in their_addr; // connector's address information
    char yes=1;
    struct hostent *he;

	struct timeval tv;
	tv.tv_sec = 0;
	tv.tv_usec = recv_timeout;

    /*if ((he=gethostbyname(host)) == NULL) {  // get the host info 
        close();
        perror("gethostbyname");
        return false;
    }*/

	struct addrinfo hints = {}, *addrs;
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;

	char port_str[16] = {};
	sprintf(port_str, "%d", portnum);
	printf("%s", host);

	if (getaddrinfo(host, port_str, &hints, &addrs) != 0) {
	        perror("gethostbyname");
       		close();
        	return false;
	}

    if ((socket_ = socket(PF_INET, SOCK_STREAM, 0)) == SOCKET_ERROR) {
        perror("socket");
        return false;

    }

   /* if (setsockopt(socket_,SOL_SOCKET,SO_SNDTIMEO,
                   (char*)(&send_timeout),sizeof(int)) == SOCKET_ERROR) {
        perror("setsockopt");
        close();
        return false;
    }*/

	if (setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO,
		(struct timeval *)&tv, sizeof(struct timeval)) == SOCKET_ERROR) {
		perror("setsockopt");
		close();
		return false;
	}

    int flag=1;
    if (setsockopt(socket_,IPPROTO_TCP,TCP_NODELAY,
                   (char*)(&flag),sizeof(int)) == SOCKET_ERROR) {
        perror("setsockopt");
        close();
        return false;
    }

	//struct sockaddr_in *temp = (struct sockaddr_in*)(addrs->ai_addr);

    their_addr.sin_family = AF_INET;    // host byte order 
    their_addr.sin_port = htons(portnum);  // short, network byte order 
    their_addr.sin_addr = ((struct sockaddr_in*)(addrs->ai_addr))->sin_addr;
    memset(&(their_addr.sin_zero), 0, 8);  // zero the rest of the struct 

    if (::connect(socket_, (struct sockaddr*)&their_addr,
                  sizeof(struct sockaddr))== SOCKET_ERROR){
        perror("connect");
        close();
        return false;
    }

    return true;
}

SimpleBroadcastSocket::SimpleBroadcastSocket() {
}

SimpleBroadcastSocket::~SimpleBroadcastSocket() {

}

int SimpleBroadcastSocket::RecvBroadcast(char *buffer, const int &size, int flags) {
  socklen_t recv_addr_len = sizeof(recv_addr_);
  if (socket_ != INVALID_SOCKET) {
    int rlen =
        recvfrom(socket_, buffer, size, flags, (struct sockaddr *)&recv_addr_, &recv_addr_len);
    if (rlen >= 0)
      buffer[rlen] = '\0';
    return rlen;
  }
  return SOCKET_ERROR;
}

int SimpleBroadcastSocket::SendBroadcast(char *buffer, const int &size) {
  int err = 0;

  if (socket_ != INVALID_SOCKET) {
    int val = 1;
    if ((err = setsockopt(socket_, SOL_SOCKET, SO_BROADCAST, (char *)&val, sizeof(int))) ==
        SOCKET_ERROR) {
      return SOCKET_ERROR;
    }

    int rlen =
        sendto(socket_, buffer, size, 0, (const struct sockaddr *)&send_addr_, sizeof(send_addr_));

	int on = 1;
    if (SOCKET_ERROR ==
        setsockopt(socket_, SOL_SOCKET, SO_REUSEADDR, (const char *)&on, sizeof(on))) {
      return SOCKET_ERROR;
    }

    return rlen;
  }
  return SOCKET_ERROR;
}

bool SimpleBroadcastSocket::init_broadcast(const unsigned int portnumber, const std::string& ip_addr) {
  // Construct remote tracker address structure
  memset((char *)&send_addr_, 0, sizeof(send_addr_));
  send_addr_.sin_family = AF_INET;
  send_addr_.sin_port = htons(portnumber);
  //send_addr_.sin_addr.s_addr = htonl(INADDR_BROADCAST);
   //send_addr_.sin_addr.s_addr = inet_addr("192.168.0.168");
   send_addr_.sin_addr.s_addr = inet_addr(ip_addr.c_str());
  // send_addr_.sin_addr.s_addr = inet_addr("127.0.0.255");

  // Construct local address structure
  /*server_address_.sin_family = AF_INET;
  server_address_.sin_addr.s_addr = htonl(INADDR_ANY);
  server_address_.sin_port = htons(portnumber);*/

  socket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (socket_ == INVALID_SOCKET) {
    return false;
  }

  int err;
  /*server_len_ = sizeof(server_address_);
  err = ::bind(socket_, (struct sockaddr *)&server_address_, server_len_);
  if (err != 0) {
    return false;
  }
  getsockname(socket_, (struct sockaddr *)&server_address_, &server_len_);*/

  int val = 1;
  int timeout = 1000;

  	struct timeval tv;
	tv.tv_sec = 1;
	tv.tv_usec = 0;

	if ((setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, (struct timeval *)&tv, sizeof(struct timeval))) == SOCKET_ERROR) {
		return false;
	}

  // Set socket to allow broadcast
  if ((err = setsockopt(socket_, SOL_SOCKET, SO_BROADCAST, (char *)&val, sizeof(int))) ==
      SOCKET_ERROR) {
    return false;
  }

  /*if ((err = setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(int))) ==
      SOCKET_ERROR) {
    return false;
  }*/

  if ((err = setsockopt(socket_, SOL_SOCKET, SO_REUSEADDR, (char *)&val, sizeof(int)) == SOCKET_ERROR)) {
	  return false;
  }

  //  int bufsize = 64000;
  //  if (setsockopt(socket_, SOL_SOCKET, SO_RCVBUF, (char *)&bufsize,
  //                 sizeof(int)) == SOCKET_ERROR) {
  //    return false;
  //  }

  char name[100];
  struct hostent *pHost = nullptr;
  err = gethostname(name, 100);
  if (err == 0 && name != NULL) {
    pHost = gethostbyname(name);
  }
  if (pHost) {
    strcpy(host_ip_, inet_ntoa((*(struct in_addr *)pHost->h_addr_list[0])));
  }

  memset(&recv_addr_, 0, sizeof(recv_addr_));

  return true;
}

void SimpleBroadcastSocket::GetBroadcastIpAddr(char **ip) {
  *ip = inet_ntoa(recv_addr_.sin_addr);
}


SimpleUDPSocket::SimpleUDPSocket() {
	
}

bool SimpleUDPSocket::setup(int port) {
	if ((socket_ = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		//error?
		return false;
	}

	//set up socket information and bind socket
	struct sockaddr_in myAddr;
	memset((char*)&myAddr, 0, sizeof(myAddr));
	myAddr.sin_family = AF_INET;
	myAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	myAddr.sin_port = htons(port);

	/*struct timeval tv;
	tv.tv_sec = 0;
	tv.tv_usec = 100000;
	if (setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0)
		std::cerr << "Couldn't set timeout." << std::endl;
	*/

	addr_len = sizeof(sockaddr);

	if (bind(socket_, (struct sockaddr*)&myAddr, sizeof(myAddr)) < 0) {
		//error?
		return false;
	}
	return true;
}

SimpleUDPSocket::~SimpleUDPSocket() {

}

int SimpleUDPSocket::send(char *buffer, const int &size) {
	std::cout << "Sending to port " << serv_addr.sin_port << std::endl;
	return sendto(socket_, buffer, size, 0, (struct sockaddr*)&serv_addr, sizeof(serv_addr));
}

int SimpleUDPSocket::recv(char *buffer, const int &size) {
	return recvfrom(socket_, buffer, size, 0, (struct sockaddr *)&serv_addr, &addr_len);
}
