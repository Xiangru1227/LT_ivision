#ifndef __SIMPLETCPSOCKET__
#define __SIMPLETCPSOCKET__

#ifdef _WIN32
#include <winsock2.h>
#else
typedef int SOCKET;
#include <arpa/inet.h>
#endif
#include <string>

class SimpleTCPSocket{
protected:
    SOCKET socket_;
    static unsigned int nactive_;

    SimpleTCPSocket();
    virtual ~SimpleTCPSocket();
    void init();
    void cleanup();

public:
    virtual int send(const char* buf, const int& size) const;
    virtual int recv(char* buf, const int& size, int flags) const;
    virtual bool close();
};


class SimpleTCPServerSocket:
    public SimpleTCPSocket{
protected:
    SOCKET serversocket_;
public:
    SimpleTCPServerSocket();
    virtual ~SimpleTCPServerSocket();

    virtual bool bind(const unsigned int portnum);
    virtual bool listen();
    virtual bool accept();
    virtual bool closeserver();
};

class SimpleTCPClientSocket:
    public SimpleTCPSocket{
public:
    SimpleTCPClientSocket(){}
    virtual ~SimpleTCPClientSocket(){}

    virtual bool connect(const char* host, const unsigned int portnum, int recv_timeout = 200);
};

class SimpleBroadcastSocket:
	public SimpleTCPSocket {
public:
	SimpleBroadcastSocket();
	virtual ~SimpleBroadcastSocket();
	int RecvBroadcast(char *buffer, const int &size, int flags);
	int SendBroadcast(char *buffer, const int &size);
	bool init_broadcast(const unsigned int portnumber, const std::string& ip_addr);
	void GetBroadcastIpAddr(char **ip);

private:
	int client_sockfd_;
	struct sockaddr_in client_address_, server_address_, send_addr_, recv_addr_;
	int client_len_, server_len_;
	char host_ip_[80];
};

class SimpleUDPSocket {
public:
	SimpleUDPSocket();
	~SimpleUDPSocket();

	bool setup(int port);

	int send(char *buffer, const int &size);
	int recv(char *buffer, const int &size);
private:
	SOCKET socket_;
	struct sockaddr_in serv_addr;
	socklen_t addr_len;
};

#endif //__SIMPLETCPSOCKET__
