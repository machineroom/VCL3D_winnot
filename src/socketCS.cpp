#define _WINSOCK_DEPRECATED_NO_WARNINGS


#include "socketCS.h"
#include <iostream>
#include <sys/socket.h>
#include <unistd.h>
#include <poll.h>
#include <netdb.h>
#include <cstring>
//#include <netinet/in.h>
//#include <arpa/inet.h>


#define INVALID_SOCKET -1

using namespace std;

Socket::Socket() : s_(0) {
  s_ = socket(AF_INET,SOCK_STREAM,0);

  if (s_ == INVALID_SOCKET) {
    throw "INVALID_SOCKET";
  }

  refCounter_ = new int(1);
}

Socket::Socket(SOCKET s) : s_(s) {
  refCounter_ = new int(1);
};

Socket::~Socket() {
  if (! --(*refCounter_)) {
    Close();
    delete refCounter_;
  }

}

Socket::Socket(const Socket& o) {
  refCounter_=o.refCounter_;
  (*refCounter_)++;
  s_         =o.s_;

}

Socket& Socket::operator=(Socket& o) {
  (*o.refCounter_)++;

  refCounter_=o.refCounter_;
  s_         =o.s_;

  return *this;
}

void Socket::Close() {
  close(s_);
}

std::string Socket::ReceiveBytes() {
  std::string ret;
  char buf[1024];
 
  while (1) {
    u_long arg = 0;
    
    struct pollfd pfd;
    pfd.fd = s_;
    pfd.events = POLLIN;
    pfd.revents = 0;
    int pr = poll(&pfd, 1, 0);
    if (pfd.revents & POLLIN) {
      int rv = recv (s_, buf, sizeof(buf), 0);
      if (rv <= 0) break;

      std::string t;

      t.assign (buf, rv);
      ret += t;
    }
  }
 
  return ret;
}

void Socket::SendBytes(const char *buf, int len) {
  send(s_,buf,len,0);
}

SocketClient::SocketClient(const std::string& host, int port) : Socket() {
	struct addrinfo hints = {};
	struct addrinfo *addrs;
	bool connected;
	hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;
    int err = getaddrinfo(host.c_str(), std::to_string(port).c_str(), &hints, &addrs);
    if (err == 0) {
		for(struct addrinfo *addr = addrs; addr != NULL; addr = addr->ai_next)
		{	
			// match what earlier socket() created as
			if (addr->ai_family == AF_INET && addr->ai_socktype == SOCK_STREAM) {
			    if (connect(s_, addr->ai_addr, addr->ai_addrlen) == 0) {
			    	std::cout << "connected to " << host << ":" << std::to_string(port) << std::endl;
			    	connected = true;
			        break;
			    }
			}
		}
		if (!connected) {
			throw "Connection problems";
		}
	} else {
		throw strerror(errno);
	}
}

