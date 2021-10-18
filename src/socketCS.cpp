#define _WINSOCK_DEPRECATED_NO_WARNINGS


#include "socketCS.h"
#include <iostream>
#include <sys/socket.h>
#include <unistd.h>
#include <poll.h>


#define INVALID_SOCKET -1

using namespace std;

Socket::Socket() : s_(0) {
  // UDP: use SOCK_DGRAM instead of SOCK_STREAM
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

