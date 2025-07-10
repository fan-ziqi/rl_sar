#pragma once

#if defined(__linux__) || defined(__APPLE__)
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <netdb.h>
#elif _WIN32
#include <winsock32.h>
#endif

#include <string>
#include <functional>
#include <cerrno>

#define FDR_UNUSED(expr){ (void)(expr); } 
#define FDR_ON_ERROR std::function<void(int, std::string)> onError = [](int errorCode, std::string errorMessage){FDR_UNUSED(errorCode); FDR_UNUSED(errorMessage)}

class BaseSocket
{
  ///< Definitions
  public:
    enum SocketType {
      TCP = SOCK_STREAM,
      UDP = SOCK_DGRAM
    };
    const uint16_t BUFFER_SIZE = 0xFFFF;
    sockaddr_in address;
    bool isClosed = false;

  protected:
    int sock = 0;
    static std::string IpToString(sockaddr_in addr) {
      char ip[INET_ADDRSTRLEN];
      inet_ntop(AF_INET, &(addr.sin_addr), ip, INET_ADDRSTRLEN);

      return std::string(ip);
    }  

    BaseSocket(FDR_ON_ERROR, SocketType sockType = TCP, int socketId = -1) {
      if (socketId < 0)
      {
        if ((this->sock = socket(AF_INET, sockType, 0)) < 0)
        {
          onError(errno, "Socket creating error.");
        }
      }
      else
      {
        this->sock = socketId;
      }
    }

  ///< Methods
  public:
    virtual void Close() {
      if(isClosed) return;

      isClosed = true;
      close(this->sock);
    }

    std::string RemoteAddress() {return IpToString(this->address);}
    int RemotePort() {return ntohs(this->address.sin_port);}
    int FileDescriptor() const { return this->sock; }
};