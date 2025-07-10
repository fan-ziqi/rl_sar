#pragma once

#include "basesocket.hpp"
#include <string.h>
#include <thread>

class UDPSocket : public BaseSocket
{
  public:
    std::function<void(std::string, std::string, std::uint16_t)> onMessageReceived;
    std::function<void(const char*, int, std::string, std::uint16_t)> onRawMessageReceived;

    explicit UDPSocket(bool useConnect = false, FDR_ON_ERROR, int socketId = -1): BaseSocket(onError, SocketType::UDP, socketId) {
      if (useConnect) {
        std::thread t(Receive, this); ///< usage with Connect()
        t.detach();
      }
      else {
        std::thread t(ReceiveFrom, this);
        t.detach();
      }
    }
      
    ///< SendTo with no connection
    void SendTo(const char *bytes, size_t byteslength, std::string host, uint16_t port, FDR_ON_ERROR) {
      sockaddr_in hostAddr;

      struct addrinfo hints, *res, *it;
      memset(&hints, 0, sizeof(hints));
      hints.ai_family = AF_INET;
      hints.ai_socktype = SOCK_DGRAM;

      int status;
      if ((status = getaddrinfo(host.c_str(), NULL, &hints, &res)) != 0) {
        onError(errno, "Invalid address." + std::string(gai_strerror(status)));
        return;
      }

      for (it = res; it != NULL; it = it->ai_next) {
        if (it->ai_family == AF_INET) { ///< IPv4
          memcpy((void *)(&hostAddr), (void *)it->ai_addr, sizeof(sockaddr_in));
          break; ///< for now, just get first ip (ipv4).
        }
      }

      freeaddrinfo(res);

      hostAddr.sin_port = htons(port);
      hostAddr.sin_family = AF_INET;

      if (sendto(this->sock, bytes, byteslength, 0, (sockaddr *)&hostAddr, sizeof(hostAddr)) < 0) {
        onError(errno, "Cannot send message to the address.");
        return;
      }
    }

    void SendTo(std::string message, std::string host, uint16_t port, FDR_ON_ERROR) {
      this->SendTo(message.c_str(), message.length(), host, port, onError);
    }

    ///< Send with Connect()
    int Send(const char *bytes, size_t byteslength) {
      if (this->isClosed)
      return -1;

      int sent = 0;
      if ((sent = send(this->sock, bytes, byteslength, 0)) < 0)
      {
        perror("send");
      }
      return sent;
    }

    int Send(std::string message) {
      return this->Send(message.c_str(), message.length());
    }

    ///< To use "Send()", need to call Connect() first
    void Connect(uint32_t ipv4, uint16_t port, FDR_ON_ERROR) {
      this->address.sin_family = AF_INET;
      this->address.sin_port = htons(port);
      this->address.sin_addr.s_addr = ipv4;

      ///< Try to connect.
      if (connect(this->sock, (const sockaddr *)&this->address, sizeof(sockaddr_in)) < 0) {
        onError(errno, "Connection failed to the host.");
        return;
      }
    }
    void Connect(std::string host, uint16_t port, FDR_ON_ERROR) {
      struct addrinfo hints, *res, *it;
      memset(&hints, 0, sizeof(hints));
      hints.ai_family = AF_INET;
      hints.ai_socktype = SOCK_DGRAM;

      int status;
      if ((status = getaddrinfo(host.c_str(), NULL, &hints, &res)) != 0) {
        onError(errno, "Invalid address." + std::string(gai_strerror(status)));
        return;
      }

      for (it = res; it != NULL; it = it->ai_next) {
        if (it->ai_family == AF_INET) { ///< IPv4
          memcpy((void *)(&this->address), (void *)it->ai_addr, sizeof(sockaddr_in));
          break; ///< for now, just get first ip (ipv4).
        }
      }

      freeaddrinfo(res);

      this->Connect((uint32_t)this->address.sin_addr.s_addr, port, onError);
    }

  private:
    static void Receive(UDPSocket *udpSocket) {
      char tempBuffer[udpSocket->BUFFER_SIZE];

      while (true) {
        int messageLength;
        messageLength = recv(udpSocket->sock, tempBuffer, udpSocket->BUFFER_SIZE, 0);

        if (messageLength < 0) {
          perror("recvfrom");
          return;
        }
        else {
          tempBuffer[messageLength] = '\0';
          if (udpSocket->onMessageReceived)
            udpSocket->onMessageReceived(std::string(tempBuffer, messageLength), IpToString(udpSocket->address), ntohs(udpSocket->address.sin_port));
          if (udpSocket->onRawMessageReceived)
            udpSocket->onRawMessageReceived(tempBuffer, messageLength, IpToString(udpSocket->address), ntohs(udpSocket->address.sin_port));
        }
      }
    }
    static void ReceiveFrom(UDPSocket *udpSocket) {
      sockaddr_in hostAddr;
      socklen_t hostAddrSize = sizeof(hostAddr);

      char tempBuffer[udpSocket->BUFFER_SIZE];

      while (true) {
        int messageLength;
        messageLength = recvfrom(udpSocket->sock, tempBuffer, udpSocket->BUFFER_SIZE, 0, (sockaddr *)&hostAddr, &hostAddrSize);

        if (messageLength < 0) {
          perror("recvfrom");
          return;
        }
        else {
          tempBuffer[messageLength] = '\0';
          if (udpSocket->onMessageReceived)
            udpSocket->onMessageReceived(std::string(tempBuffer, messageLength), IpToString(hostAddr), ntohs(hostAddr.sin_port));
          if (udpSocket->onRawMessageReceived)
            udpSocket->onRawMessageReceived(tempBuffer, messageLength, IpToString(hostAddr), ntohs(hostAddr.sin_port));
        }
    }
    }
};
