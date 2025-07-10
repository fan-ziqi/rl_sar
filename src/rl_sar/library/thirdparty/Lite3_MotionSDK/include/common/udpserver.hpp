#pragma once

#include "udpsocket.hpp"
#include <thread>

class UDPServer : public UDPSocket
{
  public:
    void Bind(std::string IPv4, std::uint16_t port, FDR_ON_ERROR) {
      if (inet_pton(AF_INET, IPv4.c_str(), &this->address.sin_addr) <= 0) {
        onError(errno, "Invalid address. Address type not supported.");
        return;
      }

      this->address.sin_family = AF_INET;
      this->address.sin_port = htons(port);

      if (bind(this->sock, (const sockaddr *)&this->address, sizeof(this->address)) < 0) {
        onError(errno, "Cannot bind the socket.");
        return;
      }
    }
    
  void Bind(int port, FDR_ON_ERROR) {
    this->Bind("0.0.0.0", port, onError);
  }

  void setBroadcast(FDR_ON_ERROR) {
    int broadcast = 1;
    if (setsockopt(this->sock, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof broadcast)) {
      onError(errno, "setsockopt(SO_BROADCAST) failed.");
      return;
    }
  }
};