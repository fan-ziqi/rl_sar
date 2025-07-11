#ifndef UDP_RECEIVER_H
#define UDP_RECEIVER_H

#include <vector>
#include <cstdint>

/**
 * @brief The UdpReceiver class for receiving UDP data.
 */
class UdpReceiver {
public:
    /**
     * @brief Constructor for UdpReceiver.
     * @param port The port to listen for UDP data.
     */
    UdpReceiver(int port);

    /**
     * @brief Receive UDP data.
     * @return A vector containing received data as uint8_t.
     */
    std::vector<uint8_t> ReceiveData();

    /**
     * @brief Destructor for UdpReceiver.
     */
    ~UdpReceiver();

private:
    int socket_fd_;
};

#endif
