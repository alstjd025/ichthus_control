/**
 * @file
 * This file implements functions to receive
 * and transmit CAN frames via SocketCAN.
 */

#ifndef MINGW

#include <SocketCAN.h>
#include <stdio.h>
// strncpy
#include <string.h>
// close
#include <unistd.h>
// socket
#include <sys/socket.h>
// SIOCGIFINDEX
#include <sys/ioctl.h>

/*
 * https://github.com/JCube001/socketcan-demo
 * http://blog.mbedded.ninja/programming/operating-systems/linux/how-to-use-socketcan-with-c-in-linux
 * https://github.com/linux-can/can-utils/blob/master/candump.c
 */

/*
#include <endian.h>
#include <sys/socket.h>
#include <sys/types.h>
*/

SocketCAN::SocketCAN()
   :CANAdapter(),
    sockfd(-1)
{
    adapter_type = ADAPTER_SOCKETCAN;
    crc_checker = CRC8(0x07, 0x00, false, false, 0x00, false);
    printf("SocketCAN adapter created.\n");
}


SocketCAN::~SocketCAN()
{
    printf("Destroying SocketCAN adapter...\n");
    if (this->is_open())
    {
        this->close();
    }
}


void SocketCAN::open(char* interface)
{
    // Request a socket
    sockfd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sockfd == -1)
    {
        printf("Error: Unable to create a CAN socket\n");
        return;
    }
    printf("Created CAN socket with descriptor %d.\n", sockfd);

    // Get the index of the network interface
    strncpy(if_request.ifr_name, interface, IFNAMSIZ);
    //if (ioctl(sockfd, SIOCGIFINDEX, &if_request) == -1)
    if (ioctl(sockfd, SIOCGIFINDEX, &if_request) == -1)
    {
        printf("Unable to select CAN interface %s: I/O control error\n", interface);

        // Invalidate unusable socket
        close();
        return;
    }
    printf("Found: %s has interface index %d.\n", interface, if_request.ifr_ifindex);

    // Bind the socket to the network interface
    addr.can_family = AF_CAN;
    addr.can_ifindex = if_request.ifr_ifindex;
    int rc = bind(
        sockfd,
        reinterpret_cast<struct sockaddr*>(&addr),
        sizeof(addr)
    );
    if (rc == -1)
    {
        printf("Failed to bind socket to network interface\n");

        // Invalidate unusable socket
        close();
        return;
    }
    printf("Successfully bound socket to interface %d.\n", if_request.ifr_ifindex);

}


void SocketCAN::close()
{
    if (!is_open())
        return;

    // Close the file descriptor for our socket
    ::close(sockfd);
    sockfd = -1;

    printf("CAN socket destroyed.\n");
}


bool SocketCAN::is_open()
{
    return (sockfd != -1);
}


void SocketCAN::transmit(can_frame_t& frame)
{
    int nbytes;
    if (!is_open())
    {
        printf("Unable to transmit: Socket not open\n");
        return;
    }
    nbytes = write(sockfd, &frame, sizeof(can_frame_t));
    if(nbytes < 0){
        printf("Write Failed %d bytes \n", nbytes);
        return;
    }
    return;
}


bool SocketCAN::receive(can_frame_t& rx_frame)
{
    // Holds the set of descriptors, that 'select' shall monitor
    fd_set descriptors;

    // Highest file descriptor in set
    int maxfd = this->sockfd;

    // Clear descriptor set
    FD_ZERO(&descriptors);
    // Add socket descriptor
    FD_SET(this->sockfd, &descriptors);

    // Wait until activity on any descriptor
    if (select(maxfd+1, &descriptors, NULL, NULL, NULL) == 1)
    {
        int len = read(this->sockfd, &rx_frame, CAN_MTU);
            
        if (len < 0)
        {
            return false;
        }
    }
    else
    {
        printf("[RX] Nothing to Receieve.\n");
        return false;
    }

    printf("[RX] terminated.\n");

    return true;
}

void SocketCAN::send_control_request(uint8_t interface, bool enable){
  can_frame_t send_data;
  make_can_frame(CONTROL_ID, interface, enable, send_data);
  transmit(send_data);
}

void SocketCAN::make_can_frame(unsigned int id_hex, uint8_t interface_id, bool value\
                                                                 ,can_frame_t& data){
  float_hex_convert converter;
  memset(converter.data, 0, 4); 
  converter.data[0] = interface_id;
  converter.data[1] = value;
  make_can_frame(id_hex, converter, data);
}

void SocketCAN::make_can_frame(unsigned int id_hex, unsigned int subsys_id, \
                        float_hex_convert converter, can_frame_t& data){
    switch(id_hex){
    // Override Ack Command
    case 0x41:{
        data.can_dlc = 8;
        data.can_id = id_hex;
        data.data[0] = subsys_id;
        data.data[1] = NONE;
        data.data[2] = NONE;
        memcpy(data.data+3, converter.data, sizeof(float));
        break;    
    }
    }
    crc_8(data.data, 7, data.data+7);
}

void SocketCAN::make_can_frame(unsigned int id_hex, float_hex_convert converter\
                                                            , can_frame_t& data){
  switch (id_hex){
  // Accel Command
  case 0x160: {
    data.can_dlc = 8;
    data.can_id = id_hex;
    data.data[0] = DEFAULT_BUS;
    data.data[2] = NONE;
    memcpy(data.data+3, converter.data, sizeof(float));
    break;
  }
  // Control Request
  case 0x60: {
    data.can_dlc = 8;
    data.can_id = id_hex;
    data.data[0] = DEFAULT_BUS;
    memcpy(data.data+1, converter.data, sizeof(float));
    break;
  }
  default:
    break;
  }
  crc_8(data.data, 7, data.data+7);
}

void SocketCAN::crc_8(uint8_t* data, int data_length_bytes, uint8_t* crc){
  *crc = crc_checker.calculate(data, data_length_bytes);
}

#endif
