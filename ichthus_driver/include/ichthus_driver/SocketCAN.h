/**
 * @file
 * This file declares an interface to SocketCAN,
 * to facilitates frame transmission and reception.
 */

#if (!defined(SOCKETCAN_H)) && (!defined(MINGW))
#define SOCKETCAN_H

#include "CRC.h"

#include "CANAdapter.h"
#include "CANFrame.h"
#include "stdbool.h"
// IFNAMSIZ, ifreq
#include <net/if.h>

/**
 * Interface request structure used for socket ioctl's
 */
typedef struct ifreq interface_request_t;

/**
 * Socket address type for CAN sockets
 */
typedef struct sockaddr_can can_socket_address_t;


/**
 * Facilitates frame transmission and reception via a CAN adapter
 */
class SocketCAN: public CANAdapter
{
  public:
    interface_request_t if_request;

    //New
    can_socket_address_t addr;

    //CRC
    CRC8 crc_checker;

    /**
     * CAN socket file descriptor
     */
    int sockfd = -1;

    /**
     * Request for the child thread to terminate
     */

    /** Constructor */
    SocketCAN();
    /** Destructor */
    ~SocketCAN();

    /**
     * Open and bind socket
     */
    void open(char*);

    /**
     * Close and unbind socket
     */
    void close();

    /**
     * Returns whether the socket is open or closed
     *
     * @retval true     Socket is open
     * @retval false    Socket is closed
     */
    bool is_open();

    /**
     * Sends the referenced frame to the bus
     */
    void transmit(can_frame_t&);

    /**
     * receive CAN Data
     */
    bool receive(can_frame_t&);

    /**
     * request control message
     */
    void send_control_request(uint8_t interface, bool enable);

    /**
     * make CAN Frame
     */
    void make_can_frame(unsigned int id_hex, uint8_t interface_id,\
                                       bool value, can_frame_t& data);
    void make_can_frame(unsigned int id_hex, unsigned int subsys_id, \
                        float_hex_convert converter, can_frame_t& data);
    void make_can_frame(unsigned int id_hex, float_hex_convert converter,\
                                                   can_frame_t& data);

    void crc_8(uint8_t* data, int data_length, uint8_t* crc);
};

#endif
