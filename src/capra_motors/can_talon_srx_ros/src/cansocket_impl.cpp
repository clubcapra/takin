#include <cstring>
#include <stdexcept>

#include <unistd.h>

#include <linux/can.h>
#include <linux/can/bcm.h>

#include <net/if.h>

#include <sys/fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/socket.h>

#include <ros/ros.h>

#include <memory>

#include "can_talon_srx/can_base.h"
#include "can_talon_srx/cansocket.h"

extern std::shared_ptr<can_talon_srx::CanInterface> can_interface;

namespace can_talon_srx
{

  CanSocketInterface::CanSocketInterface(const char* interface_name): running(true)
  {
    struct sockaddr_can addr;
    struct ifreq ifr;

    // socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    socket_ = socket(PF_CAN, SOCK_DGRAM, CAN_BCM);
    if (socket < 0)
    {
      ROS_FATAL("unable to create socket!");
    }
    ROS_INFO("successfully created socket!");

    strncpy(ifr.ifr_name, interface_name, IFNAMSIZ);
    ioctl(socket_, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    // if (bind(socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    if (connect(socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
      ROS_FATAL("unable to connect socket!");
    }
    ROS_INFO("successfully connected socket!");

    auto messageBox = std::make_shared<MessageBox>();
    receivedMessages_ = messageBox;

    ROS_INFO("spawning thread!");
    // start up a thread to read messages
    readThread = std::thread([](std::string name, std::shared_ptr<MessageBox> messages, std::atomic<bool>* running)
    {
        ROS_INFO("thread start");
        struct sockaddr_can addr;
        struct ifreq ifr;

        int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (s < 0)
        {
          ROS_FATAL("unable to create socket!");
        }
        ROS_INFO("successfully created listener socket!");

        strncpy(ifr.ifr_name, name.c_str(), IFNAMSIZ);
        ROS_INFO("binding to %s...", ifr.ifr_name);
        ioctl(s, SIOCGIFINDEX, &ifr);

        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        // addr.can_ifindex = 0;

        if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
        {
          ROS_FATAL("unable to bind socket! %d", errno);
        }
        ROS_INFO("successfully bound listener socket!");

        int flags = fcntl(s, F_GETFL, 0);
        if (flags == -1)
        {
          ROS_FATAL("unable to get socket flags!");
        }
        int retval = fcntl(s, F_SETFL, flags | O_NONBLOCK);
        if (retval == -1)
        {
          ROS_FATAL("unable to set socket flags!");
        }
        ROS_INFO("successfully set listener flags!");

        fd_set readfds;
        struct timeval timeout;
        while (*running)
        {
          // set up a fd set
          FD_ZERO(&readfds);
          FD_SET(s, &readfds);

          // 100 ms timeout
          timeout.tv_sec = 0;
          timeout.tv_usec = 100000;
          int retval = select(1, &readfds, 0, 0, &timeout);
          if (retval == -1)
          {
            ROS_WARN("select failed! %d", errno);
          }
          else if (retval == 0)
          {
            // ROS_INFO("listener timeout");
          }
          // read from s to get can frames and work with them accordingly
          struct can_frame frame;
          int nbytes = 0;
          do {
            nbytes = read(s, &frame, sizeof(frame));
            if (nbytes < 0)
            {
              if (errno != 11)
              {
                ROS_WARN("read fail %d", errno);
              }
            }
            else if (nbytes < sizeof(frame))
            {
              ROS_WARN("incomplete read %d of %ld", nbytes, sizeof(frame));
            }
            else
            {
              std::lock_guard<std::mutex> guard(messages->lock);
              // flip ID length bit
              uint32_t arbID = frame.can_id ^ (1 << 31);
              auto &ptr = messages->messages[arbID];
              // ROS_INFO("packet %08x", arbID);
              auto new_frame = Message
              {
                .arbID=arbID,
                .data={0},
                .len=frame.can_dlc,
              };
              memcpy(&new_frame.data, frame.data, 8);
              ptr = std::make_shared<Message>(new_frame);
            }
          } while (nbytes > 0);
        }

        close(s);
    }, std::string(interface_name), messageBox, &running);
  }

  CanSocketInterface::~CanSocketInterface()
  {
    running = false;
    readThread.join();
    close(socket_);
  }

  void CanSocketInterface::Init(const char* interface_name)
  {
    if (can_interface)
    {
      ROS_ERROR("trying to initialize an already initialized CAN interface!");
      return;
    }
    can_interface = std::shared_ptr<CanInterface>(new CanSocketInterface(interface_name));
  }

  void CanSocketInterface::sendMessage(uint32_t arbID, const uint8_t *data, uint8_t dataSize, int32_t periodMs, int32_t *status)
  {
    struct single_can_msg {
      struct bcm_msg_head msg_head;
      struct can_frame frames[1];
    } can_msg;

    // flip the MSB because it means the opposite in Linux CAN IDs than in 
    // CTRE CAN IDs
    arbID ^= 1 << 31;

    if (periodMs == 0)
    {
      struct no_can_msgs {
        struct bcm_msg_head msg_head;
        struct can_frame frames[0];
      } rm_msg;

      // see if a message with this arbID has already been sent;
      // if it has, cancel that message
      if (sendingIds_.find(arbID) != sendingIds_.end())
      {
        // ROS_INFO("deleting old message for %08x", arbID);
        rm_msg.msg_head.opcode = TX_DELETE;
        // rm_msg.msg_head.flags = 0;
        // rm_msg.msg_head.count = 0;
        // rm_msg.msg_head.ival1 = 0;
        // rm_msg.msg_head.ival2 = 0;
        rm_msg.msg_head.can_id = arbID;
        // rm_msg.msg_head.nframes = 0;

        int nbytes = write(socket_, &rm_msg, sizeof(rm_msg));
        if (nbytes < 0)
        {
          ROS_ERROR("unable to send CAN message: %d", errno);
        }
        else if (nbytes < sizeof(rm_msg))
        {
          ROS_ERROR("unable to send complete CAN message: sent %d of %ld", nbytes, sizeof(rm_msg));
        }
        else
        {
          sendingIds_.erase(arbID);
        }
      }
      // ROS_INFO("sending single shot message for %08x", arbID);
      // then send this message as a single shot
      can_msg.msg_head.opcode = TX_SEND;
      can_msg.msg_head.flags = 0;
      // can_msg.msg_head.count = 0;
      // can_msg.msg_head.ival1 = 0;
      // can_msg.msg_head.ival2 = 0;
      can_msg.msg_head.can_id = arbID;
      can_msg.msg_head.nframes = 1;

      can_msg.frames[0].can_id = arbID;
      can_msg.frames[0].can_dlc = dataSize;
      if (data != nullptr)
      {
        memcpy(can_msg.frames[0].data, data, dataSize);
      }
      else
      {
        memset(can_msg.frames[0].data, 0, dataSize);
      }

      int nbytes = write(socket_, &can_msg, sizeof(can_msg));
      if (nbytes < 0)
      {
        ROS_ERROR("unable to send CAN message: %d", errno);
      }
      else if (nbytes < sizeof(can_msg))
      {
        ROS_ERROR("unable to send complete CAN message: sent %d of %ld", nbytes, sizeof(can_msg));
      }
    }
    else
    {
      // write the request to the kernel and add the arbitration ID to the sending set

      // ROS_INFO("sending repeated message for %08x %d ms", arbID, periodMs);
      // ROS_INFO("%d %x %x %x %x %x %x %x %x", dataSize, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
      can_msg.msg_head.opcode = TX_SETUP;
      can_msg.msg_head.flags = SETTIMER | STARTTIMER;
      can_msg.msg_head.count = 0;
      can_msg.msg_head.ival1.tv_sec = 0;
      can_msg.msg_head.ival1.tv_usec = 0;
      can_msg.msg_head.ival2.tv_sec = periodMs / 1000;
      can_msg.msg_head.ival2.tv_usec = 1000 * (periodMs % 1000);
      can_msg.msg_head.can_id = arbID;
      can_msg.msg_head.nframes = 1;

      can_msg.frames[0].can_id = arbID;
      can_msg.frames[0].can_dlc = dataSize;
      if (data != nullptr)
      {
        memcpy(can_msg.frames[0].data, data, dataSize);
      }
      else
      {
        memset(can_msg.frames[0].data, 0, dataSize);
      }

      int nbytes = write(socket_, &can_msg, sizeof(can_msg));
      // int nbytes = write(socket_, &can_msg.frames[0], sizeof(can_msg.frames[0]));
      if (nbytes < 0)
      {
        ROS_ERROR("unable to send CAN message: %d", errno);
      }
      else if (nbytes < sizeof(can_msg))
      {
        ROS_ERROR("unable to send complete CAN message: sent %d of %ld", nbytes, sizeof(can_msg));
      }

      sendingIds_.insert(arbID);
    }
  }

  void CanSocketInterface::receiveMessage(uint32_t *messageID, uint32_t messageIDMask, uint8_t *data, uint8_t *dataSize, uint32_t *timeStamp, int32_t *status)
  {
    // ROS_INFO("receiveMessage start");
    // NOTE: not entirely sure what messageIDMask does; it's usually set to something
    // like 0x1FFFFFFF or 0xFFFFFFFF so I'm guessing it doesn't matter that much
    // Also not sure what timeStamp is for; it's not used so I'm gonna ignore it for now

    // Also need to flip the MSB because the meaning's inverted on Linux instead of CTRE
    // Having bit 31 set means the ID is 11-bit in CTRE, but having it clear means
    // the identifier is 11-bit in Linux SocketCAN
    // So we flip it
    const uint32_t arbID = (*messageID ^ (1 << 31)) & messageIDMask;
    // check the message box to see if a message has been received
    std::lock_guard<std::mutex> guard(receivedMessages_->lock);
    auto& entry = receivedMessages_->messages[arbID];
    if (entry)
    {
      // ROS_INFO("found entry!");
      *status = 0;
      // swap the entry out
      std::shared_ptr<Message> msg;
      // ROS_INFO("swapping entries");
      entry.swap(msg);
      // ROS_INFO("swap success");
      // ROS_INFO("msg len: %d, id: %xd", msg->len, arbID);

      // copy the data out
      // ROS_INFO("copying data");
      memcpy(data, msg->data, msg->len);
      // ROS_INFO("copying data len");
      *dataSize = msg->len;
    }
    else
    {
      // no entry; set status accordingly
      *status = 1;
    }
    // ROS_INFO("receiveMessage end");
  }

  void CanSocketInterface::openStreamSession(uint32_t *sessionHandle, uint32_t messageID, uint32_t messageIDMask, uint32_t maxMessages, int32_t *status)
  {
    throw std::runtime_error("not implemented");
  }

  void CanSocketInterface::closeStreamSession(uint32_t sessionHandle)
  {
    throw std::runtime_error("not implemented");
  }

  void CanSocketInterface::readStreamSession(uint32_t sessionHandle, struct tCANStreamMessage *messages, uint32_t messagesToRead, uint32_t *messagesRead, int32_t *status)
  {
    throw std::runtime_error("not implemented");
  }

  void CanSocketInterface::getCANStatus(float *percentBusUtilization, uint32_t *busOffCount, uint32_t *txFullCount, uint32_t *receiveErrorCount, uint32_t *transmitErrorCount, int32_t *status)
  {
    throw std::runtime_error("not implemented");
  }

}
