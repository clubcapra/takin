#ifndef CAN_TALON_SRX_ROS_CANSOCKET_IMPL_H
#define CAN_TALON_SRX_ROS_CANSOCKET_IMPL_H

#include "can_talon_srx/can_base.h"

#include <atomic>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <unordered_set>

namespace can_talon_srx
{

  class CanSocketInterface : public CanInterface
  {
    private:
      struct Message {
        uint32_t arbID;
        uint8_t data[8];
        uint8_t len;
      };

      struct MessageBox {
        std::mutex lock;
        std::unordered_map<uint32_t, std::shared_ptr<Message> > messages;
      };

      int socket_;
      std::atomic<bool> running;
      std::unordered_set<uint32_t> sendingIds_;
      std::shared_ptr<MessageBox> receivedMessages_;
      std::thread readThread;
    protected:
      CanSocketInterface(const char* interface_name);
    public:
      static void Init(const char* interface_name);
      ~CanSocketInterface();

      void sendMessage(uint32_t arbId, const uint8_t *data, uint8_t dataSize, int32_t periodMs, int32_t *status) override;
      void receiveMessage(uint32_t *arbId, uint32_t arbIdMask, uint8_t *data, uint8_t *dataSize, uint32_t *timeStamp, int32_t *status) override;
	  void openStreamSession(uint32_t *sessionHandle, uint32_t arbId, uint32_t arbIdMask, uint32_t maxMessages, int32_t *status) override;
	  void closeStreamSession(uint32_t sessionHandle) override;
	  void readStreamSession(uint32_t sessionHandle, struct tCANStreamMessage *messages, uint32_t messagesToRead, uint32_t *messagesRead, int32_t *status) override;
	  void getCANStatus(float *percentBusUtilization, uint32_t *busOffCount, uint32_t *txFullCount, uint32_t *receiveErrorCount, uint32_t *transmitErrorCount, int32_t *status) override;

  };

}

#endif
