#ifndef CAN_TALON_SRX_CAN_BASE_H
#define CAN_TALON_SRX_CAN_BASE_H

#include "wpilib/CANSessionMux.h"

namespace can_talon_srx
{
  class CanInterface
  {
    public:
      virtual void sendMessage(uint32_t messageID, const uint8_t *data, uint8_t dataSize, int32_t periodMs, int32_t *status) = 0;
      virtual void receiveMessage(uint32_t *messageID, uint32_t messageIDMask, uint8_t *data, uint8_t *dataSize, uint32_t *timeStamp, int32_t *status) = 0;
	  virtual void openStreamSession(uint32_t *sessionHandle, uint32_t messageID, uint32_t messageIDMask, uint32_t maxMessages, int32_t *status) = 0;
	  virtual void closeStreamSession(uint32_t sessionHandle) = 0;
	  virtual void readStreamSession(uint32_t sessionHandle, struct tCANStreamMessage *messages, uint32_t messagesToRead, uint32_t *messagesRead, int32_t *status) = 0;
	  virtual void getCANStatus(float *percentBusUtilization, uint32_t *busOffCount, uint32_t *txFullCount, uint32_t *receiveErrorCount, uint32_t *transmitErrorCount, int32_t *status) = 0;

      virtual ~CanInterface() {}
    protected:
      CanInterface() {}
  };
}

#endif
