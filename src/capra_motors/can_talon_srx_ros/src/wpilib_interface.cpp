#include <memory>

#include "ros/ros.h"

#include "can_talon_srx/can_base.h"

std::shared_ptr<can_talon_srx::CanInterface> can_interface;

extern "C"
{

void FRC_NetworkCommunication_CANSessionMux_sendMessage(uint32_t messageID, const uint8_t *data, uint8_t dataSize, int32_t periodMs, int32_t *status)
{
  if (can_interface)
  {
    can_interface.get()->sendMessage(messageID, data, dataSize, periodMs, status);
  }
  else
  {
    ROS_ERROR("CAN interface was unitialized!");
  }
}

void FRC_NetworkCommunication_CANSessionMux_receiveMessage(uint32_t *messageID, uint32_t messageIDMask, uint8_t *data, uint8_t *dataSize, uint32_t *timeStamp, int32_t *status)
{
  if (can_interface)
  {
    can_interface.get()->receiveMessage(messageID, messageIDMask, data, dataSize, timeStamp, status);
  }
  else
  {
    ROS_ERROR("CAN interface was unitialized!");
  }
}

void FRC_NetworkCommunication_CANSessionMux_openStreamSession(uint32_t *sessionHandle, uint32_t messageID, uint32_t messageIDMask, uint32_t maxMessages, int32_t *status)
{
  if (can_interface)
  {
    can_interface.get()->openStreamSession(sessionHandle, messageID, messageIDMask, maxMessages, status);
  }
  else
  {
    ROS_ERROR("CAN interface was unitialized!");
  }
}

void FRC_NetworkCommunication_CANSessionMux_closeStreamSession(uint32_t sessionHandle)
{
  if (can_interface)
  {
    can_interface.get()->closeStreamSession(sessionHandle);
  } 
  else
  {
    ROS_ERROR("CAN interface was unitialized!");
  }
}

void FRC_NetworkCommunication_CANSessionMux_readStreamSession(uint32_t sessionHandle, struct tCANStreamMessage *messages, uint32_t messagesToRead, uint32_t *messagesRead, int32_t *status)
{
  if (can_interface)
  {
    can_interface.get()->readStreamSession(sessionHandle, messages, messagesToRead, messagesRead, status);
  }
  else
  {
    ROS_ERROR("CAN interface was unitialized!");
  }
}

void FRC_NetworkCommunication_CANSessionMux_getCANStatus(float *percentBusUtilization, uint32_t *busOffCount, uint32_t *txFullCount, uint32_t *receiveErrorCount, uint32_t *transmitErrorCount, int32_t *status)
{
  if (can_interface)
  {
    can_interface.get()->getCANStatus(percentBusUtilization, busOffCount, txFullCount, receiveErrorCount, transmitErrorCount, status);
  }
  else
  {
    ROS_ERROR("CAN interface was unitialized!");
  }
}

}
