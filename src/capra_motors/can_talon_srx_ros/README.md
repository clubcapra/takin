# can-talon-srx-ros
ROS C++ library for communicating with Talon SRX over CAN

This repository contains code originating from First Robotics; the code there is within the `wpilib` folders in both `include/` and `src/`.
The rest of the repository is under an MIT license.
More appropriate licensing will be done some time eventually probably

Basically lots of code copied from the `wpilibc` from before the Talon SRX drivers were removed ( https://github.com/wpilibsuite/allwpilib/tree/3fe0f49ac082e7228c948270becb15623d27abf9 ) with a bit of reworking to let the CAN layer be abstracted away somewhat.
This project contains some example code in `src/can_talon_srx_node.cpp` to demonstrate how to use this library.

TODOS:
- [x] copy protocol stuff from wpilib
- [ ] figure out and document CAN interaction layer

## API
This package does not provide a ROS node for usage, because in most applications a node would potentially result in nonrealtime performance of the control loop.
Instead, it provides a C++ library that can be used to send and receive messages to Talon SRX connected to the robot via some standard CAN interface.
The API that is provided is a subset of the First Robotics API; currently only function calls that map to their "set-and-forget" CAN messages work, with the remaining functions resulting in an exception and probably a program abort.

The API that is available is:
```

typedef enum {
		CTR_OKAY,				//!< No Error - Function executed as expected
		CTR_RxTimeout,			//!< CAN frame has not been received within specified period of time.
		CTR_TxTimeout,			//!< Not used.
		CTR_InvalidParamValue, 	//!< Caller passed an invalid param
		CTR_UnexpectedArbId,	//!< Specified CAN Id is invalid.
		CTR_TxFailed,			//!< Could not transmit the CAN frame.
		CTR_SigNotUpdated,		//!< Have not received an value response for signal.
		CTR_BufferFull,			//!< Caller attempted to insert data into a buffer that is full.
}CTR_Code;

class CanTalonSRX {

  // ... other stuff ...

  CTR_Code GetFault_OverTemp(int &param);
  CTR_Code GetFault_UnderVoltage(int &param);
  CTR_Code GetFault_ForLim(int &param);
  CTR_Code GetFault_RevLim(int &param);
  CTR_Code GetFault_HardwareFailure(int &param);
  CTR_Code GetFault_ForSoftLim(int &param);
  CTR_Code GetFault_RevSoftLim(int &param);
  CTR_Code GetStckyFault_OverTemp(int &param);
  CTR_Code GetStckyFault_UnderVoltage(int &param);
  CTR_Code GetStckyFault_ForLim(int &param);
  CTR_Code GetStckyFault_RevLim(int &param);
  CTR_Code GetStckyFault_ForSoftLim(int &param);
  CTR_Code GetStckyFault_RevSoftLim(int &param);
  CTR_Code GetAppliedThrottle(int &param);
  CTR_Code GetCloseLoopErr(int &param);
  CTR_Code GetFeedbackDeviceSelect(int &param);
  CTR_Code GetModeSelect(int &param);
  CTR_Code GetLimitSwitchEn(int &param);
  CTR_Code GetLimitSwitchClosedFor(int &param);
  CTR_Code GetLimitSwitchClosedRev(int &param);
  CTR_Code GetSensorPosition(int &param);
  CTR_Code GetSensorVelocity(int &param);
  CTR_Code GetCurrent(double &param);
  CTR_Code GetBrakeIsEnabled(int &param);
  CTR_Code GetEncPosition(int &param);
  CTR_Code GetEncVel(int &param);
  CTR_Code GetEncIndexRiseEvents(int &param);
  CTR_Code GetQuadApin(int &param);
  CTR_Code GetQuadBpin(int &param);
  CTR_Code GetQuadIdxpin(int &param);
  CTR_Code GetAnalogInWithOv(int &param);
  CTR_Code GetAnalogInVel(int &param);
  CTR_Code GetTemp(double &param);
  CTR_Code GetBatteryV(double &param);
  CTR_Code GetResetCount(int &param);
  CTR_Code GetResetFlags(int &param);
  CTR_Code GetFirmVers(int &param);
  CTR_Code GetPulseWidthPosition(int &param);
  CTR_Code GetPulseWidthVelocity(int &param);
  CTR_Code GetPulseWidthRiseToRiseUs(int &param);
  CTR_Code GetActTraj_IsValid(int &param);
  CTR_Code GetActTraj_ProfileSlotSelect(int &param);
  CTR_Code GetActTraj_VelOnly(int &param);
  CTR_Code GetActTraj_IsLast(int &param);
  CTR_Code GetOutputType(int &param);
  CTR_Code GetHasUnderrun(int &param);
  CTR_Code GetIsUnderrun(int &param);
  CTR_Code GetNextID(int &param);
  CTR_Code GetBufferIsFull(int &param);
  CTR_Code GetCount(int &param);
  CTR_Code GetActTraj_Velocity(int &param);
  CTR_Code GetActTraj_Position(int &param);
  CTR_Code SetDemand(int param);
  CTR_Code SetOverrideLimitSwitchEn(int param);
  CTR_Code SetFeedbackDeviceSelect(int param);
  CTR_Code SetRevMotDuringCloseLoopEn(int param);
  CTR_Code SetOverrideBrakeType(int param);
  CTR_Code SetModeSelect(int param);
  CTR_Code SetProfileSlotSelect(int param);
  CTR_Code SetRampThrottle(int param);
  CTR_Code SetRevFeedbackSensor(int param);

  // ... other stuff ...

};

```

## Usage notes
`src/can_talon_srx_node.cpp` provides some example code of how to use this package.

Essentially, you will need to add this package as a dependency of your ROS package by adding the following into your package's `package.xml`:

```
  <build_depend>can_talon_srx_ros</build_depend>
```

Additionally, you'll need to add this as a dependency in your package's `CMakeLists.txt`

```
catkin_package(
 LIBRARIES can_talon_srx_ros # ... your stuff
 # ... other config stuff
)
```

When you want to use this library, you will need to include the appropriate headers:

```
#include "can_talon_srx/can_base.h"
#include "can_talon_srx/cansocket.h"
#include "wpilib/CanTalonSRX.h"
```

and then somewhere early on in your code, you will need to initialize the library by calling `can_talon_srx::CanSocketInterface::Init`, passing in the name of the CAN interface:

```
can_talon_srx::CanSocketInterface::Init("can0");
```

And then you can use the First Robotics `CanTalonSRX` class as you'd like.
It's recommended that you use `std::shared_ptr<CanTalonSRX>` instead of `CanTalonSRX` directly, because of some problems with moving unwrapped `CanTalonSRX` objects:

```
// make a motor object to control the motor with CAN ID 1
std::shared_ptr<CanTalonSRX> motor = std::shared_ptr<CanTalonSRX>(new CanTalonSRX(1));

// now you can use any of the functions listed in the API above
// set it to duty cycle mode
motor->SetModeSelect(CanTalonSRX::kMode_DutyCycle, 100);

// set motor encoder count
int pos = 0;
motor->GetEncPosition(pos);

// set the motor to move forward at 20% duty cycle (value is duty cycle*1000)
motor->SetDemand(200);
```

The best documentation will be that provided by the First Robotics' version of this code, plus the comments in their code

## Architecture
This library uses First Robotics code with very little modification.
The First Robotics code actually depends on a small C API that does the CAN message transmission and reception, so we basically just provide our own implementations of these functions.
As a result of this, only one CAN port can be used by the library at a time, because the C functions don't let you distinguish calls to different CAN ports.
It would be fairly easy to refactor the code so that it works correctly, but for now this can be worked around by using a single ROS node for each CAN port.

Currently the code uses the Linux CAN interface, so it only works on Linux systems, with CAN interfaces that are correctly detected and configured.
This can be fairly easily refactored to accept any implementation of `CANBase`, you would need to modify `CanSocketInterface::Init()` to use a `shared_ptr<CanInterface>` instead of `const char* interface_name)`.
Basically all you need to do for it to work is set the global variable `can_interface` correctly.
