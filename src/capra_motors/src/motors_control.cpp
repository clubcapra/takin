#include "ctre/Phoenix.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include "Platform-linux-socket-can.h"
#include <SDL2/SDL.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>

// SDL code from https://gist.github.com/fabiocolacio/423169234b8daf876d8eb75d8a5f2e95

using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

void joystickCallback(const geometry_msgs::Twist &twist)
{
    ROS_INFO("I heard: [%s]", twist);
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "capra_motors_control");

    ros::NodeHandle n;

    std::string interface = "can0";

    ros::Subscriber remote_control = n.subscribe("capra_remote_controller", 1000, joystickCallback);

    ctre::phoenix::platform::can::SetCANInterface(interface.c_str());

    TalonSRX *talonFL = new TalonSRX(11);
    /*     TalonSRX *talonRL = new TalonSRX(61);
    TalonSRX *talonFL = new TalonSRX(12);
    TalonSRX *talonRL = new TalonSRX(62); */

    // Initialize the joystick subsystem
    SDL_Init(SDL_INIT_JOYSTICK);

    // If there are no joysticks connected, quit the program
    if (SDL_NumJoysticks() <= 0)
    {
        printf("There are no joysticks connected. Quitting now...\n");
        SDL_Quit();
        return -1;
    }

    // Open the joystick for reading and store its handle in the joy variable
    SDL_Joystick *joy = SDL_JoystickOpen(0);

    // If the joy variable is NULL, there was an error opening it.
    if (joy != NULL)
    {
        // Get information about the joystick
        const char *name = SDL_JoystickName(joy);
        const int num_axes = SDL_JoystickNumAxes(joy);
        const int num_buttons = SDL_JoystickNumButtons(joy);
        const int num_hats = SDL_JoystickNumHats(joy);

        printf("Now reading from joystick '%s' with:\n"
               "%d axes\n"
               "%d buttons\n"
               "%d hats\n\n",
               name,
               num_axes,
               num_buttons,
               num_hats);

        int quit = 0;

        // Keep reading the state of the joystick in a loop
        while (quit == 0)
        {
            if (SDL_QuitRequested())
            {
                quit = 1;
            }

            if (SDL_JoystickGetButton(joy, 4))
            {
                ctre::phoenix::unmanaged::FeedEnable(100);
            }

            talonFL->Set(ControlMode::PercentOutput, ((double)SDL_JoystickGetAxis(joy, 1)) / 32767.0);
            /* talonRL->Set(ControlMode::PercentOutput, ((double)SDL_JoystickGetAxis(joy, 1)) / 32767.0); */

            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        SDL_JoystickClose(joy);
    }
    else
    {
        printf("Couldn't open the joystick. Quitting now...\n");
    }

    SDL_Quit();
    return 0;
}
