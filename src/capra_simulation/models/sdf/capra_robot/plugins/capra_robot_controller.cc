#ifndef CAPRA_ROBOT_CONTROLLER
#define CAPRA_ROBOT_CONTROLLER

#include <functional>
#include <memory>
#include <thread>
#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include <geometry_msgs/Twist.h>

namespace gazebo 
{

class CapraRobotController : public ModelPlugin
{
    std::unique_ptr<ros::NodeHandle> ros_node_;
    ros::Subscriber ros_sub_;
    ros::CallbackQueue ros_callback_queue_;
    std::thread ros_queue_thread_;

    physics::ModelPtr model_;

    event::ConnectionPtr update_connection_;

    // linear velocity params for /cmd_vel
    float x_, y_, z_;
    // angular velocity params for /cmd_vel
    float roll_, pitch_, yaw_;

    /// \brief ROS helper function that processes messages
    void QueueThread()
    {
        static const double timeout = 0.01;
        while (this->ros_node_->ok())
        {
            this->ros_callback_queue_.callAvailable(ros::WallDuration(timeout));
        }
    }

public:

    virtual void Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
    {
        this->model_ = parent;

        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->update_connection_ = 
            event::Events::ConnectWorldUpdateBegin(
                std::bind(&CapraRobotController::OnUpdate, this)
            );

        if (!ros::isInitialized())
        {
            int argc = 0;
            char **argv = NULL;
            ros::init(
                argc,
                argv,
                "capra_simulation/robot_controller",
                ros::init_options::NoSigintHandler
            );
        }

        // Initialize the ptr to a new node handle
        this->ros_node_.reset(
            new ros::NodeHandle("capra_simulation/robot_controller")
        );
        
        ros::SubscribeOptions options =
            ros::SubscribeOptions::create<geometry_msgs::Twist>(
                // Navigation stack publishes on /cmd_vel topic
                "/capra_simulation/cmd_vel",
                10000,
                boost::bind(&CapraRobotController::OnRosMsg, this, _1),
                ros::VoidPtr(), &this->ros_callback_queue_
            );

        // TODO : Verify necessity to have a ros::Subscriber member variable
        this->ros_sub_ = this->ros_node_->subscribe(options);

        // Spin up the queue helper thread.
        this->ros_queue_thread_ =
            std::thread(std::bind(&CapraRobotController::QueueThread, this));
    }

    // Called by the world update start event
    void OnUpdate()
    {
      // Apply a linear velocity to the model.
      this->model_->SetLinearVel(ignition::math::Vector3d(this->x_, this->y_, this->z_));
      // Apply angular velocity to the model
      this->model_->SetAngularVel(ignition::math::Vector3d(this->roll_, this->pitch_, this->yaw_));
    }

    // Update twist values internally on each ros message
    void OnRosMsg(const geometry_msgs::TwistConstPtr msg)
    {
        this->x_ = msg->linear.x;
        this->y_ = msg->linear.y;
        this->z_ = msg->linear.z;
        this->roll_ = msg->angular.x;
        this->pitch_ = msg->angular.y;
        this->yaw_ = msg->angular.z;

        // Debugging example with gzdbg output stream
        // gzdbg << "x : " << this->x << std::endl;
    }

};
    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(CapraRobotController)
}

#endif