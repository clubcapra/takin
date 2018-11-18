#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "capra_qrcode_static_publisher");

    // Public node handle
    ros::NodeHandle nh;
    // Get image transportation and publishing media
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("capra/qrcode_detection/image", 1);

    // Default image file
    std::string filename {
        "/home/minh/git/Takin/src/capra_qrcode_detection/images/shopify-faae7065b7b351d28495b345ed76096c03de28bac346deb1e85db632862fd0e4.png"
    };

    // Get the image file's system path
    if (argc > 1)
    {
        filename = argv[1];
    }

    // Get the image object from filesystem
    cv::Mat image = cv::imread(filename, cv::IMREAD_COLOR);

    // If no image was found, return immediately
    if (image.empty())
    {
        ROS_ERROR("Could not open or find the image");
        return -1;
    }

    // Publish on ROS timer events
    ros::Timer timer = nh.createTimer(
        ros::Duration(0.1), 
        [&pub, &image] (const ros::TimerEvent& event)
        {
            // Convert opencv image to ROS sensor_msgs image
            sensor_msgs::ImageConstPtr msg =
                cv_bridge::CvImage(
                    std_msgs::Header() /* empty header */,
                    "bgr8" /* image format */,
                    image /* the opencv image object */
                ).toImageMsg();

            // Publish the ROS sensor_msgs image
            pub.publish(msg);
        }
    );

    // Start spinning event loop
    ros::spin();

    return 0;
}