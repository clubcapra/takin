#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cv_bridge/cv_bridge.h>
#include <CapraAlgorithm.h>
#include <CMath.h>
#include <CUtil.h>
#include <string>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "capra_landolt_publisher");
    
    // Public node handle
    ros::NodeHandle nh;
    // Get image transportation and publishing media
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("capra/landolt_c/image", 1);

    RNG rng(0x0FFFFFFF);
    VideoCapture capture(0);

    Mat sourceMat, thresholdMat;

    while(ros::ok() && capture.read(sourceMat))
    {
        cvtColor(sourceMat, thresholdMat, COLOR_BGR2GRAY); // convert to grayscale
        blur(thresholdMat, thresholdMat, Size(3, 3)); // apply blur to grayscaled image 
        threshold(thresholdMat, thresholdMat, 140, 255, THRESH_BINARY); // apply binary thresholding

        vector<vector<Point>> contours; // list of contour points
        findContours(thresholdMat, contours, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));

        vector<array<Point, 3>> gaps;
        calgo::findLandolt(contours, gaps);

        for (int i = 0; i < gaps.size(); i++) 
        {
            Scalar color = cutil::randomColor(rng); //Debug

            vector<Point> points;
            points.push_back(gaps[i][0]);
            points.push_back(gaps[i][1]);

            float radius;
            Point2f center;
            minEnclosingCircle(points, center, radius);
            //Recenter the cercle at the center of the gaps
            center += cmath::normalizePoint(Point2f(gaps[i][2]) - center) * radius;
            circle(sourceMat, center, radius, color, 3, 8, 0);
        }

        // Convert opencv image to ROS sensor_msgs image
        sensor_msgs::ImageConstPtr msg =
            cv_bridge::CvImage(
                std_msgs::Header() /* empty header */,
                "bgr8" /* image format */,
                sourceMat /* the opencv image object */
            ).toImageMsg();

        // Publish the ROS sensor_msgs image
        pub.publish(msg);

        ros::spinOnce();
    }

    return 0;
}