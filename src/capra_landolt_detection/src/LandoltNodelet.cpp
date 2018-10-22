#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "cv_bridge/cv_bridge.h"
#include "capra_msgs/Landolts.h"
#include "nodelet/nodelet.h"

#include <pluginlib/class_list_macros.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/lock_guard.hpp>
#include <string>

using namespace cv;
using namespace std;

namespace capra
{

class LandoltNodelet : public nodelet::Nodelet
{
    public:
        LandoltNodelet() {}
        virtual void onInit();
    private:
        boost::shared_ptr<image_transport::ImageTransport> it_;
        image_transport::CameraSubscriber sub_camera_;
        int queue_size_;

        boost::mutex connect_mutex_;
        ros::Publisher landolt_pub_;

        void connectCb();
        void imageCb(const sensor_msgs::ImageConstPtr &image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);
        void findLandoltGaps(Mat imageRaw, vector<capra_msgs::LandoltGap>& landoltGaps);
        void findLandolt(vector<vector<Point>>& contours, vector<array<Point, 3>>& gaps, int minEdge, float minRatioCircle, int minDepth);
};

float magnitudePoint(cv::Point2f diff) 
{
    return sqrtf(diff.dot(diff));
}

cv::Point2f normalizePoint(cv::Point2f diff) 
{
    return diff / magnitudePoint(diff);
}

float angleBetween(cv::Point2f origin, cv::Point2f dest)
{
    float dot = origin.x * dest.x + origin.y * dest.y;  // dot product between[x1, y1] and [x2, y2]
    float det = origin.x * dest.y - origin.y * dest.x;	// determinant
    return atan2(det, dot);								// atan2(y, x) or atan2(sin, cos)
}

void LandoltNodelet::onInit()
{
    NODELET_INFO("On Init Landolt");
    ros::NodeHandle &nh         = getNodeHandle();
    ros::NodeHandle &private_nh = getPrivateNodeHandle();

    it_.reset(new image_transport::ImageTransport(nh));

    // Read parameters
    private_nh.param("queue_size", queue_size_, 5);
    
    ros::SubscriberStatusCallback connect_cb = boost::bind(&LandoltNodelet::connectCb, this);
    // Make sure we don't enter connectCb() between advertising and assigning to pub_rect_
    boost::lock_guard<boost::mutex> lock(connect_mutex_);

    landolt_pub_ = nh.advertise<capra_msgs::Landolts>("/capra/detection/landolts", 1, connect_cb, connect_cb);
}

void LandoltNodelet::connectCb()
{
    NODELET_INFO("Connect.");
    boost::lock_guard<boost::mutex> lock(connect_mutex_);
    if (landolt_pub_.getNumSubscribers() == 0)
        sub_camera_.shutdown();
    else if (!sub_camera_)
    {   
        image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
        sub_camera_ = it_->subscribeCamera("/capra/camera_3d/rgb/image_raw", queue_size_, &LandoltNodelet::imageCb, this, hints);
    }
}

void LandoltNodelet::imageCb(const sensor_msgs::ImageConstPtr &msg,
                                const sensor_msgs::CameraInfoConstPtr& info_msg)
{
    cv_bridge::CvImageConstPtr img_ptr;
    try
    {
        img_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    vector<capra_msgs::LandoltGap> landoltGaps;    
    findLandoltGaps(img_ptr->image, landoltGaps);

    capra_msgs::Landolts landolts;
    landolts.gaps = landoltGaps;
    landolts.header.stamp = ros::Time::now();
    landolts.image_header = img_ptr->header;
    // Publish the ROS sensor_msgs image
    landolt_pub_.publish(landolts);
}

void LandoltNodelet::findLandoltGaps(const Mat imageRaw, vector<capra_msgs::LandoltGap>& landoltGaps)
{
    Mat thresholdMat;
    cvtColor(imageRaw, thresholdMat, COLOR_BGR2GRAY); // convert to grayscale
    blur(thresholdMat, thresholdMat, Size(3, 3)); // apply blur to grayscaled image 
    threshold(thresholdMat, thresholdMat, 140, 255, THRESH_BINARY); // apply binary thresholding

    vector<vector<Point>> contours; // list of contour points
    findContours(thresholdMat, contours, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));

    vector<array<Point, 3>> gaps;
    findLandolt(contours, gaps, 12, 0.8f, 10);

    for (int i = 0; i < gaps.size(); i++) 
    {
        vector<Point> points;
        points.push_back(gaps[i][0]);
        points.push_back(gaps[i][1]);

        float radius;
        Point2f center;
        minEnclosingCircle(points, center, radius);
        //Recenter the cercle at the center of the gaps
        Point2f dir = normalizePoint(Point2f(gaps[i][2]) - center);
        center += dir * radius;

        capra_msgs::LandoltGap landoltGap;
        landoltGap.gap.radius = radius;
        landoltGap.gap.xcenter = center.x;
        landoltGap.gap.ycenter = center.y;
        landoltGap.angle = angleBetween(Point2f(1, 0), dir) + 180;

        landoltGaps.push_back(landoltGap);
    }
}

void LandoltNodelet::findLandolt(vector<vector<Point>>& contours, vector<array<Point, 3>>& gaps, int minEdge, float minRatioCircle, int minDepth)
{
    for (size_t i = 0; i < contours.size(); i++)
    {
        if (contours[i].size() > minEdge)
        {
            //TODO: Look if approxPolyDP would give better results.
            //vector<Point> approx;
            //approxPolyDP(contours[i], approx, arcLength(contours[i], true) * 0.001, true);

            vector<Point> hull;
            convexHull(contours[i], hull, true, true);
            double hullArea = contourArea(hull);
            
            float radius;
            Point2f center;
            minEnclosingCircle(contours[i], center, radius);
            double minArea = radius * radius * M_PI;

            if (hullArea / minArea > minRatioCircle)
            {
                vector<Vec4i> defects;
                vector<int> hullsI;
                convexHull(contours[i], hullsI, true, false);
                convexityDefects(contours[i], hullsI, defects);

                vector<Vec4i> deepDefects;
                for (int j = 0; j < defects.size(); ++j)
                {
                    const Vec4i& v = defects[j];
                    float depth = (float)v[3] / 256;
                    if (depth > minDepth)
                    {
                        deepDefects.push_back(v);
                    }
                }

                if (deepDefects.size() == 1)
                {
                    array<Point, 3> points;
                    const Vec4i& v = deepDefects[0];

                    int startidx = v[0]; points[0] = Point(contours[i][startidx]);
                    int endidx = v[1]; points[1] = Point(contours[i][endidx]);
                    int faridx = v[2]; points[2] = Point(contours[i][faridx]);

                    gaps.push_back(points);
                }
            }
        }	
    }
}

PLUGINLIB_EXPORT_CLASS(capra::LandoltNodelet, nodelet::Nodelet)
}
