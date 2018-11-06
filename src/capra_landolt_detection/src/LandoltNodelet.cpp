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
        // Subsciber
        typedef tuple<Point2f, float, float> Gap;
        boost::shared_ptr<image_transport::ImageTransport> it_;
        image_transport::CameraSubscriber sub_camera_;
        int queue_size_;

        // Publisher
        boost::mutex connect_mutex_;
        image_transport::Publisher image_pub_;
        ros::Publisher landolt_pub_;

        vector<float> cached_angles_;

        void connectCb();
        void imageCb(const sensor_msgs::ImageConstPtr &image_msg, 
                     const sensor_msgs::CameraInfoConstPtr& info_msg);
        void findLandoltGaps(const Mat imageRaw, vector<Gap>& gaps, int minEdge, float minRatioCircle, int minDepth);
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
    ros::NodeHandle &nh         = getNodeHandle();
    ros::NodeHandle &private_nh = getPrivateNodeHandle();

    it_.reset(new image_transport::ImageTransport(nh));

    // Read parameters
    private_nh.param("queue_size", queue_size_, 5);
    
    ros::SubscriberStatusCallback connect_cb = boost::bind(&LandoltNodelet::connectCb, this);
    image_transport::SubscriberStatusCallback img_connect_cb = boost::bind(&LandoltNodelet::connectCb, this);
    // Make sure we don't enter connectCb() between advertising and assigning to pub_rect_
    boost::lock_guard<boost::mutex> lock(connect_mutex_);

    landolt_pub_ = nh.advertise<capra_msgs::Landolts>("landolts", 1, connect_cb, connect_cb);
    image_pub_ = it_->advertise("image", 1, img_connect_cb, img_connect_cb);
}

void LandoltNodelet::connectCb()
{
    boost::lock_guard<boost::mutex> lock(connect_mutex_);
    if (landolt_pub_.getNumSubscribers() == 0 && image_pub_.getNumSubscribers() == 0)
    {
        NODELET_INFO("Disconnecting to Landolt Detector...");
        sub_camera_.shutdown();
    }
    else if (!sub_camera_)
    {   
        NODELET_INFO("Connecting to Landolt Detector...");
        image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
        sub_camera_ = it_->subscribeCamera("/capra/camera_3d/rgb/image_raw", queue_size_, &LandoltNodelet::imageCb, this, hints);
    }
}

void LandoltNodelet::imageCb(const sensor_msgs::ImageConstPtr &msg,
                                const sensor_msgs::CameraInfoConstPtr& info_msg)
{
    cv_bridge::CvImagePtr img_ptr;
    try
    {
        img_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    vector<Gap> gaps;  
    findLandoltGaps(img_ptr->image, gaps, 12, 0.8f, 10);

    vector<float> angles;
    for (int i = 0; i < gaps.size(); i++) 
    {
        circle(img_ptr->image, get<0>(gaps[i]), get<1>(gaps[i]), Scalar(0,0,1), 3);
        angles.push_back(get<2>(gaps[i]));
    }

    std_msgs::Header header;
    header.stamp = ros::Time::now();
    if(cached_angles_ == angles)
    {
        cached_angles_ = angles;

        capra_msgs::Landolts landolts;
        landolts.angles = cached_angles_;
        landolts.header = header;
        // Publish the ROS sensor_msgs image
        landolt_pub_.publish(landolts);
    }

    img_ptr->header = header;
    image_pub_.publish(img_ptr->toImageMsg());
}

void LandoltNodelet::findLandoltGaps(const Mat imageRaw, vector<Gap>& gaps, int minEdge, float minRatioCircle, int minDepth)
{
    Mat thresholdMat;
    cvtColor(imageRaw, thresholdMat, COLOR_BGR2GRAY); // convert to grayscale
    blur(thresholdMat, thresholdMat, Size(3, 3)); // apply blur to grayscaled image 
    threshold(thresholdMat, thresholdMat, 140, 255, THRESH_BINARY); // apply binary thresholding

    vector<vector<Point>> contours; // list of contour points
    findContours(thresholdMat, contours, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));

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
                    const Vec4i& v = deepDefects[0];

                    int startidx = v[0];
                    int endidx = v[1];
                    int faridx = v[2];

                    vector<Point> points;
                    points.push_back(Point(contours[i][startidx]));
                    points.push_back(Point(contours[i][endidx]));
                    
                    float radius;
                    Point2f center;
                    minEnclosingCircle(points, center, radius);
                    //Recenter the cercle at the center of the gaps
                    Point2f dir = normalizePoint(Point2f(contours[i][faridx]) - center);
                    center += dir * radius;

                    float angle = angleBetween(Point2f(1, 0), dir) + 180;
                    angle = (int)(angle / 45) * 45;

                    Gap gap;
                    get<0>(gap) = center;
                    get<1>(gap) = radius;
                    get<2>(gap) = angle;
                    gaps.push_back(gap);
                }
            }
        }	
    }

    std::sort(begin(gaps), end(gaps),
    [](Gap const &t1, Gap const &t2) {
        return get<1>(t1) < get<1>(t2); // sort gap by radius
    });
}

PLUGINLIB_EXPORT_CLASS(capra::LandoltNodelet, nodelet::Nodelet)
}
