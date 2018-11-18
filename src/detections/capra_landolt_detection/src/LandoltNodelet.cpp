#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "cv_bridge/cv_bridge.h"
#include "capra_msgs/Landolts.h"
#include "capra_msgs/BoundingCircles.h"
#include "nodelet/nodelet.h"

#include <pluginlib/class_list_macros.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/lock_guard.hpp>

namespace capra
{

struct Gaps {
    std::vector<float> angles;
    std::vector<float> radius;
    std::vector<cv::Point2f> centers;
};

class LandoltNodelet : public nodelet::Nodelet
{
public:
    LandoltNodelet() = default;
    void onInit() override;
private:
    // Subscriber
    boost::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::CameraSubscriber sub_camera_;
    int queue_size_{};

    // Publisher
    boost::mutex connect_mutex_;
    image_transport::Publisher image_pub_;
    ros::Publisher landolt_pub_;
    ros::Publisher bounding_pub_;

    void connectCb();
    void imageCb(const sensor_msgs::ImageConstPtr& image_msg,
                 const sensor_msgs::CameraInfoConstPtr& info_msg);
    void findLandoltGaps(const cv::Mat &imageRaw, Gaps& gaps, int minEdge, float minRatioCircle, int minDepth);
};

float magnitudePoint(const cv::Point2f& diff)
{
    return sqrtf(diff.dot(diff));
}

cv::Point2f normalizePoint(const cv::Point2f& diff)
{
    return diff / magnitudePoint(diff);
}

float angleBetween(cv::Point2f origin, cv::Point2f dest)
{
    float dot = origin.x * dest.x + origin.y * dest.y;  // dot product between[x1, y1] and [x2, y2]
    float det = origin.x * dest.y - origin.y * dest.x;	// determinant
    //Get a angle between [0, 360]
    return atan2f(det, dot) * (float)(180.0 / M_PI) + 180;	// atan2(y, x) or atan2(sin, cos)
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

    bounding_pub_ = nh.advertise<capra_msgs::BoundingCircles>("boundings", 1, connect_cb, connect_cb);
    landolt_pub_ = nh.advertise<capra_msgs::Landolts>("landolts", 1, connect_cb, connect_cb);
    image_pub_ = it_->advertise("image", 1, img_connect_cb, img_connect_cb);
}

void LandoltNodelet::connectCb()
{
    boost::lock_guard<boost::mutex> lock(connect_mutex_);
    if (landolt_pub_.getNumSubscribers() == 0 && 
        image_pub_.getNumSubscribers() == 0 &&
        bounding_pub_.getNumSubscribers() == 0)
    {
        NODELET_INFO("Disconnect to Landolt Detector...");
        sub_camera_.shutdown();
    }
    else if (!sub_camera_)
    {   
        NODELET_INFO("Connect to Landolt Detector...");
        image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
        sub_camera_ = it_->subscribeCamera("/capra/camera_3d/rgb/image_raw", static_cast<uint32_t>(queue_size_), &LandoltNodelet::imageCb, this, hints);
    }
}

void LandoltNodelet::imageCb(const sensor_msgs::ImageConstPtr &image_msg,
                                const sensor_msgs::CameraInfoConstPtr& info_msg)
{
    cv_bridge::CvImageConstPtr img_ptr;
    try
    {
        img_ptr = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    Gaps gaps;  
    findLandoltGaps(img_ptr->image, gaps, 12, 0.8f, 10);

    std_msgs::Header header;
    header.stamp = ros::Time::now();

    if(landolt_pub_.getNumSubscribers() > 0)
    {
        capra_msgs::Landolts landolts;
        landolts.angles = gaps.angles;
        landolts.header = header;

        landolt_pub_.publish(landolts);
    }

    if(bounding_pub_.getNumSubscribers() > 0)
    {
        capra_msgs::BoundingCircles boundings;
        boundings.header = header;
        boundings.radius = gaps.radius;

        std::vector<capra_msgs::Point2f> centers;
        for (auto &i : gaps.centers) {
            capra_msgs::Point2f center;
            center.x = i.x;
            center.y = i.y;
            centers.push_back(center);
        }
        
        boundings.centers = centers;
        bounding_pub_.publish(boundings);
    }

    if(image_pub_.getNumSubscribers() > 0)
    {
        cv_bridge::CvImage img_msg(header, sensor_msgs::image_encodings::BGR8, img_ptr->image);
        for (int i = 0; i < gaps.angles.size(); i++) 
        {
            circle(img_msg.image, gaps.centers[i], static_cast<int>(gaps.radius[i]), cv::Scalar(0, 0, 1), 3);
        }

        image_pub_.publish(img_msg.toImageMsg());
    }
}

void LandoltNodelet::findLandoltGaps(const cv::Mat &imageRaw, Gaps& gaps, int minEdge, float minRatioCircle, int minDepth)
{
    cv::Mat thresholdMat;
    cvtColor(imageRaw, thresholdMat, cv::COLOR_BGR2GRAY); // convert to grayscale
    blur(thresholdMat, thresholdMat, cv::Size(3, 3)); // apply blur to grayscaled image 
    threshold(thresholdMat, thresholdMat, 140, 255, cv::THRESH_BINARY); // apply binary thresholding

    std::vector<std::vector<cv::Point>> contours; // list of contour points
    findContours(thresholdMat, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    for (auto &contour : contours) {

        if (contour.size() > minEdge)
        {
            //@todo: Look if approxPolyDP would give better results.
            //vector<Point> approx;
            //approxPolyDP(contours[i], approx, arcLength(contours[i], true) * 0.001, true);

            std::vector<cv::Point> hull;
            convexHull(contour, hull, true, true);
            double hullArea = cv::contourArea(hull);
            
            float contourRadius;
            cv::Point2f contourCenter;
            minEnclosingCircle(contour, contourCenter, contourRadius);
            double minArea = contourRadius * contourRadius * M_PI;

            if (hullArea / minArea > minRatioCircle)
            {
                std::vector<cv::Vec4i> defects;
                std::vector<int> hullsI;
                convexHull(contour, hullsI, true, false);
                convexityDefects(contour, hullsI, defects);

                std::vector<cv::Vec4i> deepDefects;
                for (const auto &v : defects) {
                    float depth = (float)v[3] / 256;
                    if (depth > minDepth)
                    {
                        deepDefects.push_back(v);
                    }
                }

                if (deepDefects.size() == 1)
                {
                    const cv::Vec4i& v = deepDefects[0];

                    int startidx = v[0];
                    int endidx = v[1];
                    int faridx = v[2];

                    std::vector<cv::Point> points;
                    points.emplace_back(contour[startidx]);
                    points.emplace_back(contour[endidx]);
                    
                    float defectRadius;
                    cv::Point2f defectCenter;
                    minEnclosingCircle(points, defectCenter, defectRadius);
                    //Recenter the cercle at the defectCenter of the gaps
                    cv::Point2f dir = normalizePoint(cv::Point2f(contour[faridx]) - defectCenter);
                    defectCenter += dir * defectRadius;

                    float defectAngle = angleBetween(dir, cv::Point2f(1, 0));
                    
                    gaps.angles.push_back(defectAngle);
                    gaps.radius.push_back(defectRadius);
                    gaps.centers.push_back(defectCenter);
                }
            }
        }	
    }
}

PLUGINLIB_EXPORT_CLASS(capra::LandoltNodelet, nodelet::Nodelet)
}
