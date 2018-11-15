// Bring in my package's API, which is what I'm testing
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <capra_msgs/Landolts.h>
#include <capra_msgs/BoundingCircles.h>
#include <sensor_msgs/CameraInfo.h>

struct MetaData {
    std::vector<float> angles;
    std::vector<float> radius;
    std::vector<cv::Point2f> centers; 
};

class LandoltTest : public testing::Test
{
protected:
    virtual void SetUp()
    {
        topic_angles_ = "landolts";
        topic_images_ = "image";
        topic_boundings_ = "boundings";

        nh_.getParam("datapath", data_path_);

        // Taken from vision_opencv/image_geometry/test/utest.cpp
        double D[] = {-0.363528858080088, 0.16117037733986861, -8.1109585007538829e-05, -0.00044776712298447841, 0.0};
        double K[] = {430.15433020105519,                0.0, 311.71339830549732,
                      0.0, 430.60920415473657, 221.06824942698509,
                      0.0,                0.0,                1.0};
        double R[] = {0.99806560714807102, 0.0068562422224214027, 0.061790256276695904,
                      -0.0067522959054715113, 0.99997541519165112, -0.0018909025066874664,
                      -0.061801701660692349, 0.0014700186639396652, 0.99808736527268516};
        double P[] = {295.53402059708782, 0.0, 285.55760765075684, 0.0,
                      0.0, 295.53402059708782, 223.29617881774902, 0.0,
                      0.0, 0.0, 1.0, 0.0};

        cam_info_.header.frame_id = "tf_frame";
        cam_info_.height = 480;
        cam_info_.width  = 640;
        // No ROI
        cam_info_.D.resize(5);
        std::copy(D, D+5, cam_info_.D.begin());
        std::copy(K, K+9, cam_info_.K.begin());
        std::copy(R, R+9, cam_info_.R.begin());
        std::copy(P, P+12, cam_info_.P.begin());

        // Create raw camera subscriber and publisher
        angles_sub_ = nh_.subscribe<capra_msgs::Landolts>(topic_angles_, 1, &LandoltTest::landoltsCallback, this);
        bound_sub_ = nh_.subscribe<capra_msgs::BoundingCircles>(topic_boundings_, 1, &LandoltTest::boundingsCallback, this);

        image_transport::ImageTransport it(nh_);
        image_sub_ = it.subscribe(topic_images_, 1, &LandoltTest::imageCallback, this);
        cam_pub_ = it.advertiseCamera("/capra/camera_3d/rgb/image_raw", 1);
        //Wait for all publisher and subscriber to connect
        while(angles_sub_.getNumPublishers() == 0 || 
            bound_sub_.getNumPublishers() == 0 || 
            image_sub_.getNumPublishers() == 0 ||
            cam_pub_.getNumSubscribers() == 0)
        {
            ros::spinOnce();
        }
    }

    std::string data_path_;

    ros::NodeHandle nh_;
    std::string topic_angles_;
    std::string topic_boundings_;
    std::string topic_images_;

    int callback_count_;
    cv::Mat received_image_;
    std::vector<float> received_landolts_;
    std::vector<capra_msgs::Point2f> received_centers_;
    std::vector<float> received_radius_;

    sensor_msgs::CameraInfo cam_info_;

    MetaData image_meta_;

    ros::Subscriber angles_sub_;
    ros::Subscriber bound_sub_;
    image_transport::Subscriber image_sub_;
    image_transport::CameraPublisher cam_pub_;

public:
    void boundingsCallback(const capra_msgs::BoundingCircles::ConstPtr& msg)
    {
        received_centers_ = msg->centers;
        received_radius_ = msg->radius;
        callback_count_++;
    }

    void landoltsCallback(const capra_msgs::Landolts::ConstPtr& msg)
    {
        received_landolts_ = msg->angles;
        callback_count_++;
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImageConstPtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_FATAL("cv_bridge exception: %s", e.what());
            return;
        }
        received_image_ = cv_ptr->image.clone();
        callback_count_++;
    }

    bool publishImage(const std::string& imagePath, const std::string& metaPath)
    {
        callback_count_ = 0;

        cv::Mat mat = cv::imread(imagePath);

        if(mat.empty())
        {
            return false;
        }

        sensor_msgs::ImagePtr img = cv_bridge::CvImage(std_msgs::Header(),
                                            sensor_msgs::image_encodings::BGR8, mat).toImageMsg();
        cam_pub_.publish(*img, cam_info_);
        readMetaData(metaPath);

        while(callback_count_ < 3)
        {
            ros::spinOnce();
        }
        return true;
    }

    void readMetaData(const std::string& path)
    {
        image_meta_.angles.clear();
        image_meta_.centers.clear();
        image_meta_.radius.clear();

        cv::FileStorage fs(path, cv::FileStorage::READ);

        fs["radius"] >> image_meta_.radius;
        fs["angles"] >> image_meta_.angles;

        cv::FileNode centersNode = fs["centers"];
        cv::FileNodeIterator it = centersNode.begin(), it_end = centersNode.end();
        for( ; it != it_end; ++it)
        {
            image_meta_.centers.emplace_back((float)(*it)["x"], (float)(*it)["y"]);
        }
        fs.release();
    }
};

TEST_F(LandoltTest, testImages)
{
    std::string file_names[][2] = {
        { "landolt-c.png", "landolt-c.yml" }
    };

    size_t images_count = sizeof(file_names) / sizeof(file_names[0]);
    for (int i = 0; i < images_count; ++i)
    {
        if(!publishImage(data_path_ + file_names[i][0], data_path_ + file_names[i][1]))
            FAIL();

        size_t size = received_landolts_.size();

        EXPECT_EQ(size, received_centers_.size());
        EXPECT_EQ(size, received_radius_.size());

        EXPECT_EQ(size, image_meta_.angles.size());
        EXPECT_EQ(size, image_meta_.radius.size());
        EXPECT_EQ(size, image_meta_.centers.size());

        const double radiusError = 3.0;
        const double boundingError = 3.0;
        const double angleError = 1.0;

        for(int j = 0; j < size; j++)
        {
            bool hasFound = false;
            for(int k = 0; k < size; k++)
            {
                //Search a landolt at the landolt position in the image
                if(std::abs(image_meta_.centers[k].x - received_centers_[j].x) < boundingError &&
                   std::abs(image_meta_.centers[k].y - received_centers_[j].y) < boundingError &&
                    std::abs(image_meta_.radius[k] - received_radius_[j]) < radiusError &&
                    std::abs(image_meta_.angles[k] - received_landolts_[j]) < angleError)
                {
                    //Check if the radius and angle is the near the two values
                    hasFound = true;
                    break;
                }
            }
            EXPECT_TRUE(hasFound);
        }
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "landolt_test");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}