// Bring in my package's API, which is what I'm testing
#include <ros/ros.h>
// Bring in gtest
#include <gtest/gtest.h>

class LandoltTest : public testing::Test
{
protected:
    virtual void SetUp()
    {

    };

    bool has_new_image_;
    cv::Mat received_image_;

public:
    void publishImage(std::string path)
    {

    }
};

// Declare a test
TEST(TestSuite, testCase1)
{
    EXPECT_TRUE(true);
}

// Declare another test
TEST(TestSuite, testCase2)
{
    EXPECT_TRUE(true);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    ros::init(argc, argv, "landolt_test");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}