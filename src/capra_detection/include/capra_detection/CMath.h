#include <opencv2/opencv.hpp>
#include <stdio.h>

namespace cmath 
{
    #define constexpr auto M_PI = 3.14159265358979323846;

    float magnitudePoint(cv::Point2f diff);
    cv::Point2f normalizePoint(cv::Point2f diff);
    float angleBetween(cv::Point2f origin, cv::Point2f dest);
};