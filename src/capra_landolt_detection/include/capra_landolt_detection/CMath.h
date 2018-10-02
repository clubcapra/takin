#include <opencv2/opencv.hpp>
#include <stdio.h>

namespace capra 
{
    namespace cmath 
    {
        float magnitudePoint(cv::Point2f diff);
        cv::Point2f normalizePoint(cv::Point2f diff);
        float angleBetween(cv::Point2f origin, cv::Point2f dest);
    }
}