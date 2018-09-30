#include <cmath.h>

float cmath::magnitudePoint(cv::Point2f diff) 
{
	return sqrtf(diff.dot(diff));
}

cv::Point2f cmath::normalizePoint(cv::Point2f diff) 
{
    return diff / magnitudePoint(diff);
}

float cmath::angleBetween(cv::Point2f origin, cv::Point2f dest)
{
	float dot = origin.x * dest.x + origin.y * dest.y;  // dot product between[x1, y1] and [x2, y2]
	float det = origin.x * dest.y - origin.y * dest.x;	// determinant
	return atan2(det, dot);								// atan2(y, x) or atan2(sin, cos)
}