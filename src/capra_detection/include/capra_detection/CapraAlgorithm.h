#include <opencv2/opencv.hpp>
#include <stdio.h>

namespace calgo 
{
    using namespace cv;
    using namespace std;
    
    void findLandolt(vector<vector<Point>>& contours, vector<array<Point, 3>> gaps, int minEdge = 12, float minRatioCircle = 0.8f, int minDepth = 10);
};