#include <CapraAlgorithm.h>

using namespace cv;
using namespace std;

void calgo::findLandolt(vector<vector<Point>>& contours, vector<array<Point, 3>> gaps, int minEdge = 12, float minRatioCircle = 0.8f, int minDepth = 10)
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
				/// Draw convexityDefects
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
