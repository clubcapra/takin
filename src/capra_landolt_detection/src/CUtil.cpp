#include "CUtil.h"

namespace capra
{
    namespace cutil 
    {
        cv::Scalar randomColor(cv::RNG& rng)
		{
			int icolor = (unsigned)rng;
			return cv::Scalar(icolor & 255, (icolor >> 8) & 255, (icolor >> 16) & 255);
		}
    }
}