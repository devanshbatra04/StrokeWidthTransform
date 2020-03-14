#ifndef TEXTDETECTION_H
#define TEXTDETECTION_H

#include <opencv2/core/core.hpp>
#include <vector>

namespace SWT {
    struct SWTPoint {
        int x;
        int y;
        float SWT;
    };

    struct Ray {
        SWTPoint p;
        SWTPoint q;
        std::vector<SWTPoint> points;
    };

    struct Component {
        SWTPoint BB_pointP;
        SWTPoint BB_pointQ;
        float cx;
        float cy;
        float median;
        float mean;
        int length, width;
        std::vector<SWTPoint> points;
    };

    struct ComponentAttr {
        float mean, variance, median;
        int xmin, ymin;
        int xmax, ymax;
        float length, width;
    };

    struct ChannelAverage {
        float Red, Green, Blue;
    };

    const Scalar BLUE (255, 0, 0);
    const Scalar GREEN(0, 255, 0);
    const Scalar RED  (0, 0, 255);
}

#endif // TEXTDETECTION_H