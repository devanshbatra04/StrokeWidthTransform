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

// struct SWTPoint2d {
//     int x;
//     int y;
//     float SWT;
// };

// typedef std::pair<SWTPoint2d, SWTPoint2d> SWTPointPair2d;
// typedef std::pair<cv::Point, cv::Point>   SWTPointPair2i;

// struct Point2dFloat {
//     float x;
//     float y;
// };

// struct Ray {
//         SWTPoint2d p;
//         SWTPoint2d q;
//         std::vector<SWTPoint2d> points;
// };

// struct Point3dFloat {
//     float x;
//     float y;
//     float z;
// };


// struct Chain {
//     int p;
//     int q;
//     float dist;
//     bool merged;
//     Point2dFloat direction;
//     std::vector<int> components;
// };

// bool Point2dSort (SWTPoint2d const & lhs,
//                   SWTPoint2d const & rhs);

// cv::Mat textDetection (const cv::Mat& input, bool dark_on_light);

// void strokeWidthTransform (const cv::Mat& edgeImage,
//                            cv::Mat& gradientX,
//                            cv::Mat& gradientY,
//                            bool dark_on_light,
//                            cv::Mat& SWTImage,
//                            std::vector<Ray> & rays);

// void SWTMedianFilter (cv::Mat& SWTImage, std::vector<Ray> & rays);

// std::vector< std::vector<SWTPoint2d> > findLegallyConnectedComponents (cv::Mat& SWTImage, std::vector<Ray> & rays);

// std::vector< std::vector<SWTPoint2d> >
// findLegallyConnectedComponentsRAY (IplImage * SWTImage,
//                                 std::vector<Ray> & rays);

// void componentStats(IplImage * SWTImage,
//                                         const std::vector<SWTPoint2d> & component,
//                                         float & mean, float & variance, float & median,
//                                         int & minx, int & miny, int & maxx, int & maxy);

// void filterComponents(cv::Mat& SWTImage,
//                       std::vector<std::vector<SWTPoint2d> > & components,
//                       std::vector<std::vector<SWTPoint2d> > & validComponents,
//                       std::vector<Point2dFloat> & compCenters,
//                       std::vector<float> & compMedians,
//                       std::vector<SWTPoint2d> & compDimensions,
//                       std::vector<SWTPointPair2d > & compBB );

// std::vector<Chain> makeChains( const cv::Mat& colorImage,
//                  std::vector<std::vector<SWTPoint2d> > & components,
//                  std::vector<Point2dFloat> & compCenters,
//                  std::vector<float> & compMedians,
//                  std::vector<SWTPoint2d> & compDimensions,
//                  std::vector<SWTPointPair2d > & compBB);

}

#endif // TEXTDETECTION_H