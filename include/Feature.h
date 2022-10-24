#pragma once

#include <vector>
#include "Triangulate.h"
#include "eigen3/Eigen/Dense"
#include "opencv2/features2d.hpp"
#include "opencv2/stitching.hpp"
#include "opencv2/video.hpp"

#define ERRORSIZE 25
#define MAXCORNER 1000
#define QUALITYLEVEL 0.01
#define MINDISTANCE 8


namespace svo
{
    class Feature
    {
    public:
        Feature();
        ~Feature()=default;
        
        bool CornerFAST(const cv::Mat& src);
        bool GoodFeaturesToTrack(const cv::Mat& src);
        bool OpticalFlowPyrLK(const cv::Mat& src1, const cv::Mat& src2, svo::Feature& next);

        std::vector<uchar> mstatus;
        std::vector<float> merr;
        std::vector<cv::Point2f> mfeatures;
    };
}//namespace svo

void ManageTrackPoints(const svo::Feature& present, svo::Feature& before);
bool ManageMapPoints(const std::vector<uchar>& mstatus, std::vector<cv::Point3f>& map);
bool ManageMinusZ(svo::Triangulate& map, cv::Mat& R, std::vector<int>& id);
bool ManageMinusZSFM(svo::Triangulate& map, cv::Mat& R, std::vector<int>& id);
bool ManageMinusLocal(std::vector<svo::Feature>& localTrackPoints, const std::vector<int>& id);
void ManageInlier(std::vector<svo::Feature>& features2d, std::vector<cv::Point3f>& mapPoints3d, const cv::Mat& inlier);