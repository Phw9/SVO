#pragma once

#include <deque>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "eigen3/Eigen/Dense"
#include "Triangulate.h"
#include "PoseEstimation.h"
#include "Feature.h"

#include <pangolin/display/display.h>
#include <pangolin/gl/gl.h>
#include <pangolin/gl/gldraw.h>
#include <pangolin/handler/handler_image.h>
#include <pangolin/image/image_utils.h>
#include <pangolin/display/view.h>
#include <pangolin/handler/handler.h>
#include <pangolin/gl/viewport.h>
#include <pangolin/utils/range.h>

#include <algorithm>
#include <functional>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

std::vector<cv::Vec3f> readtvecOfGT;
std::vector<cv::Vec3f> tvecOfGT;

std::deque<std::string> readLeftImageName;
std::deque<std::string> readRightImageName;
cv::Mat leftImage;
cv::Mat RightImage;

int imageCurNum = 0;
int realFrame = 0;

svo::Feature detector;
svo::Feature trackerA, trackerB;
std::vector<svo::Feature> localTrackPointsA;
int lTPA = 0;
std::vector<svo::Feature> localTrackPointsB;
int lTPB = 0;

std::vector<uchar> stats;

svo::StrctureFromMotion getEssential;
svo::PoseEstimation getPose;

std::vector<int> mapStats;
svo::Triangulate mapPointsA, mapPointsB;
std::vector<cv::Point3f> localMapPointsA, localMapPointsB;
std::vector<svo::Triangulate> globalLandMark;
/// @brief 
int gLM = 0;
int gKF = 0;

float inlierRatio = 1000.0f;
double angularVelocity = 0;
std::vector<cv::Mat> globalRTMat; std::vector<cv::Mat> globalRMat;
std::vector<cv::Vec3d> globalRVec; std::vector<cv::Vec3d> globalTVec;
int gP = 0;

void FileRead(std::deque<std::string>& v, std::ifstream &fin);
void MakeTextFile_Left(std::ofstream& fout, const int& imageNum);
void MakeTextFile_Right(std::ofstream& fout, const int& imageNum);
void GTPoseRead(std::vector<cv::Vec3f>& v, std::ifstream& fin);

namespace Viewer
{
    class MyVisualize
	{
        private:
            int window_width;
            int window_height;

        public:
            float window_ratio;

            //생성자
            MyVisualize(int width,int height);
            void initialize();
            void active_cam();

            // pts1: GT Pose, pts2: Pose, pts3: 3D Points, pts4: FOV of 3D Points
            void DrawPoint(const std::vector<cv::Vec3d>& tvec, 
                            const std::vector<cv::Vec3f>& gtPose,
                            const std::vector<svo::Triangulate>& allOfPoints, 
                            const std::vector<cv::Point3f>& fovPoints);
            // circle is before, rectangle is after
            cv::Mat DrawFeatures(cv::Mat& src, std::vector<cv::Point2f>& beforePoints, std::vector<cv::Point2f>& afterPoints);
    };
}//namespace Viewer