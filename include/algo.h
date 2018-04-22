#pragma once

#include<iostream>
#include<vector>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <Kinect.h>

void BFS_Online(cv::Mat &InputMat, cv::Mat &LabelMat, cv::Point2i cur_SP, cv::Point3i &cCenter, int Thresh);
void Gen_CoMat1(double CoeMat[4], double addTi, double addDi);
void Gen_CoMat2(double CoeMat[12], double addTi, double addDi);
double forwardPred1(double timePoint, std::vector<double>&Input_CoMat);
double forwardPred2(double timePoint, std::vector<double>&Input_CoMat);
