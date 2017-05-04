#pragma once
/*****************************************************************************************
 * helperFunctions.h
 *
 *       Created on: 12-Mar-2015
 *    Last Modified: 12-Sep-2016
 *  Original Author: Anirudh Vemula
 *   Current Author: Meghshyam Govind Prasad
 *   Current Author: Sona Praneeth Akula
 *          Project: Quadcopter
 *      Description: 
 *
 * Date				Author							Modification
 * 12-Sep-2016	Sona Praneeth Akula			Added comments to the code
 * 08-Oct-2016	Sona Praneeth Akula 		Moved certain helper functions from ControlUINode to 
 *											this file
 *****************************************************************************************/

#ifndef _VISIONHELPER_H
#define _VISIONHELPER_H

#include <cmath>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <vector>

#include "Line2.hpp"
#include "Multiple-Plane-JLinkage/utilities.hpp"


using namespace std;
using namespace cv;

/* A simple sign function */
/**
 * @brief @todo Why R-z is being returned which is identity
 * @details
 * @param [float] roll - 
 * @param [float] pitch - 
 * @param [float] yaw - 
 * @return
 */
inline static cv::Mat
getRotationMatrix(float roll, float pitch, float yaw)
{
	roll  = roll*M_PI/180;
	pitch = pitch*M_PI/180;
	yaw   = (-1.0)*yaw*M_PI/180;
	cv::Mat R_x = cv::Mat::eye(3,3, CV_32F);
	cv::Mat R_y = cv::Mat::eye(3,3, CV_32F);
	cv::Mat R_z = cv::Mat::eye(3,3, CV_32F);
	/*
	R_z.at<float>(0,0) = cos(yaw);
	R_z.at<float>(0,2) = -sin(yaw);
	R_z.at<float>(2,0) = sin(yaw);
	R_z.at<float>(2,2) = cos(yaw);
	*/
	return R_z; // Roll & Pitch are not reliable, also most of the time we will have yaw only
}

#endif