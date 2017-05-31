#ifndef _HEADERS_H
#define _HEADERS_H
#pragma once

#include "ros/ros.h"

// OpenCV related headers
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

// C Headers
#include <cmath>
#include <ctime>
#include <cstdlib>
#include <cstdio>

// C++ Headers
#include <vector>
#include <list>
#include <string>
#include <chrono>
#include <string>
#include <fstream>
#include <sstream>
#include <list>
#include <map>
#include <algorithm>

// Linux Headers
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

#include "cvd/thread.h"
#include "cvd/image.h"
#include "cvd/byte.h"
#include "cvd/rgb.h"

#include "tum_ardrone/keypoint_coord.h"
#include "tum_ardrone/filter_state.h"

#include "Line2.hpp"
#include "Multiple-Plane-JLinkage/utilities.hpp"

#include "DebugUtility/DebugUtility.hpp"
#include "LogUtility/LogUtility.hpp"

// Namespaces
using namespace std;
using namespace cv;

inline const std::string get_current_time()
{
	std::array<char, 64> buffer;
	buffer.fill(0);
	time_t rawtime;
	time(&rawtime);
	const auto timeinfo = localtime(&rawtime);
	strftime(buffer.data(), sizeof(buffer), "%H:%M:%S", timeinfo);
	std::string timeStr(buffer.data());
	return timeStr;
}


#endif