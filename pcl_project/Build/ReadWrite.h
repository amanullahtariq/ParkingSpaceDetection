
#pragma region Standard Library
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>
#pragma endregion

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/SVD>
#include <boost\thread.hpp>

#pragma region OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#pragma endregion

#pragma region Custom Headers
#include "Constant.h"
#pragma endregion

#pragma region Namespace
using namespace std;
#pragma endregion
#pragma once
class ReadWrite
{
public:
	ReadWrite(void);
	~ReadWrite(void);
	void GetModelText(bool is_car,float height,float length,float width, float Area, float Aspect1_lw, float Aspect2_lh, float Aspect3_wh);
	void ReadOutputText(vector<int> &clust, bool runAgain);
	std::vector<cv::Point2f> ReadWrite:: GetAllRectangles();
};

