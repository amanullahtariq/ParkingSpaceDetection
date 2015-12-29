#pragma region Libraries
#include "stdafx.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "MyLines.h"
#include "LineRelationsAngle.h"
#include "LinesByLength.h"
#include "MidPointRelations.h"
#include "ParkingSpots.h"
#include "RectDetail.h"
#include <stdlib.h>
#include <list>
#include <fstream>
#include <string>
#include <iomanip>
#include <cfloat>
#include <math.h>    

#pragma region Namespace
using namespace std;
#pragma endregion

#pragma once
class ParkingSpaceDetection
{
public:
	ParkingSpaceDetection(void);
	~ParkingSpaceDetection(void);
	std::vector<ParkingSpots> main();
	cv::Mat GetImage(string path);

private :
	#pragma region Methods Declaration
	//void LoadHough(string path);
	void ParkingSpaceDetection:: LoadHough(string path);
	void cornerHarris_demo(int, void*);
	int display_caption(char* caption);
	int display_dst(int delay);
	void CheckLineExistance(MyLines lin);
	cv::Point2f computeIntersect(cv::Vec4i a, cv::Vec4i b);
	void sortCorners(std::vector<cv::Point2f>& corners, cv::Point2f center);
	void ApplyHoughTransformation();
	void BlurImage(cv::Mat src);
	
	void DrawRectangle(cv::Mat &drawing, float x1, float y1, float x2, float y2);
	vector<int> GetMaxAngleIndex(vector<LineRelationsAngle> lineDetail);
	cv::Mat ApplyLineSegmentation(cv::Mat grayImage, cv::Mat srcImage);
	float getDistanceBetweenTwoPoints(float x1, float  y1, float x2, float y2, float slope);
	float get_euclidean_distance_between_two_points(float x1, float  y1, float x2, float y2);
	vector<float> find_perpendicular_point_of_intersection_deep(float mid_x1, float mid_y1, float mid_x2, float mid_y2, float slope_i,float slope_j);
	void CategorizeByAngle(vector<cv::Vec4f> lines);
	void CategorizeByLength(int index);
	void FindMidPointDistances(int index);
	bool CheckIfLineAreSame(float x1, float x2, float y1, float y2);
	float GetLineLength(MyLines line1, MyLines line2);
	void UpdatePoints(cv::Point p1, cv::Point p2, cv::Point &p3, cv::Point &p4);
	void FindPerpendicularPoints(cv::Point p1, cv::Point &p2, int rectLength, int rectWidth);
	void CalculateRectanglePoints(MyLines line1, MyLines line2, int rectLength, int rectWidth);
	int FindMostOccuringLength();
	int FindMostOccuringWidth();
	void FindAllRectangles(int index);
	
	vector<float> find_perpendicular_point_of_intersection(
	float mid_x1, 
	float mid_y1,
	float x1,
	float y1,
	float x2,
	float y2, 
	float distance,			// width
	bool move_left
	);
	#pragma endregion
};

