#include "ParkingSpaceDetection.h"
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

#pragma region Namespace
using namespace std;
#pragma endregion

#pragma region Global Variables

//cv::Mat blurImg, canny_output;
char* source_window = "Source image";
//cv::Mat src, src_gray;
cv::Mat dst, dst2, detected_edges;
int thresh = 100;
int max_thresh = 255;
cv::RNG rng(12345);
float maxlinelength;
int edgeThresh = 1;
int lowThreshold;
int const max_lowThreshold = 100;
int ratio = 3;
int kernel_size = 3;

#pragma endregion

#pragma region Vectors
std::vector<MyLines> mylines;
std::vector<LineRelationsAngle> mylinerelations;
std::vector<MidPointRelations> midpointrelations;
std::vector<LinesByLength> linesbylength;
std::vector<ParkingSpots> spotsArray;
//std::vector<float, vector<MyLines>> linecategories;
#pragma endregion

ParkingSpaceDetection::ParkingSpaceDetection(void)
{
}

ParkingSpaceDetection::~ParkingSpaceDetection(void)
{
}

#pragma region Methods

float ParkingSpaceDetection:: getDistanceBetweenTwoPoints(float x1, float  y1, float x2, float y2, float slope)
{
	if (!_finite(slope))
	{
		return std::abs(x2 - x1);
	}
	else if (slope == 0)
	{
		return std::abs(y2 - y1);
	}
	else
	{
		float b1 = y1 - (slope*x1);
		float b2 = y2 - (slope*x2);

		return (std::abs(b2 - b1)) / sqrt(powf(slope, 2) + 1);
	}
}

float ParkingSpaceDetection:: get_euclidean_distance_between_two_points(float x1, float  y1, float x2, float y2)
{
	return sqrt(pow(y2 - y1, 2) + pow(x2 - x1, 2));
}
vector<float> ParkingSpaceDetection::find_perpendicular_point_of_intersection(
	float mid_x1, 
	float mid_y1,
	float x1,
	float y1,
	float x2,
	float y2, 
	float distance,			// width
	bool move_left
	)
{
	vector<float> point;

	if (move_left == false)
	{
		point.push_back(mid_x1 + (distance / sqrt(pow(y1 - y2, 2) + pow(x2 - x1, 2))*(y1 - y2)));
		point.push_back(mid_y1 + (distance / sqrt(pow(y1 - y2, 2) + pow(x2 - x1, 2))*(x2 - x1)));
	}
	else
	{
		point.push_back(mid_x1 - (distance / sqrt(pow(y1 - y2, 2) + pow(x2 - x1, 2))*(y1 - y2)));
		point.push_back(mid_y1 - (distance / sqrt(pow(y1 - y2, 2) + pow(x2 - x1, 2))*(x2 - x1)));
	}
	return point;
}
vector<float> ParkingSpaceDetection:: find_perpendicular_point_of_intersection_deep(float mid_x1, float mid_y1, float mid_x2, float mid_y2, float slope_i,float slope_j)
{
	vector<float> point;

	if (!_finite(slope_i))
	{
		point.push_back(mid_x2);
		point.push_back(mid_y1);
	}
	else if (slope_i == 0)
	{
		point.push_back(mid_x1);
		point.push_back(mid_y2);
	}
	else
	{
		float m1 = -1 / slope_i;
		float m2 = slope_j;

		float b1 = mid_y1 - (m1*mid_x1);
		float b2 = mid_y2 - (m2*mid_x2);

		float y = (b1*(m2 / m1) - b2) / ((m2 / m1) - 1);
		float x = (b1 + y) / m1;


		point.push_back(x);
		point.push_back(y);
	}

	return point;
}

//vector<float> find_perpendicular_point_of_intersection(float x1, float y1, float x2, float  y2, float distance)
//{
////	distance / sqrt(pow(y1-y2,2)+pow(x2-x1,2))
//}

void ParkingSpaceDetection:: CheckLineExistance(MyLines lin)
{
	// MyLineRelations is an array of objects that keeps all the lines with same angle.
	// so here first we loop MyLineRelations to find if we already have a category(in terms of angle)
	// for this line, if yes we add the line to the MyLineRelations, otherwise we create a new category
	// i.e add a new LineRelationsAngle object to the MyLineRelations.

	// mylinerelations is an array object of the class LineRelationsAngle

	/*
		it inserts all the lines with the same angle into the mylinerelations array.
	*/
	for (int i = 0; i < mylinerelations.size(); i++)
	{
		if (mylinerelations[i].getAngle() - 5 <= lin.getAngle() && mylinerelations[i].getAngle() + 5 >= lin.getAngle())
		{

			mylinerelations[i].Add(lin);
			return;
		}
	}

	// Add the line with new angle
	mylinerelations.push_back(*new LineRelationsAngle(lin));
}

/// Why we are using it ?? Muazzam ALI

void ParkingSpaceDetection:: CategorizeByAngle(vector<cv::Vec4f> lines)
{	
	// this function uses the lines vector which contains p(0,0) and p(1,1) i.e point 1 and point 2 locations 
	// (connecting the line together) of all the lines. 

	for (int i = 0; i < lines.size(); i++)
	{
		cv::Vec4i l = lines[i];
		float num1 = (l[2] - l[0]);
		float num2 = (l[3] - l[1]);

		// create a new MyLines object and save the lines points and angle from the origin
		CheckLineExistance(*new MyLines(l[0], l[1], l[2], l[3], (atan2(num1,num2) * 180 / 3.14)));
	}

}

void ParkingSpaceDetection:: CategorizeByLength(int index)
{
	vector<MyLines> lineslengt = mylinerelations[index].getLines();
	bool checked = false;
	for (int i = 0; i < lineslengt.size(); i++)
	{
		checked = false;
		for (int j = 0; j < linesbylength.size(); j++)
		{

			// categorizing lines by length here.Since we can't do the exact match
			// we're considering a range for the lengths
			if (lineslengt[i].getlength() - 3 <= linesbylength[j].getlength()
				&& lineslengt[i].getlength() + 3 >= linesbylength[j].getlength())
			{
				linesbylength[j].Add(lineslengt[i]);
				checked = true;
				break;
			}
		}

		// if we couldn't find the line in the range, we make a new category
		if (checked == false)
		{
			linesbylength.push_back(*new LinesByLength(lineslengt[i]));
		}
	}
}


void ParkingSpaceDetection:: FindMidPointDistances(int index)
{
	bool checked = false;
	bool saved = false;
	vector<MyLines> lines = mylinerelations[index].getLines();
	//float rectlength = 0;
	//// looping all the lines w.r.t each other. let's say line with i index is parent
	for (int i = 0; i < lines.size() - 1; i++)
	{
	//	int rectlength = 0;
	//	if (lines[i].getlength() < maxlinelength)
	//	{
	//		if (lines[i].getlength() / maxlinelength <= .99 && lines[i].getlength() / maxlinelength >= .50)
	//		{
	//			rectlength = maxlinelength;
	//		}
	//		else
	//		{
	//			continue;
	//		}
	//	}
	//	else
	//	{
	//		//if our line i is 2 times greater than our standard line.
	//		if (lines[i].getlength() > maxlinelength * 3) continue;

	//		rectlength = maxlinelength;
	//	}

		// line with j index is child
		for (int j = i + 1; j < lines.size(); j++)
		{



			// if we're considering the same lines as parent and child than ofcourse the distance between their
			// midpoints is 0 because both are the same line
			if (lines.at(i).getMidPointX() == lines.at(j).getMidPointX()
				&& lines.at(i).getMidPointY() == lines.at(j).getMidPointY())
			{
				continue;
			}


			// finding the distance between the midpoints of the parent and child lines, this could be too large
			// if the lines are placed at different sides of the image.
			float perp_dist_bw_midpoints = getDistanceBetweenTwoPoints(
				lines[i].getMidPointX(),
				lines[i].getMidPointY(),
				lines[j].getMidPointX(),
				lines[j].getMidPointY(),
				lines[i].getSlope());

			float non_perp_dist_bw_midpoints = get_euclidean_distance_between_two_points(
				lines[i].getMidPointX(),
				lines[i].getMidPointY(),
				lines[j].getMidPointX(),
				lines[j].getMidPointY());

			//float rectlength = get_euclidean_distance_between_two_points(lines[i].getX1(), lines[i].getY1()
			//	, lines[j].getX2(), lines[j].getY2());
			float rectlength = 0;
					// checking if the length of the parent is less than the child
					if (lines[i].getlength() < lines[j].getlength())
					{
						// if parent < child

						// maxlinelength here is referring to the length of the parent line(based on which all the other
						// lines were saved in that range) from the max occuring line category
						if (lines[i].getlength() / maxlinelength <= .99 && lines[i].getlength() / maxlinelength >= .50)
						{
							rectlength = maxlinelength;
						}
						else
						{
														rectlength = lines[i].getlength();
							//continue;
						}
					}
					else
						rectlength = lines[i].getlength();


			vector<float> new_mid_point = find_perpendicular_point_of_intersection(
				lines[i].getMidPointX(),
				lines[i].getMidPointY(),
				lines[i].getX1(),
				lines[i].getY1(),
				lines[i].getX2(),
				lines[i].getY2(),
				perp_dist_bw_midpoints,
				false);


			float aspectratio = perp_dist_bw_midpoints / rectlength;
			//float rectangleslength = get_euclidean_distance_between_two_points(lines[i].getX1(), lines[i].getY1(), lines[i].getX2(), lines[i].getY2());

			if ((perp_dist_bw_midpoints < 22 || perp_dist_bw_midpoints>26) &&
				(rectlength<50 || rectlength>60))
				continue;

			if (aspectratio<.4 || aspectratio >.6)
			{
				continue;
			}

			/*if (aspectratio<.1 || aspectratio >.2 )
			{
				continue;
			}*/
			vector<float> new_mid_point_twice_distance = find_perpendicular_point_of_intersection(
				lines[i].getMidPointX(),
				lines[i].getMidPointY(),
				lines[i].getX1(),
				lines[i].getY1(),
				lines[i].getX2(),
				lines[i].getY2(),
				perp_dist_bw_midpoints*2,
				false);

			float offset_right = get_euclidean_distance_between_two_points(lines[i].getMidPointX(),
				lines[i].getMidPointY(),
				lines[i].getX2(),
				lines[i].getY2());

			vector<float> bottom_right_corner = find_perpendicular_point_of_intersection(
				new_mid_point[0],
				new_mid_point[1],
				lines[i].getMidPointX(),
				lines[i].getMidPointY(),
				new_mid_point_twice_distance[0],
				new_mid_point_twice_distance[1],
				offset_right,
				false);

			vector<float> bottom_left_corner = find_perpendicular_point_of_intersection(
				new_mid_point[0],
				new_mid_point[1],
				lines[i].getMidPointX(),
				lines[i].getMidPointY(),
				new_mid_point_twice_distance[0],
				new_mid_point_twice_distance[1],
				offset_right,
				true);




			//vector<float> new_bottom_left = find_perpendicular_point_of_intersection(
			//	lines[i].getX1(),
			//	lines[i].getY1(), 
			//	lines[j].getMidPointX(),
			//	lines[j].getMidPointY(),
			//	lines[i].getSlope(),
			//	lines[j].getSlope());


			//vector<float> new_bottom_right = find_perpendicular_point_of_intersection(
			//	lines[i].getX2(),
			//	lines[i].getY2(),
			//	lines[j].getMidPointX(),
			//	lines[j].getMidPointY(),
			//	lines[i].getSlope(),
			//	lines[j].getSlope());

			//int rectangleslength = get_euclidean_distance_between_two_points(lines[i].getX1(), lines[i].getY1(), lines[i].getX2(), lines[i].getY2());
			//int rectangleslengthb = get_euclidean_distance_between_two_points(new_bottom_left[0], new_bottom_left[1], new_bottom_right[0], new_bottom_right[1]);

			//if (rectangleslength < 11 && rectangleslength >16)
			//{
			//	continue;
			//}
			//ParkingSpots *spot = new ParkingSpots(lines[i].getX1(),
			//	lines[i].getY1(),
			//	lines[i].getX2(),
			//	lines[i].getY2(),
			//	new_bottom_left[0],
			//	new_bottom_left[1],
			//	new_bottom_right[0],
			//	new_bottom_right[1],
			//	non_perp_dist_bw_midpoints,			// width
			//	rectlength,							// length
			//	aspectratio,
			//	lines[i].getMidPointX(),
			//	lines[i].getMidPointY(),
			//	lines[j].getMidPointX(),
			//	lines[j].getMidPointY());
			//spotsArray.push_back(*spot);

			ParkingSpots *spot = new ParkingSpots(lines[i].getX1(),
				lines[i].getY1(),
				lines[i].getX2(),
				lines[i].getY2(),
				bottom_right_corner[0],
				bottom_right_corner[1],
				bottom_left_corner[0],
				bottom_left_corner[1],
				perp_dist_bw_midpoints,			// width
				rectlength,							// length
				aspectratio,
				lines[i].getMidPointX(),
				lines[i].getMidPointY(),
				lines[j].getMidPointX(),
				lines[j].getMidPointY());
			spotsArray.push_back(*spot);

	//		int linelength = 0;

	//		// checking if the length of the parent is less than the child
	//		if (lines[i].getlength() < lines[j].getlength())
	//		{
	//			// if parent < child

	//			// maxlinelength here is referring to the length of the parent line(based on which all the other
	//			// lines were saved in that range) from the max occuring line category
	//			if (lines[i].getlength() / maxlinelength <= .99 && lines[i].getlength() / maxlinelength >= .50)
	//			{
	//				linelength = maxlinelength;
	//			}
	//			else
	//			{
	//				linelength = lines[i].getlength();
	//			}
	//		}
	//		else
	//			linelength = lines[j].getlength();


	//		// length between the mid points of lines(width)/length of the bigger line(length)
	//		float aspectratio = length / linelength;

	//		checked = false;

	//		float point1x, point1y;
	//		float point2x, point2y;
	//		float point3x, point3y;
	//		float point4x, point4y;

	//		float midpointX, midpointY;
	//		float midpointX2, midpointY2;

	//		//if the aspectratio of rectangle is between the specified range. here 1 means full size rectangle
	//		if (aspectratio >= .45 && aspectratio <= .70)
	//		{


	//			//float width = 0.0;
	//			//if (lines[i].getCeilLength() >= lines[j].getCeilLength())
	//			//{
	//			//	//point 1 of line 1
	//			//	point1x = lines[i].getX1();
	//			//	point1y = lines[i].getY1();

	//			//	// point 2 of line 1
	//			//	point2x = lines[i].getX2();
	//			//	point2y = lines[i].getY2();

	//			//	// midpoint of line 1
	//			//	midpointX = lines[i].getMidPointX();
	//			//	midpointY = lines[i].getMidPointY();

	//			//	// midpoint of line 2
	//			//	midpointX2 = lines[j].getMidPointX();
	//			//	midpointY2 = lines[j].getMidPointY();

	//			//	width = lines[i].getCeilLength();
	//			}
	//			else
	//			{
	//				//point1x = lines[j].getX1();
	//				//point1y = lines[j].getY1();

	//				//point2x = lines[j].getX2();
	//				//point2y = lines[j].getY2();

	//				//midpointX = lines[j].getMidPointX();
	//				//midpointY = lines[j].getMidPointY();

	//				//midpointX2 = lines[i].getMidPointX();
	//				//midpointY2 = lines[i].getMidPointY();
	//				//width = lines[j].getCeilLength();
	//			}



	//		


	////			// here we've two points and midpoint of line 1 and midpoint of line 2.

	//			// from this data we want to find out the two points of line 2
	//			// we first find the angle of line 1

	//			float deltaY = point2y - point1y;
	//			float deltaX = point2x - point1x;
	//			float angleInDegrees = atan2(deltaY, deltaX) * 180 / 3.14;

	//			float x_dash = length* cos(90 - lines[i].getAngle());
	//			float y_dash = length* sin(90 - lines[i].getAngle());

	//			float point3x = point1x + x_dash;
	//			float point3y = point1y + y_dash;

	//			float point4x = point2x + x_dash;
	//			float point4y = point2y + y_dash;


	//			ParkingSpots *spot = new ParkingSpots(point1x, point1y, point2x, point2y, point3x, point3y, point4x, point4y, width, length);
	//			spotsArray.push_back(*spot);

				if (saved == false)
				{
					MidPointRelations relation = *new MidPointRelations(lines[i], perp_dist_bw_midpoints, 1);
					relation.Add(lines[j]);
					midpointrelations.push_back(relation);
					saved = true;
				}
				else
				{
					midpointrelations.at(0).Add(lines[i]);
					midpointrelations.at(0).Add(lines[j]);
				}
				checked = true;
				break;

			}


		}
	


	//imshow("Drawing", drawing);
}

bool ParkingSpaceDetection:: CheckIfLineAreSame(float x1, float x2, float y1, float y2)
{
	bool result = false;
	if (x1 == x2 && y1 == y2)
	{
		result = true;
	}
	return result;
}

float ParkingSpaceDetection:: GetLineLength(MyLines line1, MyLines line2)
{
	float linelength;
	if (line1.getlength() < line2.getlength())
	{
		if (line1.getlength() / maxlinelength <= .99 && line1.getlength() / maxlinelength >= .50)
		{
			linelength = maxlinelength;
		}
		else
		{
			linelength = line1.getlength();
		}
	}
	else
		linelength = line2.getlength();

	return linelength;
}

void ParkingSpaceDetection:: UpdatePoints(cv::Point p1, cv::Point p2, cv::Point &p3, cv::Point &p4)
{
	float num1x =(p3.x - p1.x);
	float num1y =(p3.y - p1.y);

	float num2x =(p4.x - p1.x);
	float num2y =(p4.y - p1.y);

	// Adjust point of Line2 ;
	float l1 = sqrt(pow(num1x, 2) + pow(num1y, 2));
	float l2 = sqrt(pow(num2x, 2) + pow(num2y, 2));
	if (l1 > l2)
	{
		cv::Point newPoint = p4;
		p4 = p3;
		p3 = newPoint;
	}
}

void ParkingSpaceDetection:: FindPerpendicularPoints(cv::Point p1, cv::Point &p2, int rectLength, int rectWidth)
{
	float pi = 3.14;
	double deltaY = p2.y - p1.y;
	double deltaX = p2.x - p1.x;
	double angleInRad = atan2(deltaY, deltaX);
	float slope = deltaY / deltaX;

	if (angleInRad != pi / 2)
	{
		float xNew = 0;
		float yNew = 0;

		if (slope < 0)
		{
			xNew = p2.x - rectLength * cos(angleInRad);
			yNew = p2.y - rectLength * sin(angleInRad);
		}
		else
		{
			xNew = p2.x + rectLength * cos(angleInRad);
			yNew = p2.y + rectLength * sin(angleInRad);
		}
		p2.x = xNew;
		p2.y = yNew;
	}

}

void ParkingSpaceDetection:: CalculateRectanglePoints(MyLines line1, MyLines line2, int rectLength, int rectWidth)
{
	cv::Point p1 = cv::Point(line1.getX1(), line1.getY1());
	cv::Point p2 = cv::Point(line1.getX2(), line1.getY2());
	cv::Point p3 = cv::Point(line2.getX1(), line2.getY1());
	cv::Point p4 = cv::Point(line2.getX2(), line2.getY2());

	UpdatePoints(p1, p2, p3, p4);
	FindPerpendicularPoints(p1, p3, rectLength, rectWidth);
	FindPerpendicularPoints(p2, p4, rectLength, rectWidth);
	//Return Update points of Line2 to save the Rectangle Coordinates
	ParkingSpots *spot = new ParkingSpots(p1.x, p1.y, p2.x, p2.y, p3.x, p3.y, p4.x, p4.y);
	spotsArray.push_back(*spot);
}

int ParkingSpaceDetection:: FindMostOccuringLength()
{
	vector<RectDetail> rectDetail;

	for (int i = 0; i < spotsArray.size(); i++)
	{
		if (rectDetail.size() == 0)
		{
			rectDetail.push_back(RectDetail(spotsArray[i].getRectHeight(), true));
		}
		for (int j = 0; j < rectDetail.size(); j++)
		{
			if (rectDetail[j].getLength() == ceil(spotsArray[i].getRectHeight()))
			{ 
				rectDetail[j].AddLengthCount();

			}
			else
			{
				rectDetail.push_back(RectDetail(spotsArray[i].getRectHeight(), true));
			}
		}

	}

	int count = 0;
	int mostOccuringLength = 0;
	for (int x = 0; x < rectDetail.size(); x++)
	{
		if (count < rectDetail[x].getLengthCount())
		{
			count = rectDetail[x].getLengthCount();
			mostOccuringLength = rectDetail[x].getLength();
		}
	}

	return mostOccuringLength;
}

int ParkingSpaceDetection:: FindMostOccuringWidth()
{
	vector<RectDetail> rectDetail;

	for (int i = 0; i < spotsArray.size(); i++)
	{
		if (rectDetail.size() == 0)
		{
			rectDetail.push_back(RectDetail(spotsArray[i].getRectWidth()));
		}
		for (int j = 0; j < rectDetail.size(); j++)
		{
			if (rectDetail[j].getWidth() == ceil(spotsArray[i].getRectWidth()))
			{
				rectDetail[j].AddWidthCount();

			}
			else
			{
				rectDetail.push_back(RectDetail(spotsArray[i].getRectWidth()));
			}
		}

	}

	int count = 0;
	int mostOccuringWidth = 0;
	for (int x = 0; x < rectDetail.size(); x++)
	{
		if (count < rectDetail[x].getWidthCount())
		{
			count = rectDetail[x].getWidthCount();
			mostOccuringWidth = rectDetail[x].getWidth();
		}
	}

	return mostOccuringWidth;
}

void ParkingSpaceDetection:: FindAllRectangles(int index)
{
	int mostOccuringLength = FindMostOccuringLength();
	int mostOccuringWidth = FindMostOccuringWidth();

	vector<MyLines> lines = mylinerelations[index].getLines();
	for (int i = 0; i < lines.size(); i++)
	{
		for (int j = 0; j < lines.size(); j++)
		{
			// if the line is same, ignore
			if (CheckIfLineAreSame(lines.at(i).getMidPointX(), lines.at(j).getMidPointX(), lines.at(i).getMidPointY(), lines.at(j).getMidPointY()))
			{
				continue;
			}

			float length = getDistanceBetweenTwoPoints(lines[i].getMidPointX(), lines[i].getMidPointY()
				, lines[j].getMidPointX(), lines[j].getMidPointY(),
				lines[i].getAngle());

			int linelength = GetLineLength(lines[i], lines[j]);
			float aspectratio = length / linelength;

			if (aspectratio >= .45 && aspectratio <= .70)
			{
				float width = 0.0;
				if (lines[i].getCeilLength() > lines[j].getCeilLength())
				{
					CalculateRectanglePoints(lines[i], lines[j], mostOccuringLength, mostOccuringWidth);

				}
				else if (lines[j].getCeilLength() > lines[i].getCeilLength())
				{
					CalculateRectanglePoints(lines[j], lines[i], mostOccuringLength, mostOccuringWidth);
				}
			}
		}
	}
}

cv::Mat ParkingSpaceDetection:: ApplyLineSegmentation(cv::Mat grayImage, cv::Mat srcImage)
{
#if 1
	cv::Ptr<cv::LineSegmentDetector> ls = createLineSegmentDetector(cv::LSD_REFINE_STD);
#else
	Ptr<LineSegmentDetector> ls = createLineSegmentDetector(LSD_REFINE_NONE);
#endif
	vector<cv::Vec4f> lines_std;
	ls->detect(grayImage, lines_std);

	cv::Mat drawnLines(grayImage);
	ls->drawSegments(drawnLines, lines_std);

	return drawnLines;
}

cv::Mat ParkingSpaceDetection:: GetImage(string path)
{
	cv::Mat src = cv::imread(path);
	return src;
}

void destroy_vector(std::vector<cv::Vec4f> &v)
{
   v.clear();
}

void ParkingSpaceDetection:: LoadHough(string path)
{
	int MAX_KERNEL_LENGTH = 31;
	cv::Mat blurImg;
	cv::Mat src = GetImage(path);
	cv::Mat src_gray;
	//bilateralFilter(src, blurImg, MAX_KERNEL_LENGTH, MAX_KERNEL_LENGTH * 2, MAX_KERNEL_LENGTH / 2);

	// bilateral filter applied to image
	/*cv::imshow("Apply Bilateral", blurImg);*/
	cvtColor(src, src_gray, CV_RGB2GRAY);
	/*cv::imshow("Gray Image",src_gray);*/
	cv::Ptr<cv::LineSegmentDetector> ls = createLineSegmentDetector(cv::LSD_REFINE_ADV);

//#if 1
//	cv::Ptr<cv::LineSegmentDetector> ls = createLineSegmentDetector(cv::LSD_REFINE_STD);
//#else
//	Ptr<LineSegmentDetector> ls = createLineSegmentDetector(LSD_REFINE_NONE);
//#endif
	vector<cv::Vec4f> lines;

	// Detect the lines
   ls->detect(src_gray, lines);

   //destroy_vector(lines);
//
//	//cv::Mat drawnLines(src_gray);
//	cv::Mat drawnLines2(src_gray);
//	cv::Mat drawnLines3(src_gray);
//
//	// red line segments detected.
//	//ls->drawSegments(drawnLines, lines);
//	//imshow("Standard refinement", drawnLines);
//
//	//CategorizeByAngle(lines);
//
//	vector<int> indexes;
//	int index = 0;
//	int max = 0;
//
	// loops on the class array that has lines w.r.t an angle and find the class index
	// which has max lines with the same angle. Return all the lines for that category ( based on angle )
//	indexes = GetMaxAngleIndex(mylinerelations);
//	int ii = 0;
//	for (int z = 0; z < indexes.size(); z++)
//	{
//
//		vector<cv::Vec4f> lines3;
//		vector<MyLines> lineeee = mylinerelations[indexes[z]].getLines();
//		cv::Vec4f *ll;
//			for (int i = 0; i < lineeee.size(); i++)
//			{
//
//				ll = new cv::Vec4f(lineeee[i].getX1(), lineeee[i].getY1(), lineeee[i].getX2(), lineeee[i].getY2());
//				lines3.push_back(*ll);
//				ii++;
//			}
//
//		ls->drawSegments(drawnLines3, lines3);
//
//		//We know all the lines. We find the midpoints of all the lines, consider the distances between them
//		//and take the range mode which will give us the max number of values.
//		CategorizeByLength(indexes[z]);
//
//		// we find the category(created in terms of lengths in CategorizeByLength function)
//		// in which we've max number of lines.
//
//		// what we've done here is that we took one line as parent and based on it's distance we compared
//		// all the other lines and added them to the same section if those lines have same distance as the parent
//		// otherwise we take the line and create a new category or a new parent.
//		max = 0;
//		int index2 = 0;
//		for (int j = 0; j < linesbylength.size(); j++)
//		{
//			if (linesbylength[j].getLines().size() > max)
//			{
//				max = linesbylength[j].getLines().size();
//				index2 = j;
//			}
//		}
//
//		// find the lengt of the parent line for this category
//		maxlinelength = linesbylength[index2].getlength();
//
//#pragma region Muazzam Code
//		FindMidPointDistances(indexes[z]);
//#pragma endregion
//
//		//vector<tempObject>().swap(mylines);
//
//
//		max = 0;
//		for (int i = 0; i < midpointrelations.size(); i++)
//		{
//			if (midpointrelations[i].getLines().size()>max)
//			{
//				max = midpointrelations[i].getLines().size();
//				index = i;
//			}
//		}
//		vector<cv::Vec4f> lines2;
//		vector<MyLines> lineee = midpointrelations[index].getLines();
//		for (int i = 0; i < lineee.size(); i++)
//		{
//			ll = new cv::Vec4f(lineee[i].getX1(), lineee[i].getY1(), lineee[i].getX2(), lineee[i].getY2());
//			lines2.push_back(*ll);
//		}
//
//
//
//		ls->drawSegments(drawnLines2, lines2);
//		cv::Scalar color;
//		for (int i = 0; i < spotsArray.size(); i++)
//		{
//			color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
//			line(drawnLines2, cv::Point(spotsArray[i].getPoint1X(), spotsArray[i].getPoint1Y()), cv::Point(spotsArray[i].getPoint2X(), spotsArray[i].getPoint2Y()), color, 2, 8, 0);
//			line(drawnLines2, cv::Point(spotsArray[i].getPoint1X(), spotsArray[i].getPoint1Y()), cv::Point(spotsArray[i].getPoint3X(), spotsArray[i].getPoint3Y()), color, 2, 8, 0);
//			line(drawnLines2, cv::Point(spotsArray[i].getPoint2X(), spotsArray[i].getPoint2Y()), cv::Point(spotsArray[i].getPoint4X(), spotsArray[i].getPoint4Y()), color, 2, 8, 0);
//			line(drawnLines2, cv::Point(spotsArray[i].getPoint3X(), spotsArray[i].getPoint3Y()), cv::Point(spotsArray[i].getPoint4X(), spotsArray[i].getPoint4Y()), color, 2, 8, 0);
//		}
//		
//		//imshow("Final Touch", drawnLines2);
//		
//	}

	//return drawnLines2;

}

void ParkingSpaceDetection:: DrawRectangle(cv::Mat &drawing, float x1, float y1, float x2, float y2)
{
	cv::Rect rec = cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2));
	cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
	rectangle(drawing, rec, color, 2, 8, 0);
}

vector<int> ParkingSpaceDetection:: GetMaxAngleIndex(vector<LineRelationsAngle> mylinerelations)
{
	// this function finds out which are the maximum number lines with the same angle.

	vector<int> index;

	int maxVal = 0;
	for (int i = 0; i < mylinerelations.size(); i++)
	{
		vector<MyLines> myl = mylinerelations[i].getLines();
		// looping all the lines for that angle
		if (myl.size()>maxVal)
		{
			maxVal = myl.size();
			index.push_back(i);
		}
	}

	// returns the index for mylinerelations which contains the max number of lines with same angle.
	return index;
}

std::vector<ParkingSpots> ParkingSpaceDetection:: main()
{
	string path = "..\\Data\\2d-images\\georgeskoehlerallee-manual1.png";
	LoadHough(path);
	
	 return spotsArray;
/*
	line(drawnLines2, cv::Point(spotsArray[i].getPoint1X(), spotsArray[i].getPoint1Y()), cv::Point(spotsArray[i].getPoint2X(), spotsArray[i].getPoint2Y()), color, 2, 8, 0);
			line(drawnLines2, cv::Point(spotsArray[i].getPoint1X(), spotsArray[i].getPoint1Y()), cv::Point(spotsArray[i].getPoint3X(), spotsArray[i].getPoint3Y()), color, 2, 8, 0);
			line(drawnLines2, cv::Point(spotsArray[i].getPoint2X(), spotsArray[i].getPoint2Y()), cv::Point(spotsArray[i].getPoint4X(), spotsArray[i].getPoint4Y()), color, 2, 8, 0);
			line(drawnLines2, cv::Point(spotsArray[i].getPoint3X(), spotsArray[i].getPoint3Y()), cv::Point(spotsArray[i].getPoint4X(), spotsArray[i].getPoint4Y()), color, 2, 8, 0);
*/
}

#pragma endregion


