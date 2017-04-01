#include <iostream>
#include "Source.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <list>
#include <fstream>
#include <string>
#include <iomanip>

using namespace cv;
using namespace std;

// haris corner

/// Global variables

char* source_window = "Source image";
char* corners_window = "Corners detected";

/// Function header
void cornerHarris_demo(int, void*);
// haris corner

// filtering


/// Global Variables
int DELAY_CAPTION = 1500;
int DELAY_BLUR = 100;
int MAX_KERNEL_LENGTH = 31;

//Mat src; Mat dst;
//char window_name[] = "Filter Demo 1";

/// Function headers
int display_caption(char* caption);
int display_dst(int delay);

// filtering

Mat src, src_gray;
Mat dst, dst2, detected_edges;
int thresh = 100;
int max_thresh = 255;
RNG rng(12345);

int edgeThresh = 1;
int lowThreshold;
int const max_lowThreshold = 100;
int ratio = 3;
int kernel_size = 3;
char* window_name = "Edge Map";



// keeps lines data.
class MyLines {
	int x1, y1;
	int x2, y2;

	float angle;

public:
	MyLines(float x11, float y11, float  x22, float  y22, float anglee){
		x1 = x11;
		x2 = x22;
		y1 = y11;
		y2 = y22;
		angle = anglee;
	}

	float getX1()	{ return x1; }
	float getY1()	{ return y1; }
	float getX2()	{ return x2; }
	float getY2()	{ return y2; }

	float getAngle()	{ return angle; }
};


// keeps all the lines with the same angle together.
class LineRelations {

	vector<MyLines> liness;
	float angle;

public:

	LineRelations(MyLines lin)
	{
		angle = lin.getAngle();
		Add(lin);
	}



	void Add(MyLines l)
	{
		liness.push_back(l);
	}

	float getAngle()	{ return angle; }
	vector<MyLines> getLines()	{ return liness; }
};

std::vector<MyLines> mylines;
std::vector<LineRelations> mylinerelations;

void CheckLineExistance(MyLines lin)
{
	// find if lines same as this lines angle already exist.
	for (int i = 0; i < mylinerelations.size(); i++)
	{
		if (mylinerelations[i].getAngle() == lin.getAngle())
		{
			mylinerelations[i].Add(lin);
			return;
		}
	}

	// Add the line with new angle
	mylinerelations.push_back(*new LineRelations(lin));
}

cv::Point2f computeIntersect(cv::Vec4i a, cv::Vec4i b)				
{
	int x1 = a[0], y1 = a[1], x2 = a[2], y2 = a[3];
	int x3 = b[0], y3 = b[1], x4 = b[2], y4 = b[3];

	if (float d = ((float)(x1 - x2) * (y3 - y4)) - ((y1 - y2) * (x3 - x4)))
	{
		cv::Point2f pt;
		pt.x = ((x1*y2 - y1*x2) * (x3 - x4) - (x1 - x2) * (x3*y4 - y3*x4)) / d;
		pt.y = ((x1*y2 - y1*x2) * (y3 - y4) - (y1 - y2) * (x3*y4 - y3*x4)) / d;
		return pt;
	}
	else
		return cv::Point2f(-1, -1);
}
void sortCorners(std::vector<cv::Point2f>& corners, cv::Point2f center)
{
	std::vector<cv::Point2f> top, bot;

	for (int i = 0; i < corners.size(); i++)
	{
		if (corners[i].y < center.y)
			top.push_back(corners[i]);
		else
			bot.push_back(corners[i]);
	}

	cv::Point2f tl = top[0].x > top[1].x ? top[1] : top[0];
	cv::Point2f tr = top[0].x > top[1].x ? top[0] : top[1];
	cv::Point2f bl = bot[0].x > bot[1].x ? bot[1] : bot[0];
	cv::Point2f br = bot[0].x > bot[1].x ? bot[0] : bot[1];

	corners.clear();
	corners.push_back(tl);
	corners.push_back(tr);
	corners.push_back(br);
	corners.push_back(bl);
}




void loadHough(string path)
{

	Mat src = imread(path, 1);

	Mat dst, cdst;

	//applying filtering.
	bilateralFilter (src, dst2, MAX_KERNEL_LENGTH, MAX_KERNEL_LENGTH * 2, MAX_KERNEL_LENGTH / 2);
	imshow("bilateral", dst2);
	imshow("normal", src);

	Canny(dst2, dst, 50, 200, 3);
	imshow("canny", dst);
	cv::dilate(src, src, cv::Mat(), cv::Point(-1, -1));
	cvtColor(dst, cdst, CV_GRAY2BGR);

#if 0
	vector<Vec2f> lines;
	HoughLines(dst, lines, 1, CV_PI / 180, 70, 30, 10);

	for (size_t i = 0; i < lines.size(); i++)
	{
		float rho = lines[i][0], theta = lines[i][1];
		Point pt1, pt2;
		double a = cos(theta), b = sin(theta);
		double x0 = a*rho, y0 = b*rho;
		pt1.x = cvRound(x0 + 1000 * (-b));
		pt1.y = cvRound(y0 + 1000 * (a));
		pt2.x = cvRound(x0 - 1000 * (-b));
		pt2.y = cvRound(y0 - 1000 * (a));
		line(cdst, pt1, pt2, Scalar(0, 0, 255), 3, CV_AA);
	}
#else
	vector<Vec4i> lines;
	HoughLinesP(dst, lines, 1, CV_PI / 180, 45, 2, 5);
	for (size_t i = 0; i < lines.size(); i++)
	{
		Vec4i l = lines[i];

		cout << "l[0] = " << l[0] << endl;
		cout << "l[1]= " << l[1] << endl;
		cout << "l[2]= " << l[2] << endl;
		cout << "l[3]= " << l[3] << endl;
		float denom = (l[2] - l[0]);
		if (denom == 0)
		{
			continue;
		}// this is so that when we find the angle, we don't divide a number by 0 which gives us runtime exception.
		
		cout << "Found angle = " << ((atan((l[3] - l[1]) / denom)) * 180 / 3.14) << endl;
		std::string::size_type sz;;



		CheckLineExistance(*new MyLines(l[0], l[1], l[2], l[3], (atan2((l[2] - l[0]), (l[3] - l[1])) * 180 / 3.14)));
	
	}

	Point pt = Point(10, 8);


	cout << "Lines Size : " << lines.size() << endl;
	cout << "Same angles : " << mylinerelations.size();
	

	ofstream myfile("example.txt", std::ios_base::app);
	for (int i = 0; i < mylinerelations.size(); i++)
	{

		if (mylinerelations[i].getLines().size() > 6)
		{
			vector<MyLines> l = mylinerelations[i].getLines();

			for (int o = 0; o < l.size(); o++)
			{
				/*if (o == 4)
					break;*/

				if (myfile.is_open())
				{
					myfile << "Line Angle == " << l[o].getAngle() << endl;
					myfile << " no. of lines == " << mylinerelations[i].getLines().size() << endl;
					myfile << "x1  == " << l[o].getX1() << endl;
					myfile << "y1 == " << l[o].getY1() << endl;
					myfile << "x2 == " << l[o].getX2() << endl;
					myfile << "y2 == " << l[o].getY2() << endl;
					myfile << "====================================== " << endl;
				}
				line(cdst, Point(l[o].getX1(), l[o].getY1()), Point(l[o].getX2(), l[o].getY2()), Scalar(0, 0, 255), 2, .01);
			}
		}
	}
	myfile.close();

	// Needed for visualization only - Expanding Lines such that they intersect
	for (int i = 0; i < lines.size(); i++)
	{
		cv::Vec4i v = lines[i];
		lines[i][0] = 0;
		lines[i][1] = ((float)v[1] - v[3]) / (v[0] - v[2]) * -v[0] + v[1];
		lines[i][2] = src.cols;
		lines[i][3] = ((float)v[1] - v[3]) / (v[0] - v[2]) * (src.cols - v[2]) + v[3];
	}

	// find lines that are intersecting with each other.
	std::vector<cv::Point2f> corners;
	for (int i = 0; i < lines.size(); i++)
	{
		for (int j = i + 1; j < lines.size(); j++)
		{
			cv::Point2f pt = computeIntersect(lines[i], lines[j]);
			if (pt.x >= 0 && pt.y >= 0)
				corners.push_back(pt);
		}
	}

	std::vector<cv::Point2f> approx;
	cv::approxPolyDP(cv::Mat(corners), approx, cv::arcLength(cv::Mat(corners), true) * 0.02, true);


	// Get mass center
	cv::Point2f center(0, 0);
	for (int i = 0; i < corners.size(); i++)
		center += corners[i];

	center *= (1. / corners.size());

#endif

	imshow("detected lines", cdst);

	waitKey();

}


int main(int argc, char** argv)
{
	string path = "D:\\repo\\bitbucket\\ParkingSpace\\parkingdetect\\parkingdetect\\images\\test.jpg";
	loadHough(path);
	return 0;

}

Source::Source()
{
}


Source::~Source()
{
}

