#include "ReadWrite.h"
#pragma region Standard Library
#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <iterator>
#pragma endregion
#pragma region Point cloud library
#include <pcl/common/transforms.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/common/common.h>
#include <pcl/common/angles.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#pragma endregion
#include <boost\thread.hpp>
#include <boost\algorithm\string\split.hpp>
#include <boost\algorithm\string\classification.hpp>

#pragma region OpenCV
#include <opencv2/highgui/highgui.hpp>
#pragma endregion

#pragma region Custom Headers
#include "Constant.h"
#pragma endregion

#pragma region Namespace
using namespace std;
#pragma endregion



ReadWrite::ReadWrite(void)
{
}

ReadWrite::~ReadWrite(void)
{
}

#pragma region Read  and Write Text File

void ReadWrite:: GetModelText(bool is_car,float height,float length,float width, float Area, float Aspect1_lw, float Aspect2_lh, float Aspect3_wh)
{
	Constant *ConstantClass = new Constant();
	FILE * pFile;
	pFile = fopen (ConstantClass->input_file_path,"a");
	if (pFile!=NULL)
	{
		string model = "0 1:"+boost::lexical_cast<string>(height)+" 2:"+boost::lexical_cast<string>(length)+" 3:"+boost::lexical_cast<string>(width)+" 4:"+boost::lexical_cast<string>(Area) + " 5:"+boost::lexical_cast<string>(Aspect1_lw)+"\n" ;
		if(is_car)
			{
				//string model = "1 1:"+boost::lexical_cast<string>(height)+" 2:"+boost::lexical_cast<string>(length)+" 3:"+boost::lexical_cast<string>(width)+" 4:"+boost::lexical_cast<string>(Area) + " 5:"+boost::lexical_cast<string>(Aspect1_lw)+"\n" ;
				//string model = "+1 1:"+boost::lexical_cast<string>(height)+" 2:"+boost::lexical_cast<string>(length)+" 3:"+boost::lexical_cast<string>(width)+" 4:"+boost::lexical_cast<string>(Area)+" 5:"+boost::lexical_cast<string>(Aspect1_lw)+" 6:"+boost::lexical_cast<string>(Aspect2_lh)+" 7:"+boost::lexical_cast<string>(Aspect3_wh)+"\n" ;
				const char * c = model.c_str();
				fputs (c,pFile);
				fclose (pFile);
			}
		else if(!is_car)
			{
				//string model = "-1 1:"+boost::lexical_cast<string>(height)+" 2:"+boost::lexical_cast<string>(length)+" 3:"+boost::lexical_cast<string>(width)+" 4:"+boost::lexical_cast<string>(Area) + " 5:"+boost::lexical_cast<string>(Aspect1_lw)+"\n" ;
				//string model = "-1 1:"+boost::lexical_cast<string>(height)+" 2:"+boost::lexical_cast<string>(length)+" 3:"+boost::lexical_cast<string>(width)+" 4:"+boost::lexical_cast<string>(Area)+" 5:"+boost::lexical_cast<string>(Aspect1_lw)+" 6:"+boost::lexical_cast<string>(Aspect2_lh)+" 7:"+boost::lexical_cast<string>(Aspect3_wh)+"\n" ;
				const char * c = model.c_str();
				fputs (c,pFile);
				fclose (pFile);
			}
	}
	else cout << "Unable to open file";
}

void ReadWrite:: ReadOutputText(vector<int> &clust, bool runAgain = false)
{
	Constant *ConstantClass = new Constant();
	char *fileName;
	if(runAgain)
	{
		fileName = ConstantClass->output_file_path;
	}
	else
	{
		fileName = ConstantClass->sec_output_file_path;
	}
	
	string line;
	
	std::ifstream output (fileName);
	if (output.is_open())
	{
		while (getline(output,line))
		{
			int numb;
			istringstream ( line ) >> numb;
			clust.push_back(numb);
		}
		output.close();
	}
	else
	{
		std::cout << "Unable to open file" << std::endl << std::endl;
	}
}


std::vector<cv::Point2f> ReadWrite:: GetAllRectangles()
{
	std::vector<cv::Point2f> spotsArray;
	Constant *ConstantClass = new Constant();
	char *fileName;
	fileName = ConstantClass->spotArrayFile;
	
	string line;
	
	std::ifstream output (fileName);
	if (output.is_open())
	{
		while (getline(output,line))
		{
			 cv::Point2f point;
			 float numb;
			 std::vector<std::string> tokens;
			 boost::split(tokens, line, boost::is_any_of(","));
			 //int i = 0;
			 for (size_t i = 0; i < tokens.size(); i++)
			 {
				  float numb;
			     istringstream (tokens[i] ) >> numb;
				 if( i % 2 == 0)
				 {
					  point.x = numb;
				 }
				 else
				 {
					  point.y = numb;
				 }
				 if( i % 2  == 1)
				 {
					 spotsArray.push_back(point);
				 }
			 }
						
		}
		output.close();
	}
	else
	{
		std::cout << "Unable to open file" << std::endl << std::endl;
	}
	return spotsArray;
}

#pragma endregion
