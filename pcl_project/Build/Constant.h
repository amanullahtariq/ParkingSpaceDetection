#pragma region Standard Library
#include <iostream>
#pragma endregion

#pragma region Point cloud library
#pragma endregion

#pragma region OpenCV

#include <opencv2/highgui/highgui.hpp>

#pragma endregion

/// Contains all the Constants here
#pragma once
class Constant
{

public:
	char * spotArrayFile;
	std::string imgPath;
	char * pcd_filename ;
	std::string pcd_filename1;
	std::string pcd_filename2;
	std::string pcd_filename3;
	std::string pcd_filename4;
	std::string path;
	std::string ImageWindowName;
	std::string cloudWindowName;
	char * trian_file_path;
	char * model_file_path;
	char * output_file_path;
	char * sec_model_file_path;
	char * sec_output_file_path;
	char * input_file_path;
	int user_data;
	bool useRegionGrowing;
	bool trainData;
	int PCDNumber;

public:
	Constant(void);
	~Constant(void);
};

