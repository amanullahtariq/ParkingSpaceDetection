#include "Constant.h"




Constant::Constant(void)
{
	/*char * filePath = "..\\Data\\3d-pointclouds\\";
	char * filename = "parkinglot" ;
	strcpy( pcd_filename, filePath);
	strcat(pcd_filename,filename);*/
	spotArrayFile = "spots.txt";
	imgPath = "..\\Data\\2d-images\\georgeskoehlerallee-manual1.png";
	pcd_filename = "..\\Data\\3d-pointclouds\\parkinglot.pcd";
	pcd_filename1 = "..\\Data\\3d-pointclouds\\parkinglot2015-06-17.pcd";
	pcd_filename2 = "..\\Data\\3d-pointclouds\\parkinglot2015-06-18.pcd";
	pcd_filename3 = "..\\Data\\3d-pointclouds\\parkinglot2015-06-23.pcd";
	pcd_filename4 = "..\\Data\\3d-pointclouds\\parkinglot2015-06-25.pcd";

	path = "..\\Data\\2d-images\\georgeskoehlerallee.png";
	ImageWindowName = "2D Image";
	cloudWindowName = "PCD Cloud Viewer";
	output_file_path = "output.out";
	sec_output_file_path = "output2.out";
	input_file_path = "input.txt";
	useRegionGrowing = true;
	PCDNumber = 0;
	trainData = false;
	if(useRegionGrowing)
	{
		trian_file_path = "train_rg.txt";
		model_file_path = "train_rg.model";
		sec_model_file_path = "train_rg2.model";
	}
	else 
	{
		trian_file_path = "train_eu.txt";
		model_file_path = "train_eu.model";

		sec_model_file_path = "train_eu2.model";


	}
}


Constant::~Constant(void)
{
}

