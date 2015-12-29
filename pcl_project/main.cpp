#pragma region Standard Library
#include "Build/stdafx.h"
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>
#include "linear.h"
#include "Build/ParkingSpaceDetection.h"
#pragma endregion

#pragma region Point cloud library
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/histogram_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/common/common.h>
#include <pcl/common/angles.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <pcl/range_image/range_image.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/console/parse.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include "region_growing/region_growing.h"
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

#include "Build/Constant.h"
#include "Build/ReadWrite.h"
#include "Build/ParkingSpaceDetection.h"

#pragma endregion

#pragma region Namespace
//using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;	

#pragma endregion

#pragma region Namespace
//using namespace cv;
using namespace std;

#pragma endregion

#pragma region Public Variables
#define PI 3.14159265
int user_data;
pcl::PointCloud<pcl::PointXYZI>::Ptr cloudWithoutPlane;
#pragma endregion

#pragma region lib linear parameters

struct feature_node *x_space;
struct parameter param;
struct problem prob;
struct model* model_;
int flag_cross_validation;
int flag_find_C;
int flag_C_specified;
int flag_solver_specified;
int nr_fold;
double bias;
struct feature_node *x;
int max_nr_attr = 64;

int flag_predict_probability=0;

static int (*info)(const char *fmt,...) = &printf;
int print_null(const char *s,...) {return 0;}

#define Malloc(type,n) (type *)malloc((n)*sizeof(type))
#define INF HUGE_VAL

#pragma endregion

#pragma region Cloud Viewer Event

void viewerOneOff (pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor (1.0, 0.5, 1.0);
    pcl::PointXYZ o;
    o.x = 1.0;
    o.y = 0;
    o.z = 0;
    viewer.addSphere (o, 0.25, "sphere", 0);
    std::cout << "i only run once" << std::endl;
    
}
    
void viewerPsycho (pcl::visualization::PCLVisualizer& viewer)
{
    static unsigned count = 0;
    std::stringstream ss;
    ss << "Once per viewer loop: " << count++;
    viewer.removeShape ("text", 0);
    viewer.addText (ss.str(), 200, 300, "text", 0);
    
    //FIXME: possible race condition here:
    user_data++;
}

void PickPointEvent (const pcl::visualization::PointPickingEvent &event, void* args)
{
	if (event.getPointIndex () == -1) 
				return; 
	pcl::PointXYZ current_point;
	event.getPoint(current_point.x, current_point.y, current_point.z);
}

#pragma endregion

#pragma region Show Cloud 

void ShowCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr  cloud)
{
	Constant *constant = new Constant();
	pcl::visualization::CloudViewer viewer(constant->cloudWindowName);
	viewer.registerPointPickingCallback  (PickPointEvent, (void*)&viewer); 
	viewer.showCloud(cloud);
    while (!viewer.wasStopped ())
    {
		user_data++;
	}

}

void ShowCloudXYZ(pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud)
{
	Constant *constant = new Constant();
	pcl::visualization::CloudViewer viewer(constant->cloudWindowName);
	viewer.showCloud(cloud);
	viewer.registerPointPickingCallback  (PickPointEvent, (void*)&viewer);
    while (!viewer.wasStopped ())
    {
		user_data++;
	}

}

void ShowCloudRGB( pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud )
{
	Constant *constant = new Constant();
	pcl::visualization::CloudViewer viewer(constant->cloudWindowName);
	viewer.showCloud(cloud);
    while (!viewer.wasStopped ())
    {
		user_data++;
	}
}

#pragma endregion

#pragma region Final Cloud

pcl::PointCloud<pcl::PointXYZRGB>::Ptr GetFinalCloud(vector<int> clusterLabel,vector<int> clusterLabel2,pcl::PointCloud<pcl::PointXYZI>::Ptr clustloud,int index )
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr finalCloud(new pcl::PointCloud<pcl::PointXYZRGB>); 
	for (int i = 0 ; i < clustloud->size() ; i++)
	{

		pcl::PointXYZRGB newPoint;
		newPoint.x = clustloud->points.at(i).x;
		newPoint.y = clustloud->points.at(i).y;
		newPoint.z = clustloud->points.at(i).z;

		if(clusterLabel.at(index) >= 0 && clusterLabel2.at(index) >= 0)
		{
			newPoint.r = 255;
		}
		else //if(clusterLabel.at(index) <0 && clusterLabel2.at(index) < 0)
		{
			newPoint.g = 255;
		}

		finalCloud->points.push_back(newPoint);
		
	}
	finalCloud->height = 1;
	finalCloud->width = clustloud->width;
	return finalCloud;
}

#pragma endregion

pcl::PointCloud<pcl::PointXYZI>::Ptr NormalizeIntensityOfPCD(pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud)
{
	int max_intensity = -1;
	int min_index = 0;
	int max_index = 0;
	double distance_from_origin = 0;

	float threshold = 0;

	// find point closest to x=0,y=0,z=0
	for(int i = 0 ; i < pointCloud->size(); i++)
	{
		if(pointCloud->points.at(i).intensity > max_intensity)
			max_intensity = pointCloud->points.at(i).intensity;
	}
	///threshold = 0.5;
	threshold = max_intensity / 2;
	for(int i = 0 ; i < pointCloud->points.size(); i++)
	{
		
		if(  threshold >= pointCloud->points.at(i).intensity)
		{
			//pointCloud->points.at(i).intensity = max_intensity;

			pointCloud->points.at(i).intensity = 2;
		}
		//else if( ( max_intensity - 2)> pointCloud->points.at(i).intensity)
		//{
		//	//pointCloud->points.at(i).intensity = max_intensity;

		//	pointCloud->points.at(i).intensity = 0;
		//}
		else if ( threshold <  pointCloud->points.at(i).intensity)
		{
			pointCloud->points.at(i).intensity = 0;
		}
		else
		{
			//pointCloud->points.at(i).intensity = pointCloud->points.at(i).intensity/max_intensity;
			pointCloud->points.at(i).intensity = 1;
		}
	}
	return pointCloud;
}

#pragma region Get Data from PCD
pcl::PointCloud<pcl::PointXYZI>::Ptr GetPcdData()
{
	Constant *ConstantClass = new Constant();
	//float _max_int;
	pcl::RangeImagePlanar::Ptr range_image(new pcl::RangeImagePlanar);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZ> pointCloud;
	if (loadPCDFile (ConstantClass->pcd_filename, *cloud))
	{
		print_error ("Unable to load PCD file.\n");
	}
	/*for(int i=0; i<cloud->size(); i++)
    {
		cloud->points.at(i).z = 0;
    }*/
	
	return cloud;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr GetPcdData(string filename)
{
	float _max_int;
	pcl::RangeImagePlanar::Ptr range_image(new pcl::RangeImagePlanar);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZ> pointCloud;
	if (loadPCDFile (filename, *cloud))
	{
		print_error ("Unable to load PCD file.\n");
	}
	/*for(int i=0; i<cloud->size(); i++)
    {
		cloud->points.at(i).z = 0;
    }*/
	
	//ShowCloud(cloud);

	return cloud;
}

#pragma endregion

double get_angle_of_point_from_origin(double x, double y)
{
	double theta = atan2(y,x) * 180/ 3.14;	
	return theta;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr scale_down_point_cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr  cloud,double scalingfactor){
	cloud->points.resize(cloud->width/scalingfactor);

	return cloud;
}

cv::Mat LoadImage()
{
	Constant *ConstantClass = new Constant();
	cv::Mat src = cv::imread(ConstantClass->path);
	return src;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr ApplyRotation(double scalingFactor,double theta,pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
	Eigen::Matrix4f rotation_1 = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f scaling_1 = Eigen::Matrix4f::Identity();
	
	rotation_1 (0,0) = cos (theta);
	rotation_1 (0,1) = -sin(theta);
	rotation_1 (1,0) = sin (theta);
	rotation_1 (1,1) = cos (theta);
	rotation_1 (0,3) = 0;			// translate forward to 0. Since we don't need to translate our point cloud.

	pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZI> ());
	pcl::transformPointCloud (*cloud, *transformed_cloud, rotation_1);

	return transformed_cloud;

}

pcl::PointCloud<pcl::PointXYZI>::Ptr ConvertPngToPcd(cv::Mat srcImage)
{
	//typedef unsigned char uchar;
	cv::Mat src_gray;
	cvtColor(srcImage, src_gray, CV_RGB2GRAY);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ImagePoints (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr ImagePointsIntensity (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointXYZRGB newPoint ;
	pcl::PointXYZI newPoints ;
	for (int x = 0 ; x < srcImage.rows ; x++)
	{
		for (int y = 0; y < srcImage.cols ; y++)
 		{
			cv::Vec3b intensity = srcImage.at<cv::Vec3b>(x,y);
			newPoint.x = x;
			newPoint.y = y;
			newPoint.z = 0;
			newPoint.r = intensity.val[0] ;
			newPoint.g = intensity.val[1] ;
			newPoint.b = intensity.val[2] ;
			ImagePoints->points.push_back(newPoint);
		}
	}
	for (int x = 0 ; x < src_gray.cols  ; x++)
	{
		for (int y = 0; y < src_gray.rows ; y++)
 		{
			cv::Scalar sIntensity = src_gray.at<char>(cv::Point(x, y));
			newPoints.x = x;
			newPoints.y = y;
			newPoints.z = 0;
			newPoints.intensity = sIntensity.val[0];
			ImagePointsIntensity->points.push_back(newPoints);
		}
	}

	return ImagePointsIntensity;

}

void CombineBothTestImages(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPoints, pcl::PointCloud<pcl::PointXYZ>::Ptr ImagePointsIntensity)
{
	float z_value = cloudPoints->points.at(0).z;
	int countz = 0;
	for (int i = 0 ; i < ImagePointsIntensity->size() ; i++)
	{
		if( cloudPoints->points.at(i).z <= 0)
		{
			pcl::PointXYZ points;
			points.x = ImagePointsIntensity->points.at(i).x;
			points.y = ImagePointsIntensity->points.at(i).y;
			points.z = ImagePointsIntensity->points.at(i).z;
			//points.intensity = ImagePointsIntensity->points.at(i).intensity;

			cloudPoints->points.push_back(points);

		}

	}
	ShowCloudXYZ(cloudPoints);
}

void CombineBothImages(pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPoints, pcl::PointCloud<pcl::PointXYZI>::Ptr ImagePointsIntensity)
{
	float z_value = cloudPoints->points.at(0).z;
	int countz = 0;
	for (int i = 0 ; i < ImagePointsIntensity->size() ; i++)
	{
			pcl::PointXYZI points;
			points.x = ImagePointsIntensity->points.at(i).x;
			points.y = ImagePointsIntensity->points.at(i).y;
			points.z = -2;
			points.intensity = ImagePointsIntensity->points.at(i).intensity;

			cloudPoints->points.push_back(points);


	}
	ShowCloud(cloudPoints);
}

pcl::PointCloud<pcl::Normal>::Ptr ComputeNormal(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
	pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
	ne.setInputCloud (cloud);
	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI> ());
	ne.setSearchMethod (tree);

	 pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

	 // Use all neighbors in a sphere of radius 3cm
	  ne.setRadiusSearch (0.03);

	  // Compute the features
	  ne.compute (*cloud_normals);


	  return cloud_normals;
}

#pragma region Mouse Event for Image

//double GetPointAngleFromOrigin(double x, double y)
//{
//	double theta = atan2(y,x) * 180/ PI;	
//	return theta;
//	
//}

//void CallBackFunc(int event, int x, int y, int flags, void* userdata)
//{
//     if  ( event == EVENT_LBUTTONDOWN )
//     {
//		line( srcImage, Point(x, y), Point(x, y), Scalar(0,0,255), 3, CV_AA);
//        cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
//
//		pcl::PointXYZ point;
//		point.x = x;
//		point.y = y;
//		point.z = GetPointAngleFromOrigin(x,y);
//
//		PointImageList.points.push_back(point);
//
//		if(PointCount == 6)
//		{
//			cv::setMouseCallback(ImageWindowName, NULL, NULL);
//		}
//		else 
//		{
//			PointCount++;
//		}
//	 }
//	 imshow(ImageWindowName, srcImage);
//}

//int main() 
//{
//    Point p;
//	srcImage = imread(path);
//	 if ( srcImage.empty() ) 
//     { 
//          cout << "Error loading the image" << endl;
//          return -1; 
//	 }
//	 Show in a window
//	namedWindow(ImageWindowName, CV_WINDOW_AUTOSIZE);
//	imshow(ImageWindowName, srcImage);
//	imshow(ImageWindowName, srcImage);
//
//    pass a pointer to `p` as parameter
//    setMouseCallback(ImageWindowName,CallBackFunc, NULL ); 
//
//     p will update with new mouse-click image coordinates 
//     whenever user clicks on the image window 
//	waitKey(0);
//}

#pragma endregion

pcl::PointCloud<pcl::PointXYZI>::Ptr ApplyTransformation(pcl::PointCloud<pcl::PointXYZI>::Ptr srcCloud, Eigen::Matrix4f transformationMatrix)
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZI> ());
	pcl::transformPointCloud (*srcCloud, *transformed_cloud, transformationMatrix);
	/*ShowCloud(transformed_cloud);*/
	return transformed_cloud;
}

#pragma region Umeyama

pcl::PointCloud<pcl::PointXYZI>::Ptr ApplyUmeyamaTransformation(pcl::PointCloud<pcl::PointXYZI>::Ptr srcCloud, Eigen::Transform<double,3,Eigen::Affine> transformMatrix)
{
	Eigen::Matrix4f transformationMatrix = Eigen::Matrix4f::Identity();
	transformationMatrix (0,0) = transformMatrix (0,0);
	transformationMatrix (0,1) = transformMatrix(0,1);
	transformationMatrix (0,2) = transformMatrix(0,2);


	transformationMatrix (1,0) = transformMatrix(1,0);
	transformationMatrix (1,1) = transformMatrix(1,1);
	transformationMatrix (2,2) = transformMatrix(2,2);

	transformationMatrix (2,0) = transformMatrix(2,0);
	transformationMatrix (2,1) = transformMatrix(2,1);
	transformationMatrix (2,2) = transformMatrix(2,2);

	//For Translation
	transformationMatrix (0,3) = transformMatrix(0,3);
	transformationMatrix (1,3) = transformMatrix(1,3);
	transformationMatrix (2,3) = transformMatrix(2,3);

	transformMatrix.translation();
	pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZI> ());
	pcl::transformPointCloud (*srcCloud, *transformed_cloud, transformationMatrix);
	//ShowCloud(transformed_cloud);
	return transformed_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ApplyTestUmeyamaTransformation(pcl::PointCloud<pcl::PointXYZ>::Ptr srcCloud, Eigen::Transform<double,3,Eigen::Affine> transformMatrix)
{
	Eigen::Matrix4f transformationMatrix = Eigen::Matrix4f::Identity();
	transformationMatrix (0,0) = transformMatrix (0,0);
	transformationMatrix (0,1) = transformMatrix(0,1);
	transformationMatrix (0,2) = transformMatrix(0,2);


	transformationMatrix (1,0) = transformMatrix(1,0);
	transformationMatrix (1,1) = transformMatrix(1,1);
	transformationMatrix (2,2) = transformMatrix(2,2);

	transformationMatrix (2,0) = transformMatrix(2,0);
	transformationMatrix (2,1) = transformMatrix(2,1);
	transformationMatrix (2,2) = transformMatrix(1,1);

	//For Translation
	transformationMatrix (0,3) = transformMatrix(0,3);
	transformationMatrix (1,3) = transformMatrix(1,3);
	transformationMatrix (2,3) = transformMatrix(2,3);

	cout << transformationMatrix;
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::transformPointCloud (*srcCloud, *transformed_cloud, transformationMatrix);
	ShowCloudXYZ(transformed_cloud);
	return transformed_cloud;
}

Eigen::Transform<double,3,Eigen::Affine> GetUmeyamaTransformationMatrix(pcl::PointCloud<pcl::PointXYZ>::Ptr srcCloud,pcl::PointCloud<pcl::PointXYZ>::Ptr targetCloud )
{
	 Eigen::Matrix<double, 3, Eigen::Dynamic> mat_2d (3, 4);
	 Eigen::Matrix<double, 3, Eigen::Dynamic> mat_3d (3, 4);
	 Eigen::Matrix4f rotation_1 = Eigen::Matrix4f::Identity();
	 pcl::PointXYZ point;
	 for (int i=0;i<srcCloud->points.size();i++)
	{
		mat_2d(0,i) = targetCloud->points[i].x;
		mat_2d(1,i) = targetCloud->points[i].y;
		mat_2d(2,i) = targetCloud->points[i].z;

		mat_3d(0,i) = srcCloud->points[i].x;
		mat_3d(1,i) = srcCloud->points[i].y;
		mat_3d(2,i) = srcCloud->points[i].z;

	 }

	Eigen::Transform<double,3,Eigen::Affine> transformationMatrix;
	//cout << Eigen::umeyama(mat_2d,mat_3d,true);
	transformationMatrix = Eigen::umeyama(mat_2d,mat_3d,true);
	return transformationMatrix;
}

#pragma endregion 

#pragma region Points

pcl::PointCloud<pcl::PointXYZ>::Ptr GetSrcPoints()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr srcCloud (new pcl::PointCloud<pcl::PointXYZ>);
	Constant *constant = new Constant();

	pcl::PointXYZ point;
	
	if (constant->PCDNumber == 0)
	{
		point.x = 3471.74976;
		point.y = 1863.53870;
		point.z = 0.0;
		srcCloud->points.push_back(point);

		point.x = 3481.54897;
		point.y = 1862.95776;
		point.z = 0.0;
		srcCloud->points.push_back(point);

		point.x = 3481.36816;
		point.y = 1860.64807;
		point.z = 0.0;
		srcCloud->points.push_back(point);

		point.x = 3471.74976;
		point.y = 1861.25366;
		point.z = 0.0;
		srcCloud->points.push_back(point);
	}
	else if(constant->PCDNumber == 1) // work not done for image 1
	{
		point.x = 7.58129454;
		point.y = 13.6810627;
		point.z = 0.0;
		srcCloud->points.push_back(point);

		point.x =  7.21988234;
		point.y =  9.02739239;
		point.z = 0.0;
		srcCloud->points.push_back(point);

		point.x = 5.07857323;
		point.y = 9.02739239;
		point.z = 0.0;
		srcCloud->points.push_back(point);

		point.x = 5.07857323;
		point.y = 13.6810627;
		point.z = 0.0;
		srcCloud->points.push_back(point);
	}
	else if(constant->PCDNumber == 2)
	{
		point.x = -48.7819023;
		point.y = 10.5346279;
		point.z = -0.214209631;
		srcCloud->points.push_back(point);

		point.x = -38.4412346;
		point.y = 10.5346279;
		point.z = -0.288870901;
		srcCloud->points.push_back(point);

		point.x = -38.4412346;
		point.y = 8.42752743;
		point.z = -0.186465368;
		srcCloud->points.push_back(point);

		point.x = -48.7819023;
		point.y = 8.42752743;
		point.z = -0.176338524;
		srcCloud->points.push_back(point);
	}
	else if(constant->PCDNumber == 3)
	{
		point.x = 2.39341736;
		point.y = 16.7883186;
		point.z = 2.87492300;
		srcCloud->points.push_back(point);

		point.x = 4.48917675;
		point.y = 16.7883186;
		point.z = 3.93414130;
		srcCloud->points.push_back(point);

		point.x = 4.48917675;
		point.y = 11.9112120;
		point.z = 2.82092786;
		srcCloud->points.push_back(point);

		point.x =  2.39341736;
		point.y =  11.9112120;
		point.z = 2.63751006;
		srcCloud->points.push_back(point);
	}
	

	return srcCloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr GetTargetPoints()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr targetCloud (new pcl::PointCloud<pcl::PointXYZ>);
	Constant *constant = new Constant();
	pcl::PointXYZ point;

	if(constant->PCDNumber == 2 || constant->PCDNumber == 0 )
	{
		point.x = 407.0;
		point.y = 458.0;
		point.z = 0.0;
		targetCloud->points.push_back(point);

		point.x = 516.0;
		point.y = 458.0;
		point.z = 0.0;
		targetCloud->points.push_back(point);

		point.x = 516.0;
		point.y = 487.0;
		point.z = 0.0;
		targetCloud->points.push_back(point);

		point.x = 407.0;
		point.y = 487.0;
		point.z = 0.0;
		targetCloud->points.push_back(point);
	}
	else if(constant->PCDNumber == 1)
	{
		point.x = 762.0;
		point.y = 464.0;
		point.z = 0.0;
		targetCloud->points.push_back(point);

		point.x = 817.0;
		point.y = 464.0;
		point.z = 0.0;
		targetCloud->points.push_back(point);

		point.x = 817.0;
		point.y = 493.0;
		point.z = 0.0;
		targetCloud->points.push_back(point);

		point.x = 762.0;
		point.y = 493.0;
		point.z = 0.0;
		targetCloud->points.push_back(point);
	} 
	else if(constant->PCDNumber == 3)
	{
		point.x = 762.0;
		point.y = 464.0;
		point.z = 0.0;
		targetCloud->points.push_back(point);

		point.x = 817.0;
		point.y = 464.0;
		point.z = 0.0;
		targetCloud->points.push_back(point);

		point.x = 817.0;
		point.y = 493.0;
		point.z = 0.0;
		targetCloud->points.push_back(point);

		point.x = 762.0;
		point.y = 493.0;
		point.z = 0.0;
		targetCloud->points.push_back(point);
	}

		

	return targetCloud;
}

#pragma endregion

pcl::PointXYZ FindMaxValues(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
	pcl::PointXYZ point;
	point.x = cloud->points[0].x;
	point.y = cloud->points[0].y;
	point.z = cloud->points[0].z;

	for (size_t i = 1; i < cloud->points.size (); ++i)
	{
		if( cloud->points[i].z > point.z )
			point.z  = cloud->points[i].z;
		if( cloud->points[i].y > point.y)
			point.y = cloud->points[i].y;
		if( cloud->points[i].x > point.x)
			point.x = cloud->points[i].x;
	}

	return point;
}

pcl::PointXYZ FindMinValues(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
	pcl::PointXYZ point;
	point.x = cloud->points[0].x;
	point.y = cloud->points[0].y;
	point.z = cloud->points[0].z;

	for (size_t i = 1; i < cloud->points.size (); ++i)
	{
		if( cloud->points[i].z < point.z )
			point.z  = cloud->points[i].z;
		if( cloud->points[i].y < point.y)
			point.y = cloud->points[i].y;
		if( cloud->points[i].x < point.x)
			point.x = cloud->points[i].x;
	}

	return point;
}

float FindDifference(float num1, float num2)
{
	float val = 0.0;
	val = num2 - num1;
	return val;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr GetPlane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{

	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZI> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (3);

  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);
 if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
  }

  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;


	pcl::PointCloud<pcl::PointXYZI>::Ptr cloudSurface (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointIndices::Ptr fInliers (new pcl::PointIndices);
	
	std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;



  pcl::PointXYZI p;
	  for (size_t i = 0; i < inliers->indices.size (); ++i)
	  {

		  p.x = cloud->points[inliers->indices[i]].x;
		  p.y = cloud->points[inliers->indices[i]].y;
		  p.z = cloud->points[inliers->indices[i]].z;
		  p.intensity = cloud->points[inliers->indices[i]].intensity;

		  cloudSurface->push_back(p);
	  }

	pcl::ExtractIndices<pcl::PointXYZI> extract;
	// Extract the inliers
	
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*cloud_p);

	//ShowCloud(cloudSurface);

	//ShowCloud(cloud_p);
	return cloud_p;

}

bool CreateInputFile(pcl::PointCloud<pcl::PointXYZI>::Ptr srcCloud)
{
	bool result = false;
	float length = 0.0;
	float width = 0.0;
	float height = 0.0;
	float aspectRatio = 0.0;
	float Area = 0.0;
	float aspectlw = 0.0;
	float aspectlh = 0.0;
	float aspectwh = 0.0;

	pcl::PointXYZ maxPoint = FindMaxValues(srcCloud);
	pcl::PointXYZ minPoint = FindMinValues(srcCloud);
	height = FindDifference(minPoint.z,maxPoint.z);
	length = FindDifference(minPoint.y,maxPoint.y);
	width = FindDifference(minPoint.x,maxPoint.x);
	Area = 2 * (length*height +length*width + height*width);
	aspectlw = length/width;
	aspectlh = length/height; 
	aspectwh = width/height;
	//ShowCloud(srcCloud);
	aspectRatio = width / length;
	ReadWrite *rw = new ReadWrite();
	int car = 0;
	rw->GetModelText(car,height,length,width,Area,aspectlw,aspectlh,aspectwh);

	return result;
}

#pragma region Train
void print_null(const char *s) {}

void exit_with_help()
{
	printf(
	"Usage: train [options] training_set_file [model_file]\n"
	"options:\n"
	"-s type : set type of solver (default 1)\n"
	"  for multi-class classification\n"
	"	 0 -- L2-regularized logistic regression (primal)\n"
	"	 1 -- L2-regularized L2-loss support vector classification (dual)\n"
	"	 2 -- L2-regularized L2-loss support vector classification (primal)\n"
	"	 3 -- L2-regularized L1-loss support vector classification (dual)\n"
	"	 4 -- support vector classification by Crammer and Singer\n"
	"	 5 -- L1-regularized L2-loss support vector classification\n"
	"	 6 -- L1-regularized logistic regression\n"
	"	 7 -- L2-regularized logistic regression (dual)\n"
	"  for regression\n"
	"	11 -- L2-regularized L2-loss support vector regression (primal)\n"
	"	12 -- L2-regularized L2-loss support vector regression (dual)\n"
	"	13 -- L2-regularized L1-loss support vector regression (dual)\n"
	"-c cost : set the parameter C (default 1)\n"
	"-p epsilon : set the epsilon in loss function of SVR (default 0.1)\n"
	"-e epsilon : set tolerance of termination criterion\n"
	"	-s 0 and 2\n"
	"		|f'(w)|_2 <= eps*min(pos,neg)/l*|f'(w0)|_2,\n"
	"		where f is the primal function and pos/neg are # of\n"
	"		positive/negative data (default 0.01)\n"
	"	-s 11\n"
	"		|f'(w)|_2 <= eps*|f'(w0)|_2 (default 0.001)\n"
	"	-s 1, 3, 4, and 7\n"
	"		Dual maximal violation <= eps; similar to libsvm (default 0.1)\n"
	"	-s 5 and 6\n"
	"		|f'(w)|_1 <= eps*min(pos,neg)/l*|f'(w0)|_1,\n"
	"		where f is the primal function (default 0.01)\n"
	"	-s 12 and 13\n"
	"		|f'(alpha)|_1 <= eps |f'(alpha0)|,\n"
	"		where f is the dual function (default 0.1)\n"
	"-B bias : if bias >= 0, instance x becomes [x; bias]; if < 0, no bias term added (default -1)\n"
	"-wi weight: weights adjust the parameter C of different classes (see README for details)\n"
	"-v n: n-fold cross validation mode\n"
	"-C : find parameter C (only for -s 0 and 2)\n"
	"-q : quiet mode (no outputs)\n"
	);
	exit(1);
}

void exit_input_error(int line_num)
{
	fprintf(stderr,"Wrong input format at line %d\n", line_num);
	exit(1);
}

static char *pcd_line = NULL;
static int max_line_len;

static char* readline(FILE *input)
{
	int len;

	if(fgets(pcd_line,max_line_len,input) == NULL)
		return NULL;

	while(strrchr(pcd_line,'\n') == NULL)
	{
		max_line_len *= 2;
		pcd_line = (char *) realloc(pcd_line,max_line_len);
		len = (int) strlen(pcd_line);
		if(fgets(pcd_line+len,max_line_len-len,input) == NULL)
			break;
	}
	return pcd_line;
}

void do_find_parameter_C()
{
	double start_C, best_C, best_rate;
	double max_C = 1024;
	if (flag_C_specified)
		start_C = param.C;
	else
		start_C = -1.0;
	printf("Doing parameter search with %d-fold cross validation.\n", nr_fold);
	find_parameter_C(&prob, &param, nr_fold, start_C, max_C, &best_C, &best_rate);
	printf("Best C = %lf  CV accuracy = %g%%\n", best_C, 100.0*best_rate);
}

void do_cross_validation()
{
	int i;
	int total_correct = 0;
	double total_error = 0;
	double sumv = 0, sumy = 0, sumvv = 0, sumyy = 0, sumvy = 0;
	double *target = Malloc(double, prob.l);

	cross_validation(&prob,&param,nr_fold,target);
	if(param.solver_type == L2R_L2LOSS_SVR ||
	   param.solver_type == L2R_L1LOSS_SVR_DUAL ||
	   param.solver_type == L2R_L2LOSS_SVR_DUAL)
	{
		for(i=0;i<prob.l;i++)
		{
			double y = prob.y[i];
			double v = target[i];
			total_error += (v-y)*(v-y);
			sumv += v;
			sumy += y;
			sumvv += v*v;
			sumyy += y*y;
			sumvy += v*y;
		}
		printf("Cross Validation Mean squared error = %g\n",total_error/prob.l);
		printf("Cross Validation Squared correlation coefficient = %g\n",
				((prob.l*sumvy-sumv*sumy)*(prob.l*sumvy-sumv*sumy))/
				((prob.l*sumvv-sumv*sumv)*(prob.l*sumyy-sumy*sumy))
			  );
	}
	else
	{
		for(i=0;i<prob.l;i++)
			if(target[i] == prob.y[i])
				++total_correct;
		printf("Cross Validation Accuracy = %g%%\n",100.0*total_correct/prob.l);
	}

	free(target);
}

void parse_command_line(char *input_file_name, char *model_file_name,bool runAgain = false)
{
	
	void (*print_func)(const char*) = NULL;	// default printing to stdout

#pragma region Default settings
	//param.solver_type = L2R_L2LOSS_SVC_DUAL;
	//param.C = 1;
	//param.eps = INF; // see setting below
	//param.p = 0.1;
	//param.nr_weight = 0;
	//param.weight_label = NULL;
	//param.weight = NULL;
	//param.init_sol = NULL;
	//flag_cross_validation = 0;
	//flag_C_specified = 0;
	//flag_solver_specified = 0;
	//flag_find_C = 0;
	//bias = -1;
#pragma endregion

#pragma region custom settings
	if(runAgain)
	{
		param.solver_type = L2R_L2LOSS_SVC_DUAL; //L1R_LR;
	}
	else
	{
		param.solver_type = L1R_LR; //L1R_LR;
	}
	param.C = 1;
	param.eps = INF; // see setting below
	param.p = 1;
	param.nr_weight = 0;
	param.weight_label = NULL;
	param.weight = NULL;
	param.init_sol = NULL;
	flag_cross_validation = 0;
	flag_C_specified = 0;
	flag_solver_specified = 1;
	flag_find_C = 0;
	bias = -1;
#pragma endregion


	// parse options
	/*for(i=1;i<argc;i++)
	{
		if(argv[i][0] != '-') break;
		if(++i>=argc)
			exit_with_help();
		switch(argv[i-1][1])
		{
			case 's':
				param.solver_type = atoi(argv[i]);
				flag_solver_specified = 1;
				break;

			case 'c':
				param.C = atof(argv[i]);
				flag_C_specified = 1;
				break;

			case 'p':
				param.p = atof(argv[i]);
				break;

			case 'e':
				param.eps = atof(argv[i]);
				break;

			case 'B':
				bias = atof(argv[i]);
				break;

			case 'w':
				++param.nr_weight;
				param.weight_label = (int *) realloc(param.weight_label,sizeof(int)*param.nr_weight);
				param.weight = (double *) realloc(param.weight,sizeof(double)*param.nr_weight);
				param.weight_label[param.nr_weight-1] = atoi(&argv[i-1][2]);
				param.weight[param.nr_weight-1] = atof(argv[i]);
				break;

			case 'v':
				flag_cross_validation = 1;
				nr_fold = atoi(argv[i]);
				if(nr_fold < 2)
				{
					fprintf(stderr,"n-fold cross validation: n must >= 2\n");
					exit_with_help();
				}
				break;

			case 'q':
				print_func = &print_null;
				i--;
				break;

			case 'C':
				flag_find_C = 1;
				i--;
				break;

			default:
				fprintf(stderr,"unknown option: -%c\n", argv[i-1][1]);
				exit_with_help();
				break;
		}
	}*/

	set_print_string_function(print_func);
	//if(input_file_name == NULL)
	//{
	//	// determine filenames
	//	if(i>=argc)
	//		exit_with_help();

	//	strcpy(input_file_name, argv[i]);

	//	if(i<argc-1)
	//		strcpy(model_file_name,argv[i+1]);
	//	else
	//	{
	//		char *p = strrchr(argv[i],'/');
	//		if(p==NULL)
	//			p = argv[i];
	//		else
	//			++p;
	//		sprintf(model_file_name,"%s.model",input_file_name);
	//	}
	//}	
	

	// default solver for parameter selection is L2R_L2LOSS_SVC
	if(flag_find_C)
	{
		if(!flag_cross_validation)
			nr_fold = 5;
		if(!flag_solver_specified)
		{
			fprintf(stderr, "Solver not specified. Using -s 2\n");
			param.solver_type = L2R_L2LOSS_SVC;
		}
		else if(param.solver_type != L2R_LR && param.solver_type != L2R_L2LOSS_SVC)
		{
			fprintf(stderr, "Warm-start parameter search only available for -s 0 and -s 2\n");
			exit_with_help();
		}
	}

	if(param.eps == INF)
	{
		switch(param.solver_type)
		{
			case L2R_LR:
			case L2R_L2LOSS_SVC:
				param.eps = 0.01;
				break;
			case L2R_L2LOSS_SVR:
				param.eps = 0.001;
				break;
			case L2R_L2LOSS_SVC_DUAL:
			case L2R_L1LOSS_SVC_DUAL:
			case MCSVM_CS:
			case L2R_LR_DUAL:
				param.eps = 0.1;
				break;
			case L1R_L2LOSS_SVC:
			case L1R_LR:
				param.eps = 0.01;
				break;
			case L2R_L1LOSS_SVR_DUAL:
			case L2R_L2LOSS_SVR_DUAL:
				param.eps = 0.1;
				break;
		}
	}
}

// read in a problem (in libsvm format)
void read_problem(const char *filename)
{
	int max_index, inst_max_index, i;
	size_t elements, j;
	FILE *fp = fopen(filename,"r");
	char *endptr;
	char *idx, *val, *label;

	if(fp == NULL)
	{
		fprintf(stderr,"can't open input file %s\n",filename);
		exit(1);
	}

	prob.l = 0;
	elements = 0;
	max_line_len = 1024;
	pcd_line = Malloc(char,max_line_len);
	while(readline(fp)!=NULL)
	{
		char *p = strtok(pcd_line," \t"); // label

		// features
		while(1)
		{
			p = strtok(NULL," \t");
			if(p == NULL || *p == '\n') // check '\n' as ' ' may be after the last feature
				break;
			elements++;
		}
		elements++; // for bias term
		prob.l++;
	}
	rewind(fp);

	prob.bias=bias;

	prob.y = Malloc(double,prob.l);
	prob.x = Malloc(struct feature_node *,prob.l);
	x_space = Malloc(struct feature_node,elements+prob.l);

	max_index = 0;
	j=0;
	for(i=0;i<prob.l;i++)
	{
		inst_max_index = 0; // strtol gives 0 if wrong format
		readline(fp);
		prob.x[i] = &x_space[j];
		label = strtok(pcd_line," \t\n");
		if(label == NULL) // empty line
			exit_input_error(i+1);

		prob.y[i] = strtod(label,&endptr);
		if(endptr == label || *endptr != '\0')
			exit_input_error(i+1);

		while(1)
		{
			idx = strtok(NULL,":");
			val = strtok(NULL," \t");

			if(val == NULL)
				break;

			errno = 0;
			x_space[j].index = (int) strtol(idx,&endptr,10);
			if(endptr == idx || errno != 0 || *endptr != '\0' || x_space[j].index <= inst_max_index)
				exit_input_error(i+1);
			else
				inst_max_index = x_space[j].index;

			errno = 0;
			x_space[j].value = strtod(val,&endptr);
			if(endptr == val || errno != 0 || (*endptr != '\0' && !isspace(*endptr)))
				exit_input_error(i+1);

			++j;
		}

		if(inst_max_index > max_index)
			max_index = inst_max_index;

		if(prob.bias >= 0)
			x_space[j++].value = prob.bias;

		x_space[j++].index = -1;
	}

	if(prob.bias >= 0)
	{
		prob.n=max_index+1;
		for(i=1;i<prob.l;i++)
			(prob.x[i]-2)->index = prob.n;
		x_space[j-2].index = prob.n;
	}
	else
		prob.n=max_index;

	fclose(fp);
}

void ExecuteTrain(bool runAgain = false)
{
	Constant *ConstantClass = new Constant();
	char input_file_name[1024];
	char model_file_name[1024];
	const char *error_msg;
	if(runAgain)
	{
		strcpy( input_file_name, ConstantClass->trian_file_path);
		strcpy( model_file_name, ConstantClass->sec_model_file_path);

	}
	else
	{
		strcpy( input_file_name, ConstantClass->trian_file_path);
		strcpy( model_file_name, ConstantClass->model_file_path);
	}

	parse_command_line(input_file_name, model_file_name,runAgain);
	read_problem(input_file_name);
	error_msg = check_parameter(&prob,&param);

	if(error_msg)
	{
		fprintf(stderr,"ERROR: %s\n",error_msg);
		exit(1);
	}

	if (flag_find_C)
	{
		do_find_parameter_C();
	}
	else if(flag_cross_validation)
	{
		do_cross_validation();
	}
	else
	{
		model_=train(&prob, &param);
		if(save_model(model_file_name, model_))
		{
			fprintf(stderr,"can't save model to file %s\n",model_file_name);
			exit(1);
		}
		free_and_destroy_model(&model_);
	}
	destroy_param(&param);
	free(prob.y);
	free(prob.x);
	free(x_space);
	free(pcd_line);

	//return 0;
}

#pragma endregion

#pragma region Predict Data

void do_predict(FILE *input, FILE *output)
{
	int correct = 0;
	int total = 0;
	double error = 0;
	double sump = 0, sumt = 0, sumpp = 0, sumtt = 0, sumpt = 0;

	int nr_class=get_nr_class(model_);
	double *prob_estimates=NULL;
	int j, n;
	int nr_feature=get_nr_feature(model_);
	if(model_->bias>=0)
		n=nr_feature+1;
	else
		n=nr_feature;

	if(flag_predict_probability)
	{
		int *labels;

		if(!check_probability_model(model_))
		{
			fprintf(stderr, "probability output is only supported for logistic regression\n");
			exit(1);
		}

		labels=(int *) malloc(nr_class*sizeof(int));
		get_labels(model_,labels);
		prob_estimates = (double *) malloc(nr_class*sizeof(double));
		fprintf(output,"labels");
		for(j=0;j<nr_class;j++)
			fprintf(output," %d",labels[j]);
		fprintf(output,"\n");
		free(labels);
	}

	max_line_len = 1024;
	pcd_line = (char *)malloc(max_line_len*sizeof(char));
	while(readline(input) != NULL)
	{
		int i = 0;
		double target_label, predict_label;
		char *idx, *val, *label, *endptr;
		int inst_max_index = 0; // strtol gives 0 if wrong format

		label = strtok(pcd_line," \t\n");
		if(label == NULL) // empty line
			exit_input_error(total+1);

		target_label = strtod(label,&endptr);
		if(endptr == label || *endptr != '\0')
			exit_input_error(total+1);

		while(1)
		{
			if(i>=max_nr_attr-2)	// need one more for index = -1
			{
				max_nr_attr *= 2;
				x = (struct feature_node *) realloc(x,max_nr_attr*sizeof(struct feature_node));
			}

			idx = strtok(NULL,":");
			val = strtok(NULL," \t");

			if(val == NULL)
				break;
			errno = 0;
			x[i].index = (int) strtol(idx,&endptr,10);
			if(endptr == idx || errno != 0 || *endptr != '\0' || x[i].index <= inst_max_index)
				exit_input_error(total+1);
			else
				inst_max_index = x[i].index;

			errno = 0;
			x[i].value = strtod(val,&endptr);
			if(endptr == val || errno != 0 || (*endptr != '\0' && !isspace(*endptr)))
				exit_input_error(total+1);

			// feature indices larger than those in training are not used
			if(x[i].index <= nr_feature)
				++i;
		}

		if(model_->bias>=0)
		{
			x[i].index = n;
			x[i].value = model_->bias;
			i++;
		}
		x[i].index = -1;

		if(flag_predict_probability)
		{
			int j;
			predict_label = predict_probability(model_,x,prob_estimates);
			fprintf(output,"%g",predict_label);
			for(j=0;j<model_->nr_class;j++)
				fprintf(output," %g",prob_estimates[j]);
			fprintf(output,"\n");
		}
		else
		{
			predict_label = predict(model_,x);
			fprintf(output,"%g\n",predict_label);
		}

		if(predict_label == target_label)
			++correct;
		error += (predict_label-target_label)*(predict_label-target_label);
		sump += predict_label;
		sumt += target_label;
		sumpp += predict_label*predict_label;
		sumtt += target_label*target_label;
		sumpt += predict_label*target_label;
		++total;
	}
	if(check_regression_model(model_))
	{
		info("Mean squared error = %g (regression)\n",error/total);
		info("Squared correlation coefficient = %g (regression)\n",
			((total*sumpt-sump*sumt)*(total*sumpt-sump*sumt))/
			((total*sumpp-sump*sump)*(total*sumtt-sumt*sumt))
			);
	}
	else
		info("Accuracy = %g%% (%d/%d)\n",(double) correct/total*100,correct,total);
	if(flag_predict_probability)
		free(prob_estimates);
}

void ExecutePredict(bool runAgain = false)
{
	Constant *ConstantClass = new Constant();
	char input_file_name[1024];
	char output_file_name[1024];
	char model_file_name[1024];
	FILE *input, *output;
	int i;
	strcpy( input_file_name, ConstantClass->input_file_path);
	if(runAgain)
	{
		strcpy( output_file_name,ConstantClass->sec_output_file_path);
		strcpy( model_file_name, ConstantClass->sec_model_file_path);
	}
	else
	{
		strcpy( output_file_name,ConstantClass->output_file_path);
		strcpy( model_file_name, ConstantClass->model_file_path);
	}

	//flag_predict_probability = atoi(argv[i]);
	// parse options
	//for(i=1;i<argc;i++)
	//{
	//	if(argv[i][0] != '-') break;
	//	++i;
	//	switch(argv[i-1][1])
	//	{
	//		case 'b':
	//			flag_predict_probability = atoi(argv[i]);
	//			break;
	//		case 'q':
	//			info = &print_null;
	//			i--;
	//			break;
	//		default:
	//			fprintf(stderr,"unknown option: -%c\n", argv[i-1][1]);
	//			exit_with_help();
	//			break;
	//	}
	//}

	input = fopen(input_file_name,"r");
	if(input == NULL)
	{
		fprintf(stderr,"can't open input file %s\n",input_file_name);
		exit(1);
	}

	output = fopen(output_file_name,"w");
	if(output == NULL)
	{
		fprintf(stderr,"can't open output file %s\n",output_file_name);
		exit(1);
	}

	if((model_=load_model(model_file_name))==0)
	{
		fprintf(stderr,"can't open model file %s\n",model_file_name);
		exit(1);
	}

	x = (struct feature_node *) malloc(max_nr_attr*sizeof(struct feature_node));
	do_predict(input, output);
	free_and_destroy_model(&model_);
	free(pcd_line);
	free(x);
	fclose(input);
	fclose(output);
}

#pragma endregion

#pragma region Segmentation

void WritePCDFile(string fileName, int fileCount,pcl::PointCloud<pcl::PointXYZI>::Ptr cluster )
{
	pcl::PCDWriter writer;
	std::stringstream ss;
	ss << fileName << fileCount << ".pcd";
	writer.write <pcl::PointXYZI> (ss.str(), *cluster, false); //*
			
}

void GetCloudFromCluster(std::vector<pcl::PointIndices> clusters,pcl::PointCloud<pcl::PointXYZI>::Ptr srcCloud)
{
	// For every cluster...
	pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_all(new pcl::PointCloud<pcl::PointXYZI>);
	cluster_all->width = 0;
	cluster_all->height = 1;
	cluster_all->is_dense = true;
	string fileName = "cluster_";
	int j = 0;

	

	for (std::vector<pcl::PointIndices>::const_iterator i = clusters.begin(); i != clusters.end(); ++i)
	{

		pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>);
		for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++)
		{ 
			cluster->points.push_back(srcCloud->points[*point]);
			//cluster_all->points.push_back(srcCloud->points[*point]);
		}
		cluster->width = cluster->points.size();
		CreateInputFile(cluster);
		
	}
	// here we have the input file we can predict here 

	ExecutePredict();
	ExecutePredict(true);
	
}

std::vector <pcl::PointIndices> RegionGrowingSegmentation(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::search::Search<pcl::PointXYZI>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZI> > (new pcl::search::KdTree<pcl::PointXYZI>);
    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod (tree);
    normal_estimator.setInputCloud (cloud);
	normal_estimator.setKSearch(50);
    normal_estimator.compute (*normals);

	 pcl::RegionGrowing<pcl::PointXYZI, pcl::Normal> reg;
	 reg.setMinClusterSize(100);
	 reg.setMaxClusterSize (1000);
	 reg.setSearchMethod (tree);
	 reg.setNumberOfNeighbours (10);
	 reg.setInputCloud (cloud);
	 reg.setInputNormals (normals);
	 reg.setSmoothnessThreshold (1.5 * M_PI);
	 reg.setCurvatureThreshold (0.5);
	  
	  std::vector <pcl::PointIndices> clusters;
	  reg.extract (clusters);

	  std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
	  
	  GetCloudFromCluster(clusters,cloud);



	  return clusters;
}

std::vector <pcl::PointIndices> ApplyClustering(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_p)
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZI>);
	kdtree->setInputCloud(cloud_p);
 
	// Euclidean clustering object.
	pcl::EuclideanClusterExtraction<pcl::PointXYZI> clustering;
	clustering.setClusterTolerance(2);
	clustering.setMinClusterSize(100);
	clustering.setMaxClusterSize(1000);
	clustering.setSearchMethod(kdtree);
	clustering.setInputCloud(cloud_p);
	std::vector<pcl::PointIndices> clusters;
	clustering.extract(clusters);
	std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
	// For every cluster...
	pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_all(new pcl::PointCloud<pcl::PointXYZI>);
	cluster_all->width = 0;
	cluster_all->height = 1;
	cluster_all->is_dense = true;

	GetCloudFromCluster(clusters,cloud_p);

	return clusters;
}

#pragma endregion

#pragma region Apply Transformation

pcl::PointCloud<pcl::PointXYZI>::Ptr SelectPCDDataSettings()
{
	Constant *ConstantClass = new Constant();
	pcl::PointCloud<pcl::PointXYZI>::Ptr  cloudPoints;
	if(ConstantClass->PCDNumber == 0)
	{
		// Load 3D PCD data
		cloudPoints = NormalizeIntensityOfPCD(GetPcdData(ConstantClass->pcd_filename));
		//ShowCloud(cloudPoints);
	}
	else if(ConstantClass->PCDNumber == 1)
	{
		// Load 3D PCD data
		cloudPoints = NormalizeIntensityOfPCD(GetPcdData(ConstantClass->pcd_filename1));
		//ShowCloud(cloudPoints);
	}
	else if(ConstantClass->PCDNumber == 2)
	{
		// Load 3D PCD data
		cloudPoints = NormalizeIntensityOfPCD(GetPcdData(ConstantClass->pcd_filename2));
		//ShowCloud(cloudPoints);
	}
	else if(ConstantClass->PCDNumber == 3)
	{
		// Load 3D PCD data
		cloudPoints = NormalizeIntensityOfPCD(GetPcdData(ConstantClass->pcd_filename3));
		//ShowCloud(cloudPoints);
	}
	else if(ConstantClass->PCDNumber == 4)
	{
		// Load 3D PCD data
		cloudPoints = NormalizeIntensityOfPCD(GetPcdData(ConstantClass->pcd_filename4));
		//ShowCloud(cloudPoints);
	}
	return cloudPoints;
}

std::vector <pcl::PointIndices> ApplyTranformationOnRealData ()
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr  cloudPoints = SelectPCDDataSettings();
	
	//ShowCloud(cloudPoints);
	// Load 2D Image Data
	cv::Mat srcImage= LoadImage();
	pcl::PointCloud<pcl::PointXYZI>::Ptr imagePoints = ConvertPngToPcd(srcImage);
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr srcCloud  = GetSrcPoints();
	pcl::PointCloud<pcl::PointXYZ>::Ptr targetCloud = GetTargetPoints();
	Eigen::Transform<double,3,Eigen::Affine> transformMatrix = GetUmeyamaTransformationMatrix(targetCloud,srcCloud);
	pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud = ApplyUmeyamaTransformation(cloudPoints,transformMatrix);
	//ShowCloud(transformed_cloud);
	//CombineBothImages(transformed_cloud,imagePoints);
	cloudWithoutPlane = GetPlane(transformed_cloud);
	//RegionGrowingSegmentation(cloud);
	Constant *constant = new Constant();

	if(constant->trainData)
	{
		ExecuteTrain();
		ExecuteTrain(true);
	}
	std::vector <pcl::PointIndices> clusters;
	if(constant->useRegionGrowing)
	{
		clusters = RegionGrowingSegmentation(cloudWithoutPlane);
	}
	else
	{
		clusters = ApplyClustering(cloudWithoutPlane);
	}
	return clusters;
}

void GetNormalOfPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	  // Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
	ne.setInputCloud (cloud);
	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI> ());
	ne.setSearchMethod (tree);
	// Output datasets
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
	// Use all neighbors in a sphere of radius 3cm
	ne.setRadiusSearch (0.03);
	// Compute the features
	ne.compute (*cloud_normals);

}

void ApplyEuclideanClusterExtraction()
{
	Constant *ConstantClass = new Constant();
	pcl::PCDReader reader;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
	reader.read (ConstantClass->pcd_filename, *cloud);
	std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*	
	ShowCloudXYZ(cloud); 
	// Create the filtering object: downsample the dataset using a leaf size of 1cm
	  pcl::VoxelGrid<pcl::PointXYZ> vg;
	  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	  vg.setInputCloud (cloud);
	  vg.setLeafSize (0.01f, 0.01f, 0.01f);
	  vg.filter (*cloud_filtered);
	  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

	   // Create the segmentation object for the planar model and set all the parameters
	  pcl::SACSegmentation<pcl::PointXYZ> seg;
	  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
	  pcl::PCDWriter writer;
	  seg.setOptimizeCoefficients (true);
	  seg.setModelType (pcl::SACMODEL_PLANE);
	  seg.setMethodType (pcl::SAC_RANSAC);
	  seg.setMaxIterations (100);
	  seg.setDistanceThreshold (0.02);

	 int i=0, nr_points = (int) cloud_filtered->points.size ();
  while (cloud_filtered->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers); 
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_filtered = *cloud_f;
  }

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

  int j = 0;
  float count = 0.0;
  pcl::PointCloud<pcl::PointXYZI>::Ptr regions (new pcl::PointCloud<pcl::PointXYZI>);
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
	{
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
	  pcl::PointXYZI points ;
	  points.x = cloud_filtered->points[*pit].x;
	  points.y = cloud_filtered->points[*pit].y;
	  points.z = cloud_filtered->points[*pit].z;
	  points.intensity = count;

	  regions->points.push_back(points);
	}
	count = count + 0.1;
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
	ShowCloudXYZ(cloud_cluster);
    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    std::stringstream ss;
    ss << "cloud_cluster_" << j << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
    j++;
  }


}

#pragma endregion

void ApplyOnOtherPCD()
{
	Constant *ConstantClass = new Constant();
	pcl::PointCloud<pcl::PointXYZI>::Ptr  cloudPoints = NormalizeIntensityOfPCD(GetPcdData(ConstantClass->pcd_filename2));
	cv::Mat srcImage= LoadImage();
	pcl::PointCloud<pcl::PointXYZI>::Ptr imagePoints = ConvertPngToPcd(srcImage);
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr srcCloud  = GetSrcPoints();
	pcl::PointCloud<pcl::PointXYZ>::Ptr targetCloud = GetTargetPoints();
	Eigen::Transform<double,3,Eigen::Affine> transformMatrix = GetUmeyamaTransformationMatrix(targetCloud,srcCloud);
	pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud = ApplyUmeyamaTransformation(cloudPoints,transformMatrix);
	CombineBothImages(transformed_cloud,imagePoints);
	/*pcl::PointCloud<pcl::PointXYZI>::Ptr  cloudPoints1 = NormalizeIntensityOfPCD(NormalizeIntensityOfPCD(GetPcdData(ConstantClass->pcd_filename1)));
	ShowCloud(cloudPoints1);*/
	//pcl::PointCloud<pcl::PointXYZI>::Ptr  cloudPoints = NormalizeIntensityOfPCD(GetPcdData(ConstantClass->pcd_filename));
	//pcl::PointCloud<pcl::PointXYZI>::Ptr  cloudPoints2 = NormalizeIntensityOfPCD(NormalizeIntensityOfPCD(GetPcdData(ConstantClass->pcd_filename2)));

	//for(int i = 0 ; i < cloudPoints->size(); i++)
	//{
	//	cloudPoints2->push_back(cloudPoints->at(i));
	//}

	//ShowCloud(cloudPoints2);


	//pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = GetPlane(cloudPoints2);
	ShowCloud(cloudPoints);

	/*pcl::PointCloud<pcl::PointXYZI>::Ptr  cloudPoints3 = NormalizeIntensityOfPCD(GetPcdData(ConstantClass->pcd_filename3));
	ShowCloud(cloudPoints3);

	pcl::PointCloud<pcl::PointXYZI>::Ptr  cloudPoints4 = NormalizeIntensityOfPCD(GetPcdData(ConstantClass->pcd_filename4));
	ShowCloud(cloudPoints4);

	pcl::PointCloud<pcl::PointXYZI>::Ptr  cloudPoints = NormalizeIntensityOfPCD(GetPcdData(ConstantClass->pcd_filename));
	ShowCloud(cloudPoints);*/
}

float GetMinPoint(float num1, float num2, float num3, float num4)
{
	float min;
	if(num1 < num2 && num1 < num3 && num1 < num4)
	{
		min = num1;
	}
	else if(num2 < num1 && num2 < num3 && num2 < num4)
	{
		min = num2;
	}
	else if(num3 < num1 && num3 < num2 && num3 < num4)
	{
		min = num3;
	}
	else
	{
		min = num4;
	}
	return min;
}


float GetMaxPoint(float num1, float num2, float num3, float num4)
{
	float max;
	if(num1 > num2 && num1 > num3 && num1 > num4)
	{
		max = num1;
	}
	else if(num2 > num1 && num2 > num3 && num2 > num4)
	{
		max = num2;
	}
	else if(num3 > num1 && num3 > num2 && num3 > num4)
	{
		max = num3;
	}
	else
	{
		max = num4;
	}
	return max;
}

void FindPoints(cv::Point2f &startPoint,cv::Point2f &endPoint,pcl::PointIndices cluster)
{
	int counter = 0;
	float maxX = -10000;
	float maxY = -10000;
	float minX = 10000;
	float minY = 10000;
	while (counter < cluster.indices.size ())
	{
		if( cloudWithoutPlane->points[cluster.indices[counter]].x > maxX )
		{
			maxX = cloudWithoutPlane->points[cluster.indices[counter]].x;
		}

		if( minX > cloudWithoutPlane->points[cluster.indices[counter]].x)
		{
			minX = cloudWithoutPlane->points[cluster.indices[counter]].x;
		}


		if( cloudWithoutPlane->points[cluster.indices[counter]].y > maxY )
		{
			maxY = cloudWithoutPlane->points[cluster.indices[counter]].y;
		}
		if( minY > cloudWithoutPlane->points[cluster.indices[counter]].y)
		{
			minY = cloudWithoutPlane->points[cluster.indices[counter]].y;
		}

		counter++;
	}

	startPoint. x = minX;
	startPoint.y = minY;

	endPoint.x = minX + 40;
	endPoint.y = minY + 20 ;

	/*endPoint.x = maxX;
	endPoint.y = maxY;*/
}

void PlotFinalImage(std::vector<cv::Point2f> spotsArray,std::vector <pcl::PointIndices> clusters,vector<int> ClustLabel,vector<int> ClustLabel2,pcl::PointCloud<pcl::PointXYZI>::Ptr srcCloud,cv::Mat srcImage)
{
	cv::Scalar color;
	std::vector <int> removeIndex;
	for (int x = 0; x < spotsArray.size(); x = x + 4)
	{
		bool spaceAvailable = true;
		float minX =GetMinPoint(spotsArray[x].x,spotsArray[x + 1].x,spotsArray[x + 2].x,spotsArray[x + 3].x);
		float maxX =GetMaxPoint(spotsArray[x].x,spotsArray[x + 1].x,spotsArray[x + 2].x,spotsArray[x + 3].x);
		float minY =GetMinPoint(spotsArray[x].y,spotsArray[x + 1].y,spotsArray[x + 2].y,spotsArray[x + 3].y);
		float maxY =GetMaxPoint(spotsArray[x].y,spotsArray[x + 1].y,spotsArray[x + 2].y,spotsArray[x + 3].y);
		/*color = cv::Scalar(0, 0, 255);*/
		color = cv::Scalar(0, 255, 0);
		for (int i = 0 ; i < clusters.size(); i++)
		{
			if(ClustLabel2.at(i) >= 0)
			{
				int counter = 0;
				while (counter < clusters[i].indices.size ())
				{
					float clusterX =  cloudWithoutPlane->points[clusters[i].indices[counter]].x;
					float clusterY = cloudWithoutPlane->points[clusters[i].indices[counter]].y;
					if(clusterX > minX && clusterX < maxX && clusterY > minY && clusterY < maxY)
					{
						
						color = cv::Scalar(0, 0, 255);
						removeIndex.push_back(i);
						spaceAvailable= false;
						break;
					}
					
					counter++;
				}
			}
			else if(ClustLabel.at(i) >= 0)
			{
				int counter = 0;
				while (counter < clusters[i].indices.size ())
				{
					float clusterX =  cloudWithoutPlane->points[clusters[i].indices[counter]].x;
					float clusterY = cloudWithoutPlane->points[clusters[i].indices[counter]].y;
					if(clusterX > minX && clusterX < maxX && clusterY > minY && clusterY < maxY)
					{
						
						color = cv::Scalar(0, 0, 255);
						removeIndex.push_back(i);
						spaceAvailable= false;
						break;
					}
					
					counter++;
				}
			}
		}

		if(spaceAvailable )
		{
			line(srcImage, spotsArray[x], spotsArray[x + 1], color, 2, 8, 0);
			line(srcImage, spotsArray[x], spotsArray[x + 2] , color, 2, 8, 0);
			line(srcImage,  spotsArray[x + 1],  spotsArray[x + 3], color, 2, 8, 0);
			line(srcImage, spotsArray[x + 2],  spotsArray[x + 3], color, 2, 8, 0);
		}
		else 
		{
			rectangle(srcImage, spotsArray[x], spotsArray[x + 3], color,CV_FILLED, 8, 0);
		}
		
		

	}
	color = cv::Scalar(0, 0, 255);
	
	for( int j = 0 ; j < clusters.size(); j++)
	{
		bool parkingDetected = false ;
		for( int i = 0 ; i < removeIndex.size() ; i++)
		{
			if(removeIndex.at(i) == j)
			{
				parkingDetected = true ;
				break;
			}
			}
			if (parkingDetected)
			{
				continue;
			}

		if(ClustLabel2.at(j) >= 0)
		{
			for (int x = 0 ; x < clusters[j].indices.size(); x++)
			{
				float px = cloudWithoutPlane->points[clusters[j].indices[x]].x;
				float py = cloudWithoutPlane->points[clusters[j].indices[x]].y;
				line(srcImage, cv::Point2f(px,py),cv::Point2f(px,py), color, 2, 8, 0);
			}
			
		}

		else if(ClustLabel.at(j) >= 0)
		{
			for (int x = 0 ; x < clusters[j].indices.size(); x++)
			{
				float px = cloudWithoutPlane->points[clusters[j].indices[x]].x;
				float py = cloudWithoutPlane->points[clusters[j].indices[x]].y;
				line(srcImage, cv::Point2f(px,py),cv::Point2f(px,py), color, 2, 8, 0);
			}
			
		}
	}

	imshow("Final Image", srcImage);
}

int main (int argc, char **argv)
{

	std::vector <pcl::PointIndices> clusters = ApplyTranformationOnRealData();
	vector<int> ClustLabel;
	ReadWrite *rw = new ReadWrite();
	std::vector<cv::Point2f> spotsArray = rw->GetAllRectangles();
	rw->ReadOutputText(ClustLabel, false);
	vector<int> ClustLabel2;
	rw->ReadOutputText(ClustLabel2, true);

	ParkingSpaceDetection *parkingDetection = new ParkingSpaceDetection();
	/*std::vector<ParkingSpots> parkingAreas = parkingDetection->main();*/
	Constant *constant = new Constant();
	cv::Mat src = parkingDetection->GetImage(constant->imgPath);
	delete parkingDetection;
	pcl::PointCloud<pcl::PointXYZI>::Ptr  cloudPoints = SelectPCDDataSettings();
	PlotFinalImage(spotsArray,clusters,ClustLabel,ClustLabel2,cloudPoints,src);
	//ShowCloud(cluster_all);
	//delete parkingDetection;
    cv::waitKey(0);                                          // Wait for a keystroke in the window
    return 0;
}


