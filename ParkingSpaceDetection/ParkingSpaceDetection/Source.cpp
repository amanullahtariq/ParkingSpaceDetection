#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

int main(int argc, char** argv)
{
	char* imageName = "C:\\Users\\amanullahtariq\\Downloads\\good-morning-sweetheart.jpg";

	Mat image;
	image = imread(imageName, 1);



	Mat gray_image;
	cvtColor(image, gray_image, CV_BGR2GRAY);

	imwrite("../../images/Gray_Image.jpg", gray_image);

	namedWindow(imageName, CV_WINDOW_AUTOSIZE);
	namedWindow("Gray image", CV_WINDOW_AUTOSIZE);

	imshow(imageName, image);
	imshow("Gray image", gray_image);

	waitKey(0);

	return 0;
}