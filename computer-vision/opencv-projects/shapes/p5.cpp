/*
 * p5.cpp
 *
 *  Created on: 03/05/2019
 *      Author: Manuel Lorente
 */

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
	if (argc != 2)
	{
		cout << " Usage: p3.exe ImageToLoadAndDisplay" << endl;
		return -1;
	}

	Mat image;
	image = imread(argv[1], IMREAD_COLOR);						// Read the file

	if (!image.data)											// Check for invalid input
	{
		cout << "Could not open or find the image" << std::endl;
		return -1;
	}

	// Image format
	Size img_size = image.size();
	int img_channels = image.channels();
	int img_depth = image.depth();

	namedWindow("Original image", WINDOW_NORMAL);					// Create a window for display.
	imshow("Original image", image);								// Show our image inside it.

	// Resulted image
	Mat img_result;
	img_result = image.clone();

	circle(img_result, Point(160, 200), 65, Scalar(255,0,0),3,3);
	putText(img_result, "Circle", Point(215, 259), FONT_HERSHEY_COMPLEX, 1, Scalar(255, 0, 0), 2, 3);

	rectangle(img_result, Point(100, 10), Point(220,140), Scalar(0, 0, 255), 3, 3);
	putText(img_result, "Rectangle", Point(235, 50), FONT_HERSHEY_COMPLEX, 1, Scalar(0, 0, 255), 2, 3);

	ellipse(img_result, Point(375, 130), Size(80, 50), 180, 0, 360, Scalar(0, 255, 0), 3, 3);
	putText(img_result, "Ellipse", Point(350, 215), FONT_HERSHEY_COMPLEX, 1, Scalar(0, 255, 0), 2, 3);

	line(img_result, Point(0,0), Point(479, 336), Scalar(0,0,0), 3, 3);
	putText(img_result, "Line", Point(300, 300), FONT_HERSHEY_COMPLEX, 1, Scalar(0, 0, 0), 2, 3);

	namedWindow("Processed image",WINDOW_NORMAL);			// Create a window for display.
	imshow("Processed image", img_result);                   // Show our image inside it.Mat image2;


	waitKey(0);														// Wait for a keystroke in the window
	return 0;
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu
