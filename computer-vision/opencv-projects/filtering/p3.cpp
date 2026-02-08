/*
 * p3.cpp
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

	// Smoothing 1/4 - Average filter
	Mat image_avg(img_size.height, img_size.width, img_depth); 
	blur(image, image_avg, Size(4,4));
	namedWindow("Smoothed image - Average filter 4x4", WINDOW_NORMAL);					// Create a window for display.
	imshow("Smoothed image - Average filter 4x4", image_avg);							// Show our image inside it.

	// Smoothing 2/4 - Average filter
	Mat image_median(img_size.height, img_size.width, img_depth);
	medianBlur(image, image_median,5);
	namedWindow("Smoothed image - Median filter by 5", WINDOW_NORMAL);					// Create a window for display.
	imshow("Smoothed image - Median filter by 5", image_median);						// Show our image inside it.

	// Smoothing 3/4 - Gaussian filter
	Mat image_gaussian(img_size.height, img_size.width, img_depth);
	GaussianBlur(image, image_gaussian,Size(3,3),2,2);
	namedWindow("Smoothed image - Gaussian filter 3x3 std=2", WINDOW_NORMAL);					// Create a window for display.
	imshow("Smoothed image - Gaussian filter 3x3 std=2", image_gaussian);						// Show our image inside it.

	// Smoothing 4/4 - Bilateral filter
	Mat image_bilat(img_size.height, img_size.width, img_depth);
	bilateralFilter(image, image_bilat, 5, 150, 150);
	namedWindow("Smoothed image - Bilateral filter 5 150-150", WINDOW_NORMAL);				// Create a window for display.
	imshow("Smoothed image - Bilateral filter 5 150-150", image_bilat);						// Show our image inside it.

	// Binarization and truncation - Need to convert RGB image to grayscale using smoothed img to reduce the noise
	Mat image_bin(img_size.height, img_size.width, img_depth);
	Mat image_trunc(img_size.height, img_size.width, img_depth);
	Mat image_gray(img_size.height, img_size.width, img_depth);
	
	cvtColor(image_bilat, image_gray, COLOR_BGR2GRAY);
	
	threshold(image_gray, image_bin, 100, 255, THRESH_BINARY);
	threshold(image_gray, image_trunc, 100, 255, THRESH_TRUNC);

	namedWindow("Binary image - Threshold = 100DN", WINDOW_NORMAL);					// Create a window for display.
	imshow("Binary image - Threshold = 100DN", image_bin);							// Show our image inside it.
	namedWindow("Binary image - Threshold = 100DN", WINDOW_NORMAL);					// Create a window for display.
	imshow("Trunqued image - Threshold = 100DN", image_trunc);						// Show our image inside it.

	// Border detection - Canny algorithm
	Mat image_canny(img_size.height, img_size.width, img_depth);
	Canny(image_gray, image_canny, 100, 200, 3);

	namedWindow("Border detection ", WINDOW_NORMAL);					// Create a window for display.
	imshow("Border detection ", image_canny);							// Show our image inside it.


	waitKey(0);														// Wait for a keystroke in the window
	return 0;
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu
