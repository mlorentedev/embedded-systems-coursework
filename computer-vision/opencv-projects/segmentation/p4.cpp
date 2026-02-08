/*
 * p4.cpp
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

	// To remove undesired lines
	Mat aux(img_size.height, img_size.width, img_depth);
	dilate(image, aux, Mat());
	medianBlur(aux, aux, 3);
	erode(aux,aux,Mat());

	// Resulted image
	Mat img_result;
	img_result = aux.clone();

	Scalar low_th(20, 20, 20);
	Scalar high_th(20, 20, 20);

	// Sea -> blue
	Point seed_sea(1, 1);
	Scalar value_sea(255, 0, 0); 
	floodFill(img_result, seed_sea, value_sea, 0, low_th, high_th);
	
	// Spain -> red
	Point seed_sp(50, 280);
	Scalar value_sp(0, 0, 255); 
	floodFill(img_result, seed_sp, value_sp, 0, low_th, high_th);

	// Portugal -> green
	Point seed_pt(22, 266);
	Scalar value_pt(0, 255, 0);
	floodFill(img_result, seed_pt, value_pt, 0, low_th, high_th);

	// France -> yellow
	Point seed_fr(105, 225);
	Scalar value_fr(0, 255, 255);
	floodFill(img_result, seed_fr, value_fr, 0, low_th, high_th);

	namedWindow("Result", WINDOW_NORMAL);			// Create a window for display.
	imshow("Result", img_result);                   // Show our image inside it.Mat image2;
	

	waitKey(0);														// Wait for a keystroke in the window
	return 0;
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu
