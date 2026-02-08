/*
 * p2.cpp
 *
 *  Created on: 03/05/2019
 *      Author: Manuel Lorente
 */

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
	if (argc != 2)
	{
		cout << " Usage: p2.exe ImageToLoadAndDisplay" << endl;
		return -1;
	}

	Mat image;
	image = imread(argv[1], IMREAD_COLOR);						// Read the file

	if (!image.data)											// Check for invalid input
	{
		cout << "Could not open or find the image" << std::endl;
		return -1;
	}

	Size img_size = image.size();
	int img_channels = image.channels();
	int img_depth = image.depth();

	// Show image information
	printf("Image size: %dx%d \t # of channels: %d \t Depth: %d\n", img_size.width,img_size.height, img_channels, img_depth);
	
	Mat image_mask(img_size.height, img_size.width, img_depth); // Synthetic mask image
	randn(image_mask, Scalar(0, 0, 0), Scalar(256, 256, 256));

	Mat image_new;
	image.copyTo(image_new, image_mask);						// Apply mask to image

	namedWindow("Original image", WINDOW_NORMAL);				// Create a window for display.
	imshow("Original image", image);							// Show our image inside it.
	
	namedWindow("Mask image", WINDOW_NORMAL);					// Create a window for display.
	imshow("Mask image", image_mask);							// Show our image inside it.

	namedWindow("Processed image", WINDOW_NORMAL);				// Create a window for display.
	imshow("Processed image", image_new);						// Show our image inside it.
	
	// Formula to pixel direct access --> image.data[width*k*i + k*j + n] where n is number of channels, k the channel and i/j are row/col

	// Region to process
	int row_init = 100;
	int row_end = 300;
	int col_init = 100;
	int col_end = 600;

	// To invert the image
	for (int i = row_init; i < row_end; i++)
		for (int j = col_init; j < col_end; j++)
			for (int k = 0; k < img_channels; k++)
				image.data[i * img_size.width * img_channels + j * img_channels + k] = 255 - image.data[i * img_size.width * img_channels + j * img_channels + k];

	namedWindow("Inverted image", WINDOW_NORMAL);					// Create a window for display.
	imshow("Inverted image", image);								// Show our image inside it.

	waitKey(0);														// Wait for a keystroke in the window
	return 0;
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu
