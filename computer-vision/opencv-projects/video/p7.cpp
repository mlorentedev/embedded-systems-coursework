/*
 * p7.cpp
 *
 *  Created on: 03/05"019
 *      Author: Manuel Lorente
 */

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
	if (argc != 2)
	{
		cout << " Usage: opencv_basico VideoToLoadAndDisplay" << endl;
		return -1;
	}

	VideoCapture videoSource;

	videoSource.open(argv[1]);
	if (!videoSource.isOpened())
	{
		cout << "Could not open or find the video" << std::endl;
		return -1;
	}

	namedWindow("Original", WINDOW_KEEPRATIO); //resizable window;
	namedWindow("Processed", WINDOW_KEEPRATIO); //resizable window;

	Mat frame;
	Mat frame_aux;
	Mat frame_processed;
	bool proc = false;
	bool grey = false;
	for (;;)
	{
		videoSource >> frame;
		if (frame.empty())
			break;

		if (!proc)
			frame_processed = frame;
		else
		{ 
			if (grey)
				cvtColor(frame, frame_processed, COLOR_BGR2GRAY);
			else
			{ 
				cvtColor(frame, frame_aux, COLOR_BGR2GRAY);
				threshold(frame_aux, frame_processed, 100, 255, THRESH_BINARY);
			}
			frame_processed = frame_processed;
		}

		imshow("Processed", frame_processed);
		imshow("Original", frame);

		char key = (char)waitKey(5);
		
		switch (key) {
			case '1':
				grey = true;
				proc = true;
				break;
			case '2':
				grey = false;
				proc = true;
				break;
			case '3':
				proc = false;
				break;
			case 'q':
				return 0;
			case 'Q':
				return 0;
			case 27:	//escape key
				return 0;
			default:
				break;
		}
	}

	waitKey(0);                                          // Wait for a keystroke in the window
	return 0;
}