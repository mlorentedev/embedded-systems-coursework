/*
 * p6.cpp
 *
 *  Created on: 03/05/2019
 *      Author: Manuel Lorente
 */

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
	if (argc != 2)
	{
		cout << " Usage: ejemplo_opencv_mvc ImageToLoadAndProcess" << endl;
		return -1;
	}

	Mat image;
	image = imread(argv[1], IMREAD_COLOR);   // Read the file

	if (!image.data)                              // Check for invalid input
	{
		cout << "Could not open or find the image" << std::endl;
		return -1;
	}

	Mat img_result;
	img_result = image.clone();

	String facedetector_data = "C:/opencv/sources/data/haarcascades/haarcascade_frontalface_alt2.xml";
	CascadeClassifier face_detector;
	if (!face_detector.load(facedetector_data))
	{
		printf("--(!)Error loading face detector\n");
		return -1;
	}

	String eyedetector_data = "C:/opencv/sources/data/haarcascades/haarcascade_eye.xml";
	CascadeClassifier eye_detector;
	if (!eye_detector.load(eyedetector_data))
	{
		printf("--(!)Error loading eye detector\n");
		return -1;
	}

	std::vector<Rect> faces;
	std::vector<Rect> eyes;

	face_detector.detectMultiScale(img_result, faces, 1.1, 3, 0, Size(50, 50), Size(500, 500));

	for (int i = 0; i < faces.size(); i++)
	{
		printf("Detected face %d: %d %d %d %d\n", i, faces.at(i).x, faces.at(i).y, faces.at(i).width, faces.at(i).height);
		rectangle(img_result, Point(faces.at(i).x, faces.at(i).y), Point(faces.at(i).x + faces.at(i).width, faces.at(i).y + faces.at(i).height)
			, Scalar(0, 0, 255), 3, 3);

		Mat img_roi(img_result, Rect(faces.at(i).x, faces.at(i).y, faces.at(i).width, faces.at(i).height));
		eye_detector.detectMultiScale(img_roi, eyes, 1.1, 3, 0, Size(10, 10), Size(80, 80));

		for (int j = 0; j < eyes.size(); j++)
		{
			printf("Detected eye %d: %d %d %d %d\n", j, eyes.at(j).x, eyes.at(j).y, eyes.at(j).width, eyes.at(j).height);
			rectangle(img_roi, Point(eyes.at(j).x, eyes.at(j).y), Point(eyes.at(j).x + eyes.at(j).width, eyes.at(j).y + eyes.at(j).height)
				, Scalar(255, 255, 255), 3, 3);
		}
	}
	
	namedWindow("Original", WINDOW_NORMAL);
	imshow("Original", image);						 // Imagen original.
	namedWindow("Result", WINDOW_NORMAL);
	imshow("Result", img_result);                   // Imagen que tenia la ROI asociada. Puede verse como los cambios en la subimagen 'image_roi' aparecen en la ROI de esta imagen

	waitKey(0);                                          // Wait for a keystroke in the window
	return 0;
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu
