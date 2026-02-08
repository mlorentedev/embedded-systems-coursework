/*
 * pfinal.cpp
 *
 *  Created on: 11/05/2019
 *      Author: Manuel Lorente
 * Description: La práctica final de la asignatura consiste
 *				en el diseño e implementación de un sistema
 *				de procesamiento de imagen que garantice el
 *				anonimato de las personas que aparecen en
 *				un vídeo.
 *	   Mejoras: Implementadas las mejoras 1, 2, 3 y 4 
 *
 */

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/video/tracking.hpp>
#include <iostream>

// Modo debug con un video interno
//#define DEBUG

// Valores constantes - Tamaño esperado de cara, difuminado por defec
#define MIN_ROI_SIZE		50		// Tamaño esperado de cara
#define MAX_ROI_SIZE		200		// Tamaño esperado de cara
#define	BLURRING_DEFAULT	50		// Difuminador por defecto

// Maquina de estados para procesamiento
// 1: No hacer nada
// 2: Escribe en el frame texto en rojo
// 3: Difumina toda la imagen
static int status;

// Mejora 2 - Intensidad del difuminado de cara en funcion de la tecla
// 1: 10%
// 2: 20%
// 3: 30%
// 4: 40%
// 5: 50%
// 6: 60%
// 7: 70%
// 8: 80%
// 9: 90%
// 0: 100%
static int blurring;

// Mejora 2 - Si se teclea espacio, no se difuminan las caras
static bool process;

// Mejora 3 - Si se teclea i, muestra info del video
static bool video_info;

using namespace cv;
using namespace std;

// Funciones auxiliares para detección de cara y procesamiento
void faceProcessing(Mat img, std::vector<Rect> roi);
void noFace(Mat img, Size img_size);
bool processingPolling(void);
void updateStatus(void);
void updateInfo(Mat img, int nindex, int nframes, int faces_size, int fps, Size img_size);
int findMaxFace(std::vector<Rect> roi);
void camshiftFcn(std::vector<Rect> roi);

int main(int argc, char** argv)
{
	// Variable global de estado de procesamiento de frame actual
	status = 1;
	// Difuminado por defecto
	blurring = BLURRING_DEFAULT;
	// Procesado ON por defecto
	process = true;
	// Video info OFF por defecto
	video_info = false;

	// Variables auxiliares utilizadas
	int nindex = 0;
	int nfaces = 0;

#ifdef DEBUG
	if (argc != 1)
	{
		cout << " Usage: pfinal.exe" << endl;
		return -1;
	}

	// Intervalo muestreo algoritmo tracking
	int samples = 10;

	// Apertura del video de entrada
	VideoCapture videoSource;
	videoSource.open("C:/Users/Manuel/Dropbox/Universidad/MSEEI/11-VisionArtificial/Trabajo-Final/monchi.avi");
#else
	if (argc != 3)
	{
		cout << " Usage: pfinal.exe video NtrackingSamples" << endl;
		return -1;
	}
	// Apertura del video de entrada
	VideoCapture videoSource;
	videoSource.open(argv[1]);
	int samples = atoi(argv[2]);
#endif // DEBUG
	
	// Formato del video
	double fps = videoSource.get(CAP_PROP_FPS);
	int nframes = videoSource.get(CAP_PROP_FRAME_COUNT);
	int frame_width = videoSource.get(CAP_PROP_FRAME_WIDTH);
	int frame_height = videoSource.get(CAP_PROP_FRAME_HEIGHT);

	// Mejora 4 - FFmpeg DivX (MS MPEG-4 v3) (3IVD) info es la mejor para no perder info
	VideoWriter video("VideoProcessed.avi", cv::VideoWriter::fourcc('3', 'I', 'V', 'D'), fps, Size(frame_width, frame_height), true);

	if (!videoSource.isOpened())
	{
		cout << "Could not open or find the video" << std::endl;
		return -1;
	}

	// Inicialización de variables auxiliares y del clasificador
	Mat frame;
	Mat frame_processed;
	String facedetector_data = "C:/opencv/sources/data/haarcascades/haarcascade_frontalface_alt2.xml";
	CascadeClassifier face_detector;
	std::vector<Rect> faces;

	if (!face_detector.load(facedetector_data))
	{
		printf("--(!)Error loading face detector\n");
		return -1;
	}

	// Creación de visor de imagen
	namedWindow("Processed", WINDOW_KEEPRATIO);

	// Bucle principal de procesamiento
	for (;;)
	{
		// Descarga el frame actual
		videoSource >> frame;
		if (frame.empty())
			break;

		// Creacion del frame resultado
		Size img_size = frame.size();
		frame_processed = frame.clone();
		nindex++;

		// Mejor 1 - Detecta caras en frame cada N frames
		if ((nindex % samples == 0) || (nindex == 1))
		{ 
			face_detector.detectMultiScale(frame_processed, faces, 1.1, 3, 0, Size(MIN_ROI_SIZE, MIN_ROI_SIZE), Size(MAX_ROI_SIZE, MAX_ROI_SIZE));
			nfaces = faces.size();
		}
		else // Si no, hace seguimiento
		{
			camshiftFcn(faces);
		}

		if (nfaces > 0)
			faceProcessing(frame_processed, faces);
		else
			noFace(frame_processed, img_size);

		// Muestra info sobre el video si se ha pulsado la tecla i
		if (video_info)
			updateInfo(frame_processed, nindex, nframes, nfaces, fps, img_size);

		// Chequeo de entrada por teclado
		if (processingPolling())
			return 0;

		// Muestra el frame procesado
		imshow("Processed", frame_processed);

		// Guarda el video
		video.write(frame_processed);

#ifdef DEBUG
		cout << "Frame #: " + to_string(nindex) << std::endl;
		cout << "Status: " + to_string(status) << std::endl;
		cout << "# of faces: " + to_string(nfaces) << std::endl;
		cout << "Blurring: " + to_string(blurring) << std::endl;
		cout << "Process: " + to_string(process) << std::endl;
		cout << "Video info: " + to_string(video_info) << std::endl;
		cout << std::endl;
#endif
	}

	// Cierro el video y libero el recurso
	video.release();

	waitKey(0);
	return 0;
}
//*****************************************************************************
//
//! Funcion updateStatus
//!
//!	Descripcion: Actualiza el estado del procesamiento de manera circular
//!
//! Parametros: nada
//!
//! Salida: nada
//
//*****************************************************************************

void updateStatus(void)
{
	switch (status)
	{
	case 1:
		status = 2;
		break;
	case 2:
		status = 3;
		break;
	case 3:
		status = 1;
		break;
	default:
		break;
	}
}

//*****************************************************************************
//
//! Funcion noFace
//!
//!	Descripcion: Procesa la imagen sin cara detectada
//!
//! Parametros: Imagen de entrada y su tamaño
//!
//! Salida: nada
//
//*****************************************************************************

void noFace(Mat img, Size img_size)
{
	switch (status)
	{
	case 2: // Escribe en el frame texto en rojo
		putText(img, "ATENCION: ningun rostro detectado", Point(floor(img_size.width * 0.05), floor(img_size.height * 0.8)), 2, 1, Scalar(0, 0, 255), 2, 4);
		break;
	case 3: // Difumina el frame completo
		blur(img, img, Size(floor(img_size.width / 4), floor(img_size.height / 4)));
		break;
	default: // No hace nada
		break;
	}
}

//*****************************************************************************
//
//! Funcion processingPolling
//!
//!	Descripcion: comprueba entrada de teclado para posterior procesado
//!
//! Parametros: nada
//!
//! Salida: booleano para indicar fin de ejecución del procesado
//
//*****************************************************************************

bool processingPolling(void)
{
	char key = (char)waitKey(5);

	switch (key)
	{
	case 'n':
		updateStatus();
		return false;
	case 'N':
		updateStatus();
		return false;
	case '0':	// Mejora 2
		blurring = int(floor(BLURRING_DEFAULT * 1));
		return false;
	case '1':	// Mejora 2
		blurring = int(floor(BLURRING_DEFAULT * 0.1));
		return false;
	case '2':	// Mejora 2
		blurring = int(floor(BLURRING_DEFAULT * 0.2));
		return false;
	case '3':	// Mejora 2
		blurring = int(floor(BLURRING_DEFAULT * 0.3));
		return false;
	case '4':	// Mejora 2
		blurring = int(floor(BLURRING_DEFAULT * 0.4));
		return false;
	case '5':	// Mejora 2
		blurring = int(floor(BLURRING_DEFAULT * 0.5));
		return false;
	case '6':	// Mejora 2
		blurring = int(floor(BLURRING_DEFAULT * 0.6));
		return false;
	case '7':	// Mejora 2
		blurring = int(floor(BLURRING_DEFAULT * 0.7));
		return false;
	case '8':	// Mejora 2
		blurring = int(floor(BLURRING_DEFAULT * 0.8));
		return false;
	case '9':	// Mejora 2
		blurring = int(floor(BLURRING_DEFAULT * 0.9));
		return false;
	case 'i':	// Mejora 3
		video_info = !video_info;
		return false;
	case 32:	// Mejora 2 - space key
		process = !process;
		return false;
	case 'q':
		return true;
	case 'Q':
		return true;
	case 27:	// escape key
		return true;
	default:
		return false;
	}
}

//*****************************************************************************
//
//! Funcion faceProcessing
//!
//!	Descripcion: Difumina los rostros detectados
//!
//! Parametros: Imagen de entrada y coordenadas de la cara detectadas
//!
//! Salida: nada
//
//*****************************************************************************

void faceProcessing(Mat img, std::vector<Rect> roi)
{
	int i = 0; 
	
	// Busca la cara con mayor area
	i = findMaxFace(roi);
	
	// Cara a procesar
	Mat img_roi(img, Rect(roi.at(i).x, roi.at(i).y, roi.at(i).width, roi.at(i).height));

	if (process) // Mejora 2 - si se presiona "espacio" no se procesa el video
		blur(img_roi, img_roi, Size(blurring, blurring));
}

//*****************************************************************************
//
//! Funcion findMaxFace
//!
//!	Descripcion: Detecta la mayor de las caras
//!
//! Parametros: Coordenadas de las caras detectadas
//!
//! Salida: el índice de la mayor de las caras
//
//*****************************************************************************

int findMaxFace(std::vector<Rect> roi)
{
	int area = 0;
	int i = 0;
	int imax = 0;

	for (i = 0; i < roi.size(); i++)
		if (area < roi.at(i).width * roi.at(i).height)
		{
			area = roi.at(i).width * roi.at(i).height;
			imax = i;
		}

	return imax;
}

//*****************************************************************************
//
//! Funcion updateInfo
//!
//!	Descripcion: Actualiza informacion del video
//!
//! Parametros: Imagen de entrada y propiedades del video
//!
//! Salida: nada
//
//*****************************************************************************

void updateInfo(Mat img, int nindex, int nframes, int faces_size, int fps, Size img_size)
{
	string info;

	info = "Frame " + to_string(nindex) + "/" + to_string(nframes) + " - " + to_string(faces_size) + " faces at " + to_string(int(fps)) + "fps";
	putText(img, info, Point(floor(img_size.width * 0.05), floor(img_size.height * 0.9)), 2, 1, Scalar(0, 0, 255), 2, 4);
}

//*****************************************************************************
//
//! Funcion camshiftFcn
//!
//!	Descripcion: Realiza algoritmo de seguimiento
//!
//! Parametros: ROI muestreada cada N frames
//!
//! Salida: nada
//
//*****************************************************************************

void camshiftFcn(std::vector<Rect> roi)
{

	// TODO
}
