/*
 * The Example of the Withrobot oCam-1MGN-U API using with OpenCV. (Linux only.)
 *
 * This example shows how to get image from the oCam-1MGN using the Withrobot camera API.
 * And also shows how to control the oCam-1MGN using the Withrobot camera API.
 *
 * This example program usage.:
 * 	- Press the key 'q' for exit this program.
 *  - Press the key ']' for increase the exposure.
 *  - Press the key '[' for decrease the exposure.
 *  - Press the key '=' for increase the brightness.
 *  - Press the key '-' for decrease the brightness.
 */

#include <stdio.h>
#include <errno.h>
#include <iostream>
#include <fstream>
#include <unistd.h>

#include "opencv2/opencv.hpp"
// #include "libusb.h"
#include "withrobot_camera.hpp"	/* withrobot camera API */
#include <ctime>
#include "serialunix.h"
#include <chrono>
/*
 *	Main
 */
int main (int argc, char* argv[])
{
    std::string path("/dev/ttyACM0");
    SerialUnix serial(path);
    serial.sendPacket();
	const char* devPath = "/dev/video1";

    Withrobot::Camera camera(devPath);


    camera.set_format(1280, 720, Withrobot::fourcc_to_pixformat('G','R','E','Y'), 1, 60);

    Withrobot::camera_format camFormat;
    camera.get_current_format(camFormat);

    /*
     * Print infomations
     */
    std::string camName = camera.get_dev_name();
    std::string camSerialNumber = camera.get_serial_number();

    printf("dev: %s, serial number: %s\n", camName.c_str(), camSerialNumber.c_str());
    printf("----------------- Current format informations -----------------\n");
    camFormat.print();
    printf("---------------------------------------------------------------\n");

    /*
     * [ supported camera controls; The double quotes are the 'get_control' and the 'set_control' function string argument values. ]
     *
     *  [0] "Brightness",          Value(default [min, step, max]): 64 ( 64 [0, 1, 127] )  // gain
     *  [1] "Exposure (Absolute)", Value(default [min, step, max]): 39 ( 39 [1, 1, 625] )
     *
     */
    int brightness = camera.get_control("Brightness");
    int exposure = camera.get_control("Exposure (Absolute)");

    camera.set_control("Brightness", brightness);
    camera.set_control("Exposure (Absolute)", exposure);

    /*
     * Start streaming
     */
    if (!camera.start()) {
    	perror("Failed to start.");
    	exit(0);
    }

    /*
     * Initialize OpenCV
     */
    std::string windowName = camName + " " + camSerialNumber;
    cv::Mat srcImg(cv::Size(camFormat.width, camFormat.height), CV_8UC1);
    cv::namedWindow(windowName.c_str(), CV_WINDOW_KEEPRATIO|CV_WINDOW_AUTOSIZE);

    /*
     * Main loop
         */
    bool quit = false;
    cv::Point pt1(600, 300);
    cv::Point pt2(700, 400);
    clock_t begin = clock();
    clock_t end = clock();

    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point stop= std::chrono::steady_clock::now();


    std::ofstream stm;
    stm.open( "/dev/ttyACM0");
    double delta_time = 0.0;
    double mesaure_delta = true;
    double elapsed_secs = 0;
    std::string elapsed_secs_string = "";
    while (!quit) {
    	/* Copy a single frame(image) from camera(oCam-1MGN). This is a blocking function. */
    	int size = camera.get_frame(srcImg.data, camFormat.image_size, 1);
    	/* If the error occured, restart the camera. */
    	if (size == -1) {
    	    printf("error number: %d\n", errno);
    	    perror("Cannot get image from camera");
    	    camera.stop();
    	    camera.start();
    	    continue;
    	}

        
        
        

        cv::rectangle (srcImg, pt1, pt2, cv::Scalar(0));
        cv::Mat submat = cv::Mat(srcImg, cv::Rect(pt1, pt2));
        float meanBright = mean(submat)[0];

        // end = clock();
        stop= std::chrono::steady_clock::now();
        
        
        std::string meanBright_string = "meanBright " + std::to_string(meanBright);

        if ((meanBright> 100) and (mesaure_delta))
        {
            
            elapsed_secs = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count();
            elapsed_secs_string = std::to_string(elapsed_secs);
            // elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
            // elapsed_secs_string = std::to_string(elapsed_secs);
            mesaure_delta = false;
            // delta_time = 
        }

        cv::putText(srcImg, meanBright_string, cvPoint(30,60), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);
        if (mesaure_delta == false)
            {cv::putText(srcImg, elapsed_secs_string, cvPoint(30,30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);}
        else 
            {
             {cv::putText(srcImg, "waiting for SPACE ", cvPoint(30,30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);}
            }
            

    	/* Show image */
    	cv::imshow(windowName.c_str(), srcImg);
    	char key = cv::waitKey(5);

    	/* Keyboard options */
    	switch (key) {
    	/* When press the 'q' key then quit. */
    	case 'q':
    		quit = true;
    		break;

    	/* When press the '[' key then decrease the exposure time. */
    	case '[':
    		exposure = camera.get_control("Exposure (Absolute)");
    		camera.set_control("Exposure (Absolute)", --exposure);
    		break;

		/* When press the ']' key then increase the exposure time. */
    	case ']':
    		exposure = camera.get_control("Exposure (Absolute)");
    		camera.set_control("Exposure (Absolute)", ++exposure);
    		break;

		/* When press the '-' key then decrease the brightness. */
    	case '-':
    		exposure = camera.get_control("Brightness");
    		camera.set_control("Brightness", --brightness);
    		break;

		/* When press the '=' key then increase the brightness. */
    	case '=':
    		exposure = camera.get_control("Brightness");
    		camera.set_control("Brightness", ++brightness);
    		break;

        case ' ':
            // begin = clock();
            start = std::chrono::steady_clock::now();
            mesaure_delta = true;
            // usleep(200000);
            serial.sendPacket();


            // send signal
            break;

    	default:
    		break;
    	}
    }

    cv::destroyAllWindows();

    /*
     * Stop streaming
     */
    camera.stop();

	printf("Done.\n");

	return 0;
}
