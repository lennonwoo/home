#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "highgui.h"
#include "colotracker.h"
#include "region.h"
#include <string>
#include <fstream>

using namespace cv;

cv::Point g_topLeft(0,0);
cv::Point g_botRight(0,0);
cv::Point g_botRight_tmp(0,0);
bool plot = false;
bool g_trackerInitialized = false;
Rect r;
ColorTracker * g_tracker = NULL;

void drawBox(cv::Mat& image, cv::Rect box, cv::Scalar color, int thick){
    rectangle(image, cvPoint(box.x, box.y), cvPoint(box.x + box.width, box.y + box.height), color, thick);
}

static void onMouse( int event, int x, int y, int, void* param)
{
    cv::Mat img = ((cv::Mat *)param)->clone();
    if( event == cv::EVENT_LBUTTONDOWN && !g_trackerInitialized){
        std::cout << "DOWN " << std::endl;
        g_topLeft = Point(x,y);
        plot = true;
    }else if (event == cv::EVENT_LBUTTONUP && !g_trackerInitialized){
        std::cout << "UP " << std::endl;
        g_botRight = Point(x,y);
        plot = false;
        if (g_tracker != NULL)
            delete g_tracker;
        g_tracker = new ColorTracker();
        g_tracker->init(*(cv::Mat *)param, g_topLeft.x, g_topLeft.y, g_botRight.x, g_botRight.y);
        //r = Rect(g_topLeft.x, g_topLeft.y, g_botRight.x-g_topLeft.x, g_botRight.x-g_topLeft.y);
        r = Rect(g_topLeft.x, g_topLeft.y, 0, 0);
        r.width = g_botRight.x-g_topLeft.x;
        r.height = g_botRight.x-g_topLeft.y;
        g_trackerInitialized = true;
    }else if (event == cv::EVENT_MOUSEMOVE && !g_trackerInitialized){
        //plot bbox
        g_botRight_tmp = Point(x,y);
        // if (plot){
        //     cv::rectangle(img, g_topLeft, current, cv::Scalar(0,255,0), 2);
        //     imshow("output", img);
        // }
    }
}


int main(int argc, char **argv) 
{
    BBox * bb = NULL;
    cv::Mat img;
    int captureDevice = 0;
    if (argc > 1)
        captureDevice = atoi(argv[1]);

    //cv::VideoCapture webcam = cv::VideoCapture(captureDevice);
    //webcam.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    //webcam.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
    cv::VideoCapture webcam = cv::VideoCapture(0);
    if (!webcam.isOpened()){
        webcam.release();
        std::cerr << "Error during opening capture device!" << std::endl;
        return 1;
    }

    cv::namedWindow( "output", 0 );
    cv::setMouseCallback( "output", onMouse, &img);

    webcam >> img;
    //cv::Mat temp;
    //img.copyTo(temp);
    //while (!g_trackerInitialized)
    //{
    //    drawBox(img, r, cv::Scalar(0, 0, 255), 1);
    //    cv::imshow("output", img);
    //    temp.copyTo(img);
    //    if (cvWaitKey(20) == 27)
    //        return 1;
    //}


    g_topLeft.x = 205;
    g_topLeft.y = 151;
    g_botRight.x = 205 + 17;
    g_botRight.y = 151 + 50;

    g_tracker = new ColorTracker();
    g_tracker->init(img, g_topLeft.x, g_topLeft.y, g_botRight.x, g_botRight.y);
    g_trackerInitialized = true;


    std::ofstream resultsFile;
    std::string resultsPath = "output.txt";
    resultsFile.open(resultsPath);



    for(;;){

        webcam >> img;
        if (img.empty())
            return 0;

        int c = waitKey(10);
        if( (c & 255) == 27 ) {
            std::cout << "Exiting ..." << std::endl;
            break;
        }
        //some control
        switch( (char)c ){
        case 'i':
            g_trackerInitialized = false;
            g_topLeft = cv::Point(0,0);
            g_botRight_tmp = cv::Point(0,0);
            break;
        default:;

        }

        if (g_trackerInitialized && g_tracker != NULL){
            bb = g_tracker->track(img);
        }

        if (!g_trackerInitialized && plot && g_botRight_tmp.x > 0){
            cv::rectangle(img, g_topLeft, g_botRight_tmp, cv::Scalar(0,255,0), 2);
        }

        if (bb != NULL){
            cv::rectangle(img, Point2i(bb->x, bb->y), Point2i(bb->x + bb->width, bb->y + bb->height), Scalar(255, 0, 0), 3);

            resultsFile << bb->x << "," << bb->y << "," << bb->width << "," << bb->height << std::endl;
            //cv::rectangle(img, Point2i(bb->x-5, bb->y-5), Point2i(bb->x + bb->width+5, bb->y + bb->height+5), Scalar(0, 0, 255), 1);
            delete bb;
            bb = NULL;
        }

        cv::imshow("output", img);

    }

    if (g_tracker != NULL)
        delete g_tracker;

    resultsFile.close();
    return 0;
}