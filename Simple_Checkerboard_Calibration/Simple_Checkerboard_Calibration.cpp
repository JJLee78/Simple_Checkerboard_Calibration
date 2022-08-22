﻿#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <wtypes.h>
//#include <iomanip>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

using namespace std;
using namespace cv;

#define PATTERN_MAX      (80)   // # Number of the pattern images

// Get the horizontal and vertical screen sizes in pixel
void GetDesktopResolution(int& horizontal, int& vertical)
{
    RECT desktop;
    // Get a handle to the desktop window
    const HWND hDesktop = GetDesktopWindow();
    // Get the size of screen to the variable desktop
    GetWindowRect(hDesktop, &desktop);
    // The top left corner will have coordinates (0,0)
    // and the bottom right corner will have coordinates
    // (horizontal, vertical)
    horizontal = desktop.right;
    vertical = desktop.bottom;
}

int main()
{
    int corner_count, found;
    int patternNum = 0;
    int boardRows = 0;
    int boardCols = 0;
    float boardSize = 0;

    vector<Mat> srcImg;
    vector<vector<Point2f>> imgPoints;
    while (boardRows < 5 && boardCols < 5 && boardSize <= 10.0)
    {
        std::cout << "Input the rows, columns and size(mm) of checkerboard(ex : 7 10 25) : ";
        std::cin >> boardRows >> boardCols >> boardSize;
    }

    // load source images
    for (int i = 0; i < PATTERN_MAX; i++)
    {
        string path = "temp\\" + to_string(i) + ".jpg";
        Mat src = imread(path);
        if (i == 0 && src.empty())
            cout << "[Err] Failed to load source img file : " << path << endl;
        else if(i > 0 && src.empty())
        {
            patternNum += i;
            break;
        }
        else
            srcImg.push_back(src);
    }

    // set the 3D coordinates of checkerboard
    vector<Point3f> objPoint;
    for (int m = 0; m < boardRows; m++)
    {
        for (int n = 0; n < boardCols; n++)
        {
            Point3f p(m * boardSize, n * boardSize, 0.0);
            objPoint.push_back(p);
        }
    }
    vector<vector<Point3f>> objPoints;
    for (int i = 0; i < patternNum; i++)
        objPoints.push_back(objPoint);

    // get resolution of display(If source images are too large, displayed in a reduced size)
    int screenWidth, screenHeight;
    GetDesktopResolution(screenWidth, screenHeight);
    int found_num = 0;
    Size pattern_size = Size(boardCols, boardRows);
    vector<Point2f> corners;
    for (int i = 0; i < patternNum; i++)
    {
        Mat src_gray = Mat(srcImg[i].size(), CV_8UC1);
        cvtColor(srcImg[i], src_gray, COLOR_BGR2GRAY);
        // find coordinates of chessboard box
        bool isCalibrated = findChessboardCorners(src_gray, pattern_size, corners);
        if (isCalibrated)
        {
            cout << "[PASS] : " << i << ".jpg" << endl;
            found_num++;
        }
        else
            cout << "[FAIL] : " << i << ".jpg" << endl;

        // calculate subpixel of corners with criteria
        cornerSubPix(src_gray, corners, Size(10, 10), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.001)); 
     //   line(srcImg[i], corners[0], corners[1], Scalar(0, 0, 255), 5, 0);
      //  line(srcImg[i], corners[0], corners[2], Scalar(255, 0, 0), 5, 0);
      //  line(srcImg[i], corners[0], corners[3], Scalar(0, 255, 0), 5, 0);
        if (i == patternNum - 1) // show the result of calibration(only last image)
        {
            drawChessboardCorners(srcImg[i], pattern_size, corners, isCalibrated);
            Mat showingMat;
            if (srcImg[i].cols > screenWidth * 0.7)
                resize(srcImg[i], showingMat, Size(srcImg[i].cols / 2, srcImg[i].rows / 2));
            else if (srcImg[i].rows > screenHeight * 0.7)
                resize(srcImg[i], showingMat, Size(srcImg[i].cols / 2, srcImg[i].rows / 2));
            else
                showingMat = srcImg[i];
        
          //  imshow("Calibrating..", showingMat);
         //   moveWindow("Calibrating..", 10, 10);
           // waitKey(0);
        }

        //cout << "aft corner11 xy : " << corners[11].x << " " << corners[11].y << endl;
        
        imgPoints.push_back(corners);
        if (i == patternNum - 1) // show the result of calibration(only last image)
        {
            Mat camIntrinsic; // camera intrinsic
            Mat camDistort; // lens distortion
            vector<Mat> camRotVec, camTransVec; // rotation vector and transfromation vector of each source image
            calibrateCamera(objPoints, imgPoints, srcImg[0].size(), camIntrinsic, camDistort, camRotVec, camTransVec);
            cout << "===== Calibration Result =====" << endl;
            cout << "Camera intrinsic parameters :" << endl;
            cout << camIntrinsic << endl;
            cout << "Lens distortion coefficients :" << endl;
            cout << camDistort << endl;
            cout << "Camera extrinsic parameters :" << endl;
            cout << "----#0----" << endl;
            vector<vector<Point3f>> rVec;
            vector<vector<Point3f>> tVec;
            cv::Mat rvec(3, 1, CV_64FC1);
            cv::Mat tvec(3, 1, CV_64FC1);

            solvePnPRansac(objPoints[i], corners, camIntrinsic, camDistort, rvec, tvec);
            cout << "----#1----" << endl;
            vector<Point2f> corners_rotated;

            vector<cv::Point3f> xyz;
            xyz.push_back(Point3f(3, 0, 0));
            xyz.push_back(Point3f(0, 3, 0));
            xyz.push_back(Point3f(0, 0, -3));

            projectPoints(xyz, rvec, tvec, camIntrinsic, camDistort, corners_rotated);
            cout << "----#2----" << endl;
            line(srcImg[i], corners_rotated[0], corners_rotated[0], Scalar(0, 0, 255), 5);
            line(srcImg[i], corners_rotated[0], corners_rotated[1], Scalar(255, 0, 0), 5);
            line(srcImg[i], corners_rotated[0], corners_rotated[2], Scalar(0, 255, 0), 5);
            cout << "#3" << endl;


            projectPoints(objPoints[0], rvec, tvec, camIntrinsic, camDistort, corners_rotated);
            cout << "----#2----" << endl;
            line(srcImg[i], corners_rotated[0], corners_rotated[1], Scalar(0, 0, 255), 5);
            cout << "#3" << endl;
            /*cv::VideoCapture Capture;
            Capture.open(0);
            Capture.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));
            Capture.set(CV_CAP_PROP_FRAME_WIDTH, 1920);
            Capture.set(CV_CAP_PROP_FRAME_HEIGHT, 1080);
            Mat showing;
            while (Capture.read(showing))
            {
                imshow("video", showing);
                waitKey(1);
            }*/
            imshow("rotated", srcImg[i]);
            moveWindow("rotated", 10, 10);
            waitKey(0);

        }
    }
    destroyWindow("Calibrating..");

    if (found_num != patternNum)
    {
        cerr << "Calibration Images are insufficient." << endl;
        return -1;
    }


    //line(srcImg[9], corners[0], corners[2], Scalar(255, 0, 0), 5, 0);
    //line(srcImg[9], corners[0], corners[3], Scalar(0, 255, 0), 5, 0);

        //imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)
  /*  for (Mat& rottemp : camRotVec)
        cout << rottemp;
    cout << endl;
    for (Mat& transtemp : camTransVec)
        cout << transtemp;
    cout << endl;*/

    return 0;
}