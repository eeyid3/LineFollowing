// Include files for required libraries
#include <stdio.h>
#include <algorithm>
#include "opencv_aee.hpp"
#include "main.hpp"     // You can use this file for declaring defined values and functions
#include "pi2c.h"


using namespace cv;
using namespace std;

Pi2c car(0x22); // Configure the I2C interface to the Car as a global variable

void setup(void)
{
    setupCamera(320, 240);  // Enable the camera for OpenCV

}


int getMaxAreaContourId(vector <vector<cv::Point>> contours) {
    double maxArea = 0;
    int maxAreaContourId = -1;
    for (int j = 0; j < contours.size(); j++) {
        double newArea = cv::contourArea(contours.at(j));
        if (newArea > maxArea) {
            maxArea = newArea;
            maxAreaContourId = j;
        } // End if
    } // End for
    return maxAreaContourId;
} // End function

int getSecondAreaContourId(vector <vector<cv::Point>> contours) {
    double maxArea = 0;
    double secondArea = 0;
    int maxAreaContourId = -1;
    int secondAreaId = -1;
    for (int j = 0; j < contours.size(); j++) {
        double newArea = cv::contourArea(contours.at(j));
        if (newArea > maxArea) {
            secondArea = maxArea;
            secondAreaId = maxAreaContourId;
            maxArea = newArea;
            maxAreaContourId = j;
        } // End if
        else if(newArea > secondArea) {
            secondArea = newArea;
            secondAreaId = j;
        }
    } // End for
    return secondAreaId;
} // End function



int main( int argc, char** argv )
{
    setup();    // Call a setup function to prepare IO and devices

   // namedWindow("HSV Tester");   // Create a GUI window called photo

    int lowH = 0, highH = 179, lowS = 0, highS = 255, lowV = 0, highV = 255;    // Initialise some variables for HSV limits
    int lowHP = 122, highHP = 179, lowSP = 9, highSP = 145, lowVP = 67, highVP = 112;    // Initialise some variables for HSV limits

    float prev[5] = {0,0,0,0,0};

/*
    createTrackbar("Low Hue", "HSV Tester", &lowH, 179, NULL);      // Create trackbar controls for each HSV limit
    createTrackbar("High Hue", "HSV Tester", &highH, 179, NULL);

    createTrackbar("Low Sat", "HSV Tester", &lowS, 255, NULL);
    createTrackbar("High Sat", "HSV Tester", &highS, 255, NULL);

    createTrackbar("Low Value", "HSV Tester", &lowV, 255, NULL);
    createTrackbar("High Value", "HSV Tester", &highV, 255, NULL);

    */
    Mat frame = imread("OpenCV_Logo.png"); // Open an image file and store in a new matrix variable


    //load in symbols to compare with
    Mat circleSym = imread("Circle.png");
    Mat starSym = imread("Star.png");
    Mat umbrellaSym = imread("Umbrella.png");
    Mat triangleSym = imread("Triangle.png");

    //convert symbols to HSV
    Mat circleHSV;
    Mat starHSV;
    Mat umbrellaHSV;
    Mat triangleHSV;

    cvtColor(circleSym, circleHSV, COLOR_BGR2HSV);
    cvtColor(starSym, starHSV, COLOR_BGR2HSV);
    cvtColor(umbrellaSym, umbrellaHSV, COLOR_BGR2HSV);
    cvtColor(triangleSym, triangleHSV, COLOR_BGR2HSV);

    inRange(circleHSV, Scalar(0,0,0), Scalar(179,255, 200), circleHSV);
    inRange(starHSV, Scalar(0,0,0), Scalar(179,255,200), starHSV);
    inRange(umbrellaHSV, Scalar(0, 0, 0), Scalar(179, 255,200), umbrellaHSV);
    inRange(triangleHSV, Scalar(0,0,0), Scalar(179, 255, 200), triangleHSV);

    cout << circleSym.size;

    int colour = 4;
    while(1)    // Main loop to perform image processing
    {
        Mat lframe;

        while(lframe.empty())
            lframe = captureFrame(); // Capture a frame from the camera and store in a new matrix variable

        Mat frame = lframe(Range(0,160),Range(0, 320));
        rotate(frame, frame, ROTATE_180);
        // colour red = 1 green = 2 blue = 3 black = 4


        if(colour == 1){
        lowH = 147;
        highH = 179;
        lowS = 158;
        highS = 255;
        lowV = 65;
        highV = 207;
        }
        if(colour == 2){
        lowH = 63;
        highH = 87;
        lowS = 74;
        highS = 255;
        lowV = 63;
        highV = 125;
        }
        if(colour == 3){
        lowH = 88;
        highH = 125;
        lowS = 170;
        highS = 255;
        lowV = 29;
        highV = 200;
        }
        if(colour == 4){
        lowH = 0;
        highH = 179;
        lowS = 22;
        highS = 255;
        lowV = 0;
        highV = 67;
        }

        /*
        lowH = getTrackbarPos("Low Hue", "HSV Tester");        // Update the variables with the trackbar setting
        highH = getTrackbarPos("High Hue", "HSV Tester");
        lowS = getTrackbarPos("Low Sat", "HSV Tester");
        highS = getTrackbarPos("High Sat", "HSV Tester");
        lowV = getTrackbarPos("Low Value", "HSV Tester");
        highV = getTrackbarPos("High Value", "HSV Tester");
        */

        Mat frameHSV;       // Convert the frame to HSV and apply the limits
        Mat frameHSVp;      //Convert for symbol frame too
        //Mat frameHSVred;
        cvtColor(frame, frameHSV, COLOR_BGR2HSV);
        inRange(frameHSV, Scalar(lowHP, lowSP, lowVP), Scalar(highHP, highSP, highVP), frameHSVp);
        inRange(frameHSV, Scalar(lowH, lowS, lowV), Scalar(highH, highS, highV), frameHSV);



    /*
        if (colour == 1)
        {
            Mat frameHSVred;
            inRange(frameHSV, Scalar(0, 158, 65), Scalar(10, 255, 207), frameHSVred);

            addWeighted(frameHSV, 1.0, frameHSVred, 1.0, 0.0, frameHSV);

        }
*/

        //Space for frame manipulation

        vector<vector<cv::Point>> contours;
        vector<Vec4i> hierarchy;
        findContours(frameHSV, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

        vector<vector<cv::Point>> contoursP;
        vector<Vec4i> hierarchyP;
        findContours(frameHSVp, contoursP, hierarchyP, RETR_TREE, CHAIN_APPROX_SIMPLE);

        if (contours.empty() == false)
        {
        //draw contours
        drawContours(frame, contours, getMaxAreaContourId(contours), Scalar(0, 255, 0), 2, FILLED, hierarchy, 1);

        //Rect box = boundingRect(contours[getMaxAreaContourId(contours)]);
        //rectangle(frame, box, Scalar(0, 0, 255));

        //find minimum area box
        int tempId = getMaxAreaContourId(contours);
        if (tempId == -1){tempId = 0; colour = 4;}

        RotatedRect box = minAreaRect(contours.at(tempId));

        //separate points of rectangle
        Point2f points[4];
        box.points(points);

        //draw points
        line(frame, points[0], points[1], cv::Scalar(255, 0, 0), 3);
        line(frame, points[1], points[2], cv::Scalar(255, 0, 0), 3);
        line(frame, points[2], points[3], cv::Scalar(255, 0, 0), 3);
        line(frame, points[3], points[0], cv::Scalar(255, 0, 0), 3);

        Point2f centrePoint = box.center;
        //cout << centrePoint;

        int stretch1 = pow((points[0].x - points[1].x),2) + pow((points[0].y - points[1].y),2);
        int stretch2 = pow((points[1].x - points[2].x),2) + pow((points[1].y - points[2].y),2);

        int angle = round(box.angle);
        if (stretch1<=stretch2)
        {
            angle = angle + 90;
        }

        float offset = centrePoint.x;

        cout << "\n";
        //cout << angle;
        //cout << " ";

        offset = ((offset * 9)/16) - 90;
        //cout << offset;
        //cout << " ";
        float error = (2*angle)+(3*offset);

        //move previous errors up one
        for (int isaac = sizeof(prev) - 2; isaac > -1; isaac--){

        prev[isaac + 1] = prev[isaac];

        }
        prev[0] = error;

        //recalculate error with weighted average
        error = prev[1] + prev[2] + prev[3] + prev[4] + (4*prev[0]);

        int errori = round((error/8)+450);
        /*
        cout << offset;

        cout << " ";
        cout << angle;
        */
        //cout << " ";

        //cout << errori;


        char tv [2];
        tv[0] = (errori >> 8) & 0xFF;
        tv[1] = errori & 0xFF;
        car.i2cWrite(tv, 2);

        }
        else {

        //cout << "line not found";

        if(colour == 4)
        {
        int code = 9999;
                char tv [2];
        tv[0] = (code >> 8) & 0xFF;
        tv[1] = code & 0xFF;
        car.i2cWrite(tv, 2);
        }else
        {
            colour = 4;
        }

        }

        if(contoursP.empty() == false)
        {
            int maxP = getMaxAreaContourId(contoursP);
            int secP = getSecondAreaContourId(contoursP);
            if(maxP != -1 && secP != -1)
            {
                //draw contours onto frame
                //drawContours(frame, contoursP, maxP, Scalar(0, 0, 255), 2, FILLED, hierarchyP, 1);
                //drawContours(frame, contoursP, secP, Scalar(0, 0, 255), 2, FILLED, hierarchyP, 1);

                //use bounding rectangles to find which contour is the square
                Rect box1P = boundingRect(contoursP[maxP]);
                Rect box2P = boundingRect(contoursP[secP]);

                //create a matrix of points
                Point2f boundary[4];

                vector<Point> boxBound;

                if(box1P.area() > box2P.area())
                {
                    RotatedRect maxBoxP = minAreaRect(contoursP.at(maxP));
                    maxBoxP.points(boundary);
                    approxPolyDP(contoursP.at(maxP), boxBound, arcLength(contoursP.at(maxP), true) * 0.02, true);

                } else
                {
                    RotatedRect secBoxP = minAreaRect(contoursP.at(secP));
                    secBoxP.points(boundary);
                    approxPolyDP(contoursP.at(secP), boxBound, arcLength(contoursP.at(secP), true) * 0.02, true);

                }


                Mat transformed = transformPerspective(boxBound, frame, 350, 350);

                if(transformed.empty() == false)
                {

                Mat symbolHSV;
                inRange(transformed, Scalar(50, 9, 0), Scalar(179, 200, 110), symbolHSV);

                imshow("Symbol Window", symbolHSV);



                float Cmatch = compareImages(symbolHSV, circleHSV);
                float Umatch = compareImages(symbolHSV, umbrellaHSV);
                float Tmatch = compareImages(symbolHSV, triangleHSV);
                float Smatch = compareImages(symbolHSV, starHSV);

                float TopPercent = std::max({Cmatch,Umatch,Tmatch,Smatch});
                if(TopPercent>25)
                {
                    if(TopPercent == Cmatch){
                    colour = 1;
                    }
                    else if(TopPercent == Umatch){
                    printf("Shape = Umbrella");
                    }
                    else if(TopPercent == Tmatch){
                    colour = 3;
                    }
                    else if(TopPercent == Smatch){
                    colour = 2;
                    }

                }


                }


            }
        }

        Mat comparison;     // Join the two into a single image
        cvtColor(frameHSV, frameHSV, COLOR_GRAY2BGR);   // In range returns the equivalent of a grayscale image so we need to convert this before concatenation
        hconcat(frame, frameHSV, comparison);
        imshow("HSV Tester", comparison); //Display the image in the window

        int key = cv::waitKey(1);   // Wait 1ms for a keypress (required to update windows)

        key = (key==255) ? -1 : key;    // Check if the ESC key has been pressed
        if (key == 27)
            break;
	}


	closeCV();  // Disable the camera and close any windows

	return 0;
}
