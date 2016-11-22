
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "opencv2/imgproc/imgproc.hpp"
#include <math.h>

//cpp libraries now! Frankenstein source code!
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d.hpp"

using namespace cv;

/*************************************************************
Program to test basic object recognition and tracking using
OpenCV in a real-time video feed.
It uses the HSV colour space to detect Hue (main position
in colour palette) and Saturation (colour distribution/purity)
of the object which is more robust than using RGB and more
resilient to variations in the ilumination conditions.
**************************************************************/

int main(int argc, char *argv[])
{
	// Define Camera
	CvCapture * captureDevice;

	// Create variables (memory space) for
	// Images used during processing
	IplImage* frame;
	IplImage* hsv;
	IplImage* h;
	IplImage* s;
	IplImage* v;

	// Threshold for hue and saturation values
	// This values can be found empirically (for my apple!)
	// or by analysing the images, etc...
	int hLowThresh = 38;//170;
	int hHighThresh = 90;//180;
	int sLowThresh = 60;//115;
	int sHighThresh = 255;//210;

	// Are we trying to detect our object in a dark room?
	bool darkness = false;

	// Variable used for sum
	int sum = 0;

	// Create handles and titles for interface
	// Added a "const" here to get rid of the warning!
	const char* win2 = "Binary Image";

	// Variables for iterations
	int i, j;

	// Keypress variable
	int key = 0;

	// Capture a frame to test that the camera is working
	captureDevice = cvCaptureFromCAM(0);
	if(!captureDevice)
	{
		printf("Error - Camera not found\n");
		return 0;
	}

	// Create and Open the inetrface windows
	cvNamedWindow(win2, CV_WINDOW_AUTOSIZE );

	// Add widgets for thresholding (the control bars)
	// these can be used to experiment and try detecting other objects
	// or in different light conditions
	cvCreateTrackbar( "H Low Threhold", win2, &hLowThresh, 180, NULL );
	cvCreateTrackbar( "H High Threhold", win2, &hHighThresh, 180, NULL );
	cvCreateTrackbar( "S Low Threhold", win2, &sLowThresh, 255, NULL );
	cvCreateTrackbar( "S High Threhold", win2, &sHighThresh, 255, NULL );

	// Init images
	hsv = cvQueryFrame(captureDevice);
	h = cvCreateImage(cvSize(hsv->width, hsv->height) , IPL_DEPTH_8U, 1);
	s = cvCreateImage(cvSize(hsv->width, hsv->height) , IPL_DEPTH_8U, 1);
	v = cvCreateImage(cvSize(hsv->width, hsv->height) , IPL_DEPTH_8U, 1);

	// Main loop. Loop while "Esc" key NOT pressed
	// the program can also be closed with ctrl-C
	while (key != 27)
	{
		// Capture a frame
		frame = cvQueryFrame(captureDevice);


		// Now process the frame
		// ---------------------
		// Convert to HSV
		cvCvtColor(frame, hsv, CV_BGR2HSV);
		// Split into the different channels
		cvSplit(hsv, h, s, v, NULL);

		// Reset sum
		sum = 0;

		// Let's convert the (primitive) IplImage to (the more modern) Mat
		Mat va = cvarrToMat(v);

		// Detect (roughly) if we are in dark conditions
		for(i = 0;i < va.cols; i++){
			for(j = 0; j < va.rows; j++){
				Vec3b colour = va.at<Vec3b>(Point(i,j));
				sum += colour[0];
			}
		}

		sum = (int)sum/(va.cols*va.rows);

		if(sum > 80){
			darkness = false;
		}
		else{
			darkness = true;
		}

		// Applying the threshold below will produce a binary image the
		// shows either nothing, or the pixels that we consider valid.
		// Threshold the hue channel with the values we have defined
		// for our specific object 
		cvThreshold(h, h, hLowThresh, 255, CV_THRESH_TOZERO);
		cvThreshold(h, h, hHighThresh, 255, CV_THRESH_TOZERO_INV);
		cvThreshold(h, h, 0, 255, CV_THRESH_BINARY);

		// Threshold saturation channel in the same way
		cvThreshold(s, s, sLowThresh, 255, CV_THRESH_TOZERO);
		cvThreshold(s, s, sHighThresh, 255, CV_THRESH_TOZERO_INV);
		cvThreshold(s, s, 0, 255, CV_THRESH_BINARY);

		// Combine the thresholded channels with logical AND
		// to get a binary image of ONLY the pixels that have BOTH
		// the interesting values of Hue AND Saturation
		cvAnd(h,s,v,NULL);

		// Use erode to clean the image somewhat
		cvErode(v,v,NULL,1);

		// This is what we get from the provided code, with the modified thresholds
		cvShowImage(win2,v);

		// Let's get rid of the bug as indicated by the provided code
		cvQueryFrame(captureDevice);



		// From now on, I am using almost entirely OpenCV3 as it would be used in cpp!
		// (Some cpp and openCV3 are also used above)

		// Let's convert the (primitive) IplImage to (the more modern) Mat
		Mat gray = cvarrToMat(v);

		// Now let's blur what we have so far, to get rid of rough edges
		blur(gray, gray, Size(30,30));

		// Now let's invert the colours of the image
		for(i = 0;i < gray.cols; i++){
			for(j = 0; j < gray.rows; j++){
				Vec3b & colour = gray.at<Vec3b>(Point(i,j));
				colour[0] = 255 - colour[0];
				colour[1] = 255 - colour[1];
				colour[2] = 255 - colour[2];
			}
		}


		SimpleBlobDetector::Params params;
		 
		// We will detect the blobs using colours
		// and specifically the black colour, since
		// SimpleBlobDetector works MUCH better (or only)
		// while looking for darker colours. This is the reason
		// we inverted the colours of the grayscale image.
		params.filterByColor = true;
		params.blobColor = 0;

		// We also want to detect rather big objects,
		// so we set the minArea value quite high.
		params.filterByArea = true;
		params.minArea = 2000;
		params.maxArea = 300000;
		 
		// Our object is quite circular, so use this filter
		// If we turn off this filter, the code works pretty impressively
		// in the dark too!
		
		if(!darkness){
			params.filterByCircularity = true;
			params.minCircularity = 0.7;
		}
		else{
			params.filterByCircularity = false;
		}
		 

		// No detection based on convexity
		params.filterByConvexity = false;
		 

		// No detection based on inertia
		params.filterByInertia = false;

		// Instatiate the SimpleBlobDetector based on the parameters above
		Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params); 

		// Store the detected blobs here
		std::vector<KeyPoint> blobs;

		// Detect our object!
		detector->detect( gray, blobs );
		
		// Cover from IplImage to Mat again, for better compatibility
		Mat final_image = cvarrToMat(frame);

		// Draw our detected object(s)
		drawKeypoints( final_image, blobs, final_image, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

		// Show the Binary inverted blurred image
		imshow("Binary-Inverted-Blurred!", gray );

		// Show the world what we detected
		imshow("Detected Object", final_image );

		// Test for a Key press with a short wait
		key = cvWaitKey(100);
	}

	// Clean up
	cvDestroyWindow(win2);

	return 0;
}
