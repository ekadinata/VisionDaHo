#include "cv.h"
#include "highgui.h"
#include <stdio.h>

using namespace cv;
using namespace std;


bool calibrationEnabled = false;
bool flipEnabled = false;
bool flag;
const int max_val = 255; 
int min_H = 0;
int max_H = 255;
int min_S = 0;
int max_S = 255;
int min_V = 0;
int max_V = 255;

int lowThreshold = 150;
int const max_lowThreshold = 500;
int ratio = 0.3;
int kernel_size = 3;

int min_dist = 3;
int upper_threshold = 50;
int center_threshold = 45;
int min_r = 10;
int max_r = 400;

Point pos(-1,-1);

Mat image,img,hsv,th,gray,edge;

void on_trackbar(int value, void *userdata){}

void CreateTrackbar(){
	namedWindow("Kalibrasi");
	createTrackbar( "min_H", "Kalibrasi", &min_H, max_val, on_trackbar );
	createTrackbar( "max_H", "Kalibrasi", &max_H, max_val, on_trackbar );
	createTrackbar( "min_S", "Kalibrasi", &min_S, max_val, on_trackbar );
	createTrackbar( "max_S", "Kalibrasi", &max_S, max_val, on_trackbar );
	createTrackbar( "min_V", "Kalibrasi", &min_V, max_val, on_trackbar );
	createTrackbar( "max_V", "Kalibrasi", &max_V, max_val, on_trackbar );
	createTrackbar("lowThreshold", "Kalibrasi", &lowThreshold, max_lowThreshold, on_trackbar);
    createTrackbar("lowThreshold*ratio", "Kalibrasi", &ratio, 50, on_trackbar);
    createTrackbar("kernel_size", "Kalibrasi", &kernel_size, 50, on_trackbar);
    createTrackbar( "min_dist", "Kalibrasi", &min_dist, 20, on_trackbar );
    createTrackbar( "upTh", "Kalibrasi", &upper_threshold, 1000, on_trackbar );
    createTrackbar( "c_th", "Kalibrasi", &center_threshold, 1000, on_trackbar );
    createTrackbar( "min_r", "Kalibrasi", &min_r, 1000, on_trackbar );
    createTrackbar( "max_r", "Kalibrasi", &max_r, 1000, on_trackbar );
}

void morphOps(Mat &thresh){
        //create structuring element that will be used to "dilate" and "erode" image.
        //the element chosen here is a 3px by 3px rectangle
        Mat erodeElement = getStructuringElement(MORPH_RECT, Size(2, 2));
        //dilate with larger element so make sure object is nicely visible
        Mat dilateElement = getStructuringElement(MORPH_RECT, Size(1, 1));

        erode(thresh, thresh, erodeElement);
        erode(thresh, thresh, erodeElement);
        erode(thresh, thresh, erodeElement);

        dilate(thresh, thresh, dilateElement);
        dilate(thresh, thresh, dilateElement);
}

void knowBall(Point center){
	if(flag)
	{
		printf("ADA BOLA\nX:%d, Y:%d\n", center.x, center.y);
	}else{
		printf("TIDAK ADA BOLA\nX:%d, Y:%d\n", center.x, center.y);
	}                         
}

int main(int argc, char const *argv[])
{
	VideoCapture cap(1);
	//cap.open("http://127.0.0.1:9000");	

	while(1){
		cap.read(image);
		if(flipEnabled)
			flip(img, img, 0);
		GaussianBlur( image, img, Size(3, 3), 2, 2 );
		GaussianBlur( img, img, Size(9, 9), 2, 2, 4 );
		cvtColor(img,hsv,CV_BGR2HSV);
		inRange(hsv,Scalar(min_H,min_S,min_V),Scalar(max_H,max_S,max_V),th);
		morphOps(th);
		GaussianBlur( th, th, Size(9, 9), 2, 2, 4 );
	
		Canny( th, edge, lowThreshold, lowThreshold*ratio, kernel_size);
       	//edge.convertTo(draw, CV_8U);		
		GaussianBlur( edge, edge, Size(9, 9), 2, 2, 4 );
		//GaussianBlur(InputArray src, OutputArray dst, Size ksize, double sigmaX, double sigmaY=0, int borderType=BORDER_DEFAULT )
	
		vector<Vec3f> circles;
    	HoughCircles(edge, circles, CV_HOUGH_GRADIENT,1, edge.rows/min_dist, upper_threshold, center_threshold,min_r,max_r);
        if(circles.size()>0)
        {	flag = true;
	        for( size_t i = 0; i < circles.size(); i++ ) 
	        {
	        	//tanda = true;
	            Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
	            int radius = cvRound(circles[i][2]);
	            //draw the circle center
	            circle( image, center, 3, Scalar(0,255,0), -1, 8, 0 );
	            //draw the circle outline
	            circle( image, center, radius, Scalar(0,0,255), 3, 8, 0 );
	        	pos.x = cvRound(circles[i][0]);
	        	pos.y = cvRound(circles[i][1]);
	        }
	   	}
	   	else
	   	{
	   		flag = false;
	   		pos.x = -1;
	   		pos.y = -1;
	   	}

	   	knowBall(pos);
	   	
	   	namedWindow("Image", CV_WINDOW_NORMAL);
		imshow( "Image", image );
		
		if(calibrationEnabled)
		{
			namedWindow("Image", CV_WINDOW_NORMAL);
			imshow( "Image", image );
			CreateTrackbar();
			namedWindow("Edge", CV_WINDOW_NORMAL);
	    	imshow( "Edge", edge );
	    	namedWindow("Threshold", CV_WINDOW_NORMAL);
			imshow( "Threshold", th );
		}else{
			//destroyAllWindows();
			destroyWindow("Edge");
			destroyWindow("Threshold");
			destroyWindow("Kalibrasi");
		}
		
		switch(waitKey(10)){
            case 27: //'esc' key has been pressed, exit program.
                return 0;
            case 102: //'t' has been pressed. this will toggle tracking
                flipEnabled = !flipEnabled;
                if(flipEnabled == false) 
                	cout<<"flip on"<<endl;
                else 
                	cout<<"flip off"<<endl;
                break;
            case 99:
                calibrationEnabled = !calibrationEnabled;
                if(calibrationEnabled == false)
                    cout<<"calibration on"<<endl;
                else
                    cout<<"calibration off"<<endl;
        }
	}
	return 0;
}
