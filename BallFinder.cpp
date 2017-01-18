// Soccer Vision 
// Author: Andreas Ekadinata Widodo <ekadinataa@gmail.com>

/**
* KRSBI ITB
* Dago Hoogeschool Team
**/

#include "BallFinder.h"


using namespace std;
using namespace Robot;

BallFinder* BallFinder::m_UniqueInstance = new BallFinder();
/*
// Const like Walking
BallFinder::BallFinder()
{
    min_H = 0;
    max_H = 255;
    min_S = 0;
    max_S = 255;
    min_V = 0;
    max_V = 255;

    lowThreshold = 150;
    ratio = 1;
    kernel_size = 3;

    min_dist = 3;
    upper_threshold = 50;
    center_threshold = 45;
    min_r = 10;
    max_r = 400;   
}
//////////////////////
*/

//Cons like ColorFinder
BallFinder::BallFinder() : 
        pos(Point2D()),
        min_H(0),
        max_H(255),
        min_S(0),
        max_S(255),
        min_V(0),
        max_V(255),
        lowThreshold(150),
        ratio(1),
        kernel_size(3),
        min_dist(3),
        upper_threshold(50),
        center_threshold(45),
        min_r(10),
        max_r(400),
        color_section("")
        //m_result(0)
{ }

BallFinder::BallFinder(int min_H, int max_H, int min_S, int max_S, int min_V, int max_V, int lowThreshold, int ratio, int kernel_size, int min_dist, int upper_threshold, int center_threshold, int min_r, int max_r) : 
        min_H(min_H),
        max_H(max_H),
        min_S(min_S),
        max_S(max_S),
        min_V(min_V),
        max_V(max_V),
        lowThreshold(lowThreshold),
        ratio(ratio),
        kernel_size(kernel_size),
        min_dist(min_dist),
        upper_threshold(upper_threshold),
        center_threshold(center_threshold),
        min_r(min_r),
        max_r(max_r),
        color_section("")
{ }
/////////////////////////////////////

BallFinder::~BallFinder()
{ /* NONE */ }

void BallFinder::on_trackbar(int value, void *userdata){}

void BallFinder::CreateTrackbar(){
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

void BallFinder::morphOps(Mat &thresh){
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

void BallFinder::isDetectBall(Point2D center){
    if(flag)
    {
        printf("\nBALL DETECTED >> X:%lf, Y:%lf\n\r", center.X, center.Y);
    }else{
        printf("\nNOT DETECTED >> X:%lf, Y:%lf\n\r", center.X, center.Y);
    }                         
}

void BallFinder::Process(Mat image)
{ 
    GaussianBlur( image, img, Size(3, 3), 2, 2 );
    GaussianBlur( img, img, Size(9, 9), 2, 2, 4 );
    cvtColor(img,hsv,CV_BGR2HSV);
    inRange(hsv,Scalar(min_H,min_S,min_V),Scalar(max_H,max_S,max_V),th);
    morphOps(th);
    
    GaussianBlur( th, th, Size(9, 9), 2, 2, 4 );
    //fastNlMeansDenoisingColored( hsv, x, 3, 3, 3, 20 ); 
    //fastNlMeansDenoisingColoredMulti()    

    Canny( th, edge, lowThreshold, lowThreshold*ratio, kernel_size);
    GaussianBlur( edge, edge, Size(9, 9), 2, 2, 4 );

    vector<Vec3f> circles;
    HoughCircles(edge, circles, CV_HOUGH_GRADIENT,1, edge.rows/min_dist, upper_threshold, center_threshold,min_r,max_r);
    if(circles.size()>0)
    {   
        flag = true;
        for( size_t i = 0; i < circles.size(); i++ ) 
        {
            Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            int radius = cvRound(circles[i][2]);
            //draw the circle center
            circle( image, center, 3, Scalar(0,255,0), 15, 8, 0 );
            //draw the circle outline
            circle( image, center, radius, Scalar(100,100,255), 20, 8, 0 );
            pos.X = cvRound(circles[i][0]);
            pos.Y = cvRound(circles[i][1]);
        }
    }
    else
    {
        flag = false;
        pos.X = -1;
        pos.Y = -1;
    }

    isDetectBall(pos);
    printParam();
    
    namedWindow("Image", CV_WINDOW_NORMAL);
    imshow( "Image", image );
    
    waitKey(10);
}

void BallFinder::ControlPanel(minIni* ini)
{
    CreateTrackbar();
    
    namedWindow("Edge", CV_WINDOW_NORMAL);
    imshow( "Edge", edge );
    
    namedWindow("Threshold", CV_WINDOW_NORMAL);
    imshow( "Threshold", th );

    SaveINISettings(ini);
    //waitKey(30);
}


Point2D& BallFinder::getPosition()
{
    return pos;
}


void BallFinder::LoadINISettings(minIni* ini)
{
    LoadINISettings(ini, COLOR_SECTION);
}

void BallFinder::LoadINISettings(minIni* ini, const std::string &section)
{
    int value = -2;
    if((value = ini->geti(section, "min_H", INVALID_VALUE)) != INVALID_VALUE) min_H = value;
    if((value = ini->geti(section, "max_H", INVALID_VALUE)) != INVALID_VALUE) max_H = value;
    if((value = ini->geti(section, "min_S", INVALID_VALUE)) != INVALID_VALUE) min_S = value;
    if((value = ini->geti(section, "max_S", INVALID_VALUE)) != INVALID_VALUE) max_S = value;
    if((value = ini->geti(section, "min_V", INVALID_VALUE)) != INVALID_VALUE) min_V = value;
    if((value = ini->geti(section, "max_V", INVALID_VALUE)) != INVALID_VALUE) max_V = value;

    if((value = ini->geti(section, "lowThreshold", INVALID_VALUE)) != INVALID_VALUE) lowThreshold = value;
    if((value = ini->geti(section, "ratio", INVALID_VALUE)) != INVALID_VALUE)        ratio = value;
    if((value = ini->geti(section, "kernel_size", INVALID_VALUE)) != INVALID_VALUE)  kernel_size = value;

    if((value = ini->geti(section, "min_dist", INVALID_VALUE)) != INVALID_VALUE)         min_dist = value;
    if((value = ini->geti(section, "upper_threshold", INVALID_VALUE)) != INVALID_VALUE)  upper_threshold = value;
    if((value = ini->geti(section, "center_threshold", INVALID_VALUE)) != INVALID_VALUE) center_threshold = value;
    if((value = ini->geti(section, "min_r", INVALID_VALUE)) != INVALID_VALUE)            min_r = value;
    if((value = ini->geti(section, "max_r", INVALID_VALUE)) != INVALID_VALUE)            max_r = value;

/*  
    double dvalue = -2.0;
    if((dvalue = ini->getd(section, "min_percent", INVALID_VALUE)) != INVALID_VALUE)    m_min_percent = dvalue;
    if((dvalue = ini->getd(section, "max_percent", INVALID_VALUE)) != INVALID_VALUE)    m_max_percent = dvalue;
*/
    color_section = section;
}

void BallFinder::SaveINISettings(minIni* ini)
{
    SaveINISettings(ini, COLOR_SECTION);
}

void BallFinder::SaveINISettings(minIni* ini, const std::string &section)
{
    ini->put(section,   "min_H", min_H);
    ini->put(section,   "max_H", max_H);
    ini->put(section,   "min_S", min_S);
    ini->put(section,   "max_S", max_S);
    ini->put(section,   "min_V", min_V);
    ini->put(section,   "max_V", max_V);

    ini->put(section,   "lowThreshold", lowThreshold);
    ini->put(section,   "ratio", ratio);
    ini->put(section,   "kernel_size", kernel_size);

    ini->put(section,   "min_dist", min_dist);
    ini->put(section,   "upper_threshold", upper_threshold);
    ini->put(section,   "center_threshold", center_threshold);
    ini->put(section,   "min_r", min_r);
    ini->put(section,   "max_r", max_r);

    color_section = section;
}

void BallFinder::printParam()
{
    //printf("\nmin_H:%d\nmax_H:%d\nmin_S:%d\nmax_S:%d\nmin_V:%d\nmax_V:%d\nlowThreshold:%d\nratio:%d\nkernel_size:%d\nmin_dist:%d\nupper_threshold:%d\ncenter_threshold:%d\nmin_r:%d\nmin_r:%d\n", min_H, max_H, min_S, max_S, min_V, max_V, lowThreshold, ratio, kernel_size, min_dist, upper_threshold, center_threshold, min_r, max_r);
    
    printf("  ══════════════════╦══════════\n\r");
    printf("   Parameter        ║ Nilai       \n\r");
    printf("  ══════════════════╬══════════\n\r");
    printf("   min_H            ║   %d   \n\r", min_H);
    printf("   max_H            ║   %d   \n\r", max_H);
    printf("   min_S            ║   %d   \n\r", min_S);
    printf("   max_S            ║   %d   \n\r", max_S);
    printf("   min_V            ║   %d   \n\r", min_V);
    printf("   max_V            ║   %d   \n\r", max_V);
    printf("  ══════════════════╬══════════\n");
    printf("   lowThreshold     ║   %d   \n\r", lowThreshold);
    printf("   ratio            ║   %d   \n\r", ratio);
    printf("   kernel_size      ║   %d   \n\r", kernel_size);
    printf("  ══════════════════╬══════════\n\r");
    printf("   min_dist         ║   %d   \n\r", min_dist);
    printf("   upper_threshold  ║   %d   \n\r", upper_threshold);
    printf("   center_threshold ║   %d   \n\r", center_threshold);
    printf("   min_r            ║   %d   \n\r", min_r);
    printf("   max_r            ║   %d   \n\r", max_r);
    printf("  ══════════════════╩══════════\n\r");
}
