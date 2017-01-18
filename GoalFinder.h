// Soccer Vision 
// Author: Andreas Ekadinata Widodo <ekadinataa@gmail.com>

/**
* KRSBI ITB
* Dago Hoogeschool Team
**/



#ifndef GoalFinder_H
#define GoalFinder_H

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <stdio.h>

#include <string>

#include "Point.h"
#include "minIni.h"


#define COLOR_SECTION   "Find Goal"
#define INVALID_VALUE   -1024.0
#define PI 3.14159265


#define max_val 255
#define max_lowThreshold 500

using namespace cv;

namespace Robot {
	class GoalFinder
	{
		private:
			static GoalFinder* m_UniqueInstance;
			bool flag;

			Mat img;
			Mat hsv;
			Mat th;
			Mat edge;

			static void on_trackbar(int value, void *userdata);
			void CreateTrackbar();
			void morphOps(Mat &thresh);
			

		public:

			std::string color_section;

			int min_H;
			int max_H;
			int min_S;
			int max_S;
			int min_V;
			int max_V;

			int lowThreshold;
			int ratio;
			int kernel_size;

			int thLine;
			int minLinLength;
			int maxLineGap;

			int r_mean_h ;
		    int r_mean_v ;
		    int r_mean_right ;
		    int r_mean_left ;
		    
		    float teta_mean_h ;
		    float teta_mean_v ;
		    float teta_mean_left;
		    float teta_mean_right;

		    int variansi_v;


			GoalFinder();
			GoalFinder(int min_H, int max_H, int min_S, int max_S, int min_V, int max_V, int lowThreshold, int ratio, int kernel_size, int thLine, int minLinLength, int maxLineGap);
			virtual ~GoalFinder();

			static GoalFinder* GetInstance() { return m_UniqueInstance; }

			vector<Vec2f> lines;
			vector<Vec2f> horizontal;
			vector<Vec2f> vertical;
			vector<Vec2f> right_post;
			vector<Vec2f> left_post;

			float rho;
			float theta;
        	Point pt1, pt2;
        	//double a = cos(theta), b = sin(theta);
        	//double x0 = a*rho, y0 = b*rho;

			//Corner
			Point2D TR;
			Point2D TL;
			Point2D BR;
			Point2D BL;
			
			void printParam();
			void Process(Mat image);
			void ControlPanel(minIni* ini);
			void getCorner(Mat src);
			Point CalculateIntersection(int R1, float Teta1, int R2, float Teta2);
			
			//Belum ditest
			void ClassifyLineHV(float r, float t);
			void CalculateMeanH();
			void CalculateMeanV();
			void CalculateVarianceV();
			void ClassifyLineRL();
			void CalculateMeanRL();
			/////////////////////////

			float DegreesToRadians(float degrees);
			float RadiansToDegrees(float radians);

			void LoadINISettings(minIni* ini);
        	void LoadINISettings(minIni* ini, const std::string &section);
        	void SaveINISettings(minIni* ini);
        	void SaveINISettings(minIni* ini, const std::string &section);


	};
}
#endif