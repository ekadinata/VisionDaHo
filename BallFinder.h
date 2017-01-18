// Soccer Vision 
// Author: Andreas Ekadinata Widodo <ekadinataa@gmail.com>

/**
* KRSBI ITB
* Dago Hoogeschool Team
**/



#ifndef BallFinder_H
#define BallFinder_H

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <stdio.h>

#include <string>

#include "Point.h"
#include "minIni.h"

#define COLOR_SECTION   "Find Ball"
#define INVALID_VALUE   -1024.0

#define max_val 255
#define max_lowThreshold 500

using namespace cv;

namespace Robot {

	class BallFinder
	{
		private:
			static BallFinder* m_UniqueInstance;
			bool flag;

			Mat img;
			Mat hsv;
			Mat th;
			Mat gray;
			Mat edge;

			static void on_trackbar(int value, void *userdata);
			void CreateTrackbar();
			void morphOps(Mat &thresh);
			void isDetectBall(Point2D center);
			

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

			int min_dist;
			int upper_threshold;
			int center_threshold;
			int min_r;
			int max_r;

			BallFinder();
			BallFinder(int min_H, int max_H, int min_S, int max_S, int min_V, int max_V, int lowThreshold, int ratio, int kernel_size, int min_dist, int upper_threshold, int center_threshold, int min_r, int max_r);
			virtual ~BallFinder();

			static BallFinder* GetInstance() { return m_UniqueInstance; }

			Point2D pos;
			void printParam();
			void Process(Mat image);
			void ControlPanel(minIni* ini);
			Point2D& getPosition();

			void LoadINISettings(minIni* ini);
        	void LoadINISettings(minIni* ini, const std::string &section);
        	void SaveINISettings(minIni* ini);
        	void SaveINISettings(minIni* ini, const std::string &section);


	};
}

#endif