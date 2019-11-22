#pragma once
#ifndef Example_MarkerBasedAR_Marker_hpp
#define Example_MarkerBasedAR_Marker_hpp
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include<opencv2/opencv.hpp>  
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/core/eigen.hpp>
#include <zbar.h>
#include <cmath>
#include <chrono>

// for G2o
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/sba/types_six_dof_expmap.h>


using namespace std;
using namespace cv;

class Marker
{
public:
	Marker();
	~Marker();
	friend bool operator<(const Marker &M1, const Marker&M2);
	friend std::ostream & operator<<(std::ostream &str, const Marker &M);
	//
	static cv::Mat rotate(cv::Mat  in);
	static int hammDistMarker(cv::Mat bits);
	static int mat2id(const cv::Mat &bits);
	static int getMarkerId(cv::Mat &in, int &nRotations);
	//
	int id;
	std::vector<cv::Point2f> points;
};

void estimatePosition(std::vector<Marker>& detectedMarkers);
int main_detect(Mat &pImg);

#endif