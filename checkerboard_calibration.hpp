//
// Created by mc on 7/1/18.
//

#ifndef CAMERA_CALIBRATION_CHECKERBOARD_CALIBRATION_HPP
#define CAMERA_CALIBRATION_CHECKERBOARD_CALIBRATION_HPP


#include <iostream>
#include <fstream>

#include <cmath>
#include <tuple>

#include <string>
#include <vector>
#include <list>
#include <thread>
#include <algorithm>
#include <set>
#include <queue>
#include <map>

#include <cv.hpp>
#include <cxcore.hpp>
#include <highgui.h>


using namespace std;
using namespace cv;

class Checkerboard_Calibration {
private:
    vector<string> image_list;
    vector<vector<Point2f> > image_points;
    vector<float> reprojection_errors;
    Mat camera_matrix;
    Mat distort_coefficient;

    double total_average_error;
    float square_size;
    Size2i checkerboard_size;

    void calculate_board_corner_positions(vector<Point3f> &corners);
    double compute_reprojection_errors( const vector<vector<Point3f> >& objectPoints, const vector<Mat>& rvecs, const vector<Mat>& tvecs);
    bool core_calibration(Size imageSize);



    void load_file_list();
public:
    Mat get_camera_matrix();
    Mat get_distort_coefficient();
    vector<string> get_file_list();
    Mat get_undistort_image(int index);

    bool calibrate_single_image(string filename, Mat * origin, Mat * marked);
    void run_calibration();

    void set_checkerboard_size(int row, int column);
    void set_square_size(float size);


    explicit Checkerboard_Calibration(string file_list);
};


#endif //CAMERA_CALIBRATION_CHECKERBOARD_CALIBRATION_HPP
