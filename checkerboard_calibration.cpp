//
// Created by mc on 7/1/18.
//

#include "checkerboard_calibration.hpp"
void Checkerboard_Calibration::load_file_list(){
    ifstream file_list("../calibration_file_list.txt");
    if (file_list.is_open()){
        string temp;
        while (file_list >> temp){
            image_list.push_back(temp);
        }
        cout << "OK:    loaded calibration file list" << endl;
    }else{
        cout << "Error: cannot load file list." << endl;
    }
}

Mat Checkerboard_Calibration::get_camera_matrix(){
    return camera_matrix.clone();
}
Mat Checkerboard_Calibration::get_distort_coefficient(){
    return distort_coefficient.clone();
}

vector<string> Checkerboard_Calibration::get_file_list(){
    return this->image_list;
}

Mat Checkerboard_Calibration::get_undistort_image(int index){
    auto view = imread(image_list[0]);
    auto imageSize = view.size();

    Mat rview, map1, map2;

    initUndistortRectifyMap(
            this->camera_matrix, this->distort_coefficient, Mat(),
            getOptimalNewCameraMatrix(this->camera_matrix, this->distort_coefficient, imageSize, 1, imageSize, 0), imageSize,
            CV_16SC2, map1, map2);

    remap(view, rview, map1, map2, INTER_LINEAR);
    namedWindow(image_list[0], WINDOW_KEEPRATIO);
    imshow(image_list[0], rview);
}

void Checkerboard_Calibration::set_checkerboard_size(int row, int column){
    this->checkerboard_size = Size2i(row, column);
}

void Checkerboard_Calibration::set_square_size(float size){
    this->square_size = size;
}

void Checkerboard_Calibration::calculate_board_corner_positions(vector<Point3f> &corners){
    corners.clear();

    for( int i = 0; i < this->checkerboard_size.height; ++i ) {
        for (int j = 0; j < this->checkerboard_size.width; ++j) {
            corners.emplace_back(Point3f(j * this->square_size, i * this->square_size, 0));
        }
    }
}


double Checkerboard_Calibration::compute_reprojection_errors(const vector<vector<Point3f> >& objectPoints, const vector<Mat>& rvecs, const vector<Mat>& tvecs){
    vector<Point2f> imagePoints2;
    size_t totalPoints = 0;
    double totalErr = 0, err;
    reprojection_errors.resize(objectPoints.size());

    for(size_t i = 0; i < objectPoints.size(); ++i )
    {
        projectPoints(objectPoints[i], rvecs[i], tvecs[i], this->camera_matrix, this->distort_coefficient, imagePoints2);
        err = norm(this->image_points[i], imagePoints2, NORM_L2);

        size_t n = objectPoints[i].size();
        reprojection_errors[i] = (float) sqrt((long double)err*err/n);
        totalErr        += err*err;
        totalPoints     += n;
    }

    return (double)sqrt((long double)totalErr/totalPoints);
}

bool Checkerboard_Calibration::core_calibration(Size imageSize){

    this->camera_matrix = Mat::eye(3, 3, CV_64F);
    this->distort_coefficient = Mat::zeros(8, 1, CV_64F);

    vector<vector<Point3f> > object_points(1);
    calculate_board_corner_positions(object_points[0]);
    object_points.resize(this->image_points.size(), object_points[0]);

    vector<Mat> rvecs, tvecs;
    auto rms = calibrateCamera(object_points, this->image_points, imageSize, this->camera_matrix, this->distort_coefficient, rvecs, tvecs, 0);
    cout << "INFO:  Re-projection error reported by calibrateCamera " << rms << endl;
    bool ok = checkRange(this->camera_matrix) && checkRange(this->distort_coefficient);

    this->total_average_error = compute_reprojection_errors(object_points, rvecs, tvecs);

    return ok;
}

bool Checkerboard_Calibration::calibrate_single_image(string filename, Mat * origin, Mat * marked){
    *origin = imread(filename);

    vector<Point2f> point_buffer;
    if (!findChessboardCorners(*origin, this->checkerboard_size, point_buffer, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE)){
        cout << "Error: cannot find checkerboard!" << endl;
        return false;
    }

    *marked = origin->clone();
    drawChessboardCorners(*marked, this->checkerboard_size, Mat(point_buffer), true);

    this->image_points.push_back(point_buffer);
    return core_calibration(origin->size());
}

void Checkerboard_Calibration::run_calibration(){
    for (const auto & item: this->image_list){
        cout << "FILE:  " << item << endl;
        Mat origin_image, marked_image;
        if (calibrate_single_image(item, &origin_image, &marked_image)){
            cout << "OK:    success!" << endl;
        }else{
            cout << "Error: cannot calibrate this image!" << endl;
        }
    }
}


Checkerboard_Calibration::Checkerboard_Calibration(string file_list){
    this->checkerboard_size = Size2i(9,7);
    this->square_size = 10.0;
    this->total_average_error = 0;
    this->image_list.clear();
    load_file_list();
}