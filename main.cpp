#include <iostream>
#include "checkerboard_calibration.hpp"

int main() {
    std::cout << "Hello, World!" << std::endl;
    Checkerboard_Calibration cc("../calibration_file_list.txt");
    cc.set_checkerboard_size(9, 6);
    cc.set_square_size(2.6);
    cc.run_calibration();
    cc.get_undistort_image(0);



    return 0;
}