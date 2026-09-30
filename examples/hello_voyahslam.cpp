#include <iostream>
#include "voyahslam/voyahslam.h"

int main() {
    std::cout << "VoyahSLAM v"
              << voyahslam::version_major() << "."
              << voyahslam::version_minor() << std::endl;
    return 0;
}
