#include "head/FaceImage.h"
#include "boost/shared_ptr.hpp"
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char **argv) {
    FaceImage::Ptr face = FaceImage::create("/home/zhao/Dropbox/research17fall/experiment/baxter_ws/src/handover_moveit/face");

    using namespace cv;
    namedWindow("", cv::WINDOW_AUTOSIZE);
    imshow("", face->with_eyeballs_vertically(-10));
    waitKey();

    return 0;
}