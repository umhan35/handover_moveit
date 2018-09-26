#include <ros/init.h>
#include "HandoverController.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_handover");
    HandoverController().test();
    return EXIT_SUCCESS;
}