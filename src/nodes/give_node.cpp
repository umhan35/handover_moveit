#include <ros/init.h>
#include "HandoverController.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "zhao_baxter_handover_give");
//    sleep(5);
//    HandoverController();
//    HandoverController().grab()->give();
    HandoverController().grab();
    return EXIT_SUCCESS;
}