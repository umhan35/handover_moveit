#include <ros/init.h>
#include "HandoverController.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "zhao_baxter_handover_ready_to_give");
//    sleep(5);
    HandoverController().back_to_ready_to_take_object();
    return EXIT_SUCCESS;
}