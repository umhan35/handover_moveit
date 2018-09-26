#include <ros/init.h>
#include "HandoverController.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "zhao_baxter_handover_ready_to_give");
//    sleep(5);
    HandoverController().take_object_back_and_get_ready_to_give();
    return EXIT_SUCCESS;
}