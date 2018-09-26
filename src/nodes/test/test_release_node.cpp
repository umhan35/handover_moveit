#include <cstdlib>
#include <ros/init.h>
#include "ProactiveReleaseController.h"

int main(int argc, char **argv) {
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    ros::init(argc, argv, "zhao_baxter_handover_release");

    ProactiveReleaseController *releaseController = new ProactiveReleaseController(ReleaseController::PULL_VOLTAGE_DIFF);
    releaseController->async_spin();

    delete releaseController;

    ros::spin();

    return EXIT_SUCCESS;
}