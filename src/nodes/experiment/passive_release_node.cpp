#include <ros/init.h>
#include "HandoverController.h"
#include "ReleaseController.h"


int main(int argc, char **argv) {
    ros::init(argc, argv, "zhao_baxter_handover_passive_release");

    // prepare
    HandoverController handoverController;
    ReleaseController releaseController(ReleaseController::PULL_VOLTAGE_DIFF);
    std::function<void ()> on_release = [&] {
        handoverController.stop();
        handoverController.guarantee_back_to_ready_to_take_object();
        ros::shutdown();
    };

    // actually doing stuff
    handoverController.grab();
    releaseController.async_spin(on_release);
    handoverController.give();

    ros::spin();

    return EXIT_SUCCESS;
}