#include <ros/init.h>
#include <ros/node_handle.h>

#include <Gripper.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "zhao_gripper_test");

    Gripper right_gripper("right");
    sleep(4);
    right_gripper.grip();

    ros::spin();
}