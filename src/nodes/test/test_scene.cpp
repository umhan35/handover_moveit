#include <ros/init.h>
#include "Scene.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_scene");
    Scene scene;
    return EXIT_SUCCESS;
}