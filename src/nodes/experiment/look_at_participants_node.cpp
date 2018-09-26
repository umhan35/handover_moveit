#include <future>
#include <ros/init.h>
#include "head/HeadHandCoordinationEnable.h"
#include "head/Head.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "zhao_baxter_handover_look_at_participants_node");

    HeadHandCoordinationEnable head_hand_coordination_enable;

    Head head(true, ros::NodeHandle());

//    HandoverController handover_controller(false);
//    handover_controller.get_almost_ready();

    //  Retrieved by running the following command and take the position part in the pose part
    //  rostopic echo -n 1 /robot/limb/right/endpoint_state
    geometry_msgs::Point participant_face_position;
    participant_face_position.x = 1.2;
    participant_face_position.y = -0.01;
    participant_face_position.z = 0.75;

    ros::Rate r(50);
    while (ros::ok()) {
        head_hand_coordination_enable.disable();
        head.async_look_at(participant_face_position);
        ros::spinOnce();
        if (head.is_done_movement())
            break;
        else
            r.sleep();
    }

    return EXIT_SUCCESS;
}