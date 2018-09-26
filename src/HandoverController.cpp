#include "HandoverController.h"

#include "head/HeadHandCoordinationEnable.h"
#include "Gripper.h"
#include "Scene.h"
#include "msg_reader/PoseMsgReader.h"
#include "RightArm.h"

HandoverController::HandoverController(bool enable_head_coordination) {
    head_hand_coordination_enable = new HeadHandCoordinationEnable();

    read_poses();

    spinner = new ros::AsyncSpinner(1);
    spinner->start();

    gripper = new Gripper("right");

    Scene scene;

    right_arm = new RightArm;
    right_arm->set_velocity(0.6);
//    right_arm->set_velocity(0.0001);
//    right_arm->set_acceleration(0.33);
//    right_arm->visualize_only = true;

    if (enable_head_coordination) {
        head_hand_coordination_enable->get_ready();
        head_hand_coordination_enable->enable();
    }
    else {
        head_hand_coordination_enable->get_ready();
        head_hand_coordination_enable->disable();
    }
}

void HandoverController::read_poses() {
    std::string path = "/home/zhao/Dropbox/research17fall/experiment/baxter_ws/src/handover_moveit/pose/";

    natural_down_pose = PoseMsgReader::read(path + "natural-down.bag");

    grasp_poses = {
               *PoseMsgReader::read(path + "0-grasp/0-avoid-table.bag"),
               *PoseMsgReader::read(path + "0-grasp/1-almost-ready.bag"),
               *PoseMsgReader::read(path + "0-grasp/2-ready.bag")};

    give_pose = PoseMsgReader::read(path + "1-give/0.bag");

    give_poses = {
            *PoseMsgReader::read(path + "1-give/-1-lift-a-bit.bag"),
            *PoseMsgReader::read(path + "1-give/0.bag")};
}

HandoverController::~HandoverController() {
    delete gripper;
    delete right_arm;
    delete spinner;
    delete head_hand_coordination_enable;
}

HandoverController* HandoverController::grab() {
    ROS_INFO_STREAM("[timing] start grabbing...");
    gripper->open();
    right_arm->follow_trajectory(grasp_poses);
    gripper->grip();

    return this;
}

void HandoverController::give() {
    ROS_INFO_STREAM("[timing] start giving...");
    right_arm->follow_trajectory(give_poses);
//    right_arm->move_to_pose(*give_pose);
}

void HandoverController::back_to_natural_down() const {
    right_arm->move_to_pose(*natural_down_pose);
}

void HandoverController::get_almost_ready() {
    right_arm->move_to_pose(grasp_poses.at(0));
}

bool HandoverController::back_to_ready_to_take_object() {

//    this will make its back trajectory not natural
//    right_arm->move_to_pose(grasp_poses.at(0));

    std::vector<geometry_msgs::Pose> poses = { grasp_poses.at(1), grasp_poses.at(0) };
    bool result = right_arm->follow_trajectory(poses);

    return result;
}

void HandoverController::guarantee_back_to_ready_to_take_object() {
    ROS_INFO("back_to_ready_to_take_object");
    bool result = back_to_ready_to_take_object();
    while ( ! result) {
        ROS_ERROR("back_to_ready_to_take_object failed, wait 50ms and try again...");
        ros::Duration(0.05).sleep(); // sleep 50ms
        result = back_to_ready_to_take_object();
    }
}

void HandoverController::take_object_back_and_get_ready_to_give() {
    auto put_pose = grasp_poses.at(2);
    put_pose.position.z += 0.006;
    right_arm->move_to_pose(put_pose);

    gripper->open();
    right_arm->move_to_pose(grasp_poses.at(0));
}

void HandoverController::test() {
//    gripper->release();
//
//    right_arm->follow_trajectory(grasp_poses);
//    right_arm->move_to_pose(grasp_poses.at(0));
//
//    gripper->grip();
//
    auto put_pose = grasp_poses.at(2);
//    put_pose.position.z += 0.03;
    right_arm->move_to_pose(put_pose);

//    gripper->release();
//    right_arm->move_to_pose(grasp_poses.at(2));
}

void HandoverController::stop() {
    right_arm->stop();
}
