#pragma once

#include <geometry_msgs/Pose.h>

// forward declaration
namespace ros { class NodeHandle; class AsyncSpinner; }
class HeadHandCoordinationEnable;
class Gripper;
class RightArm;


class HandoverController {

public:
    HandoverController(bool enable_head_coordination = true);
    ~HandoverController();

    HandoverController* grab();
    void give();
    void back_to_natural_down() const;
    void get_almost_ready();
    bool back_to_ready_to_take_object();
    void guarantee_back_to_ready_to_take_object();
    void take_object_back_and_get_ready_to_give();

    void test();

    void stop();

private:
    ros::AsyncSpinner* spinner;

    Gripper* gripper;
    RightArm* right_arm;
    HeadHandCoordinationEnable* head_hand_coordination_enable;

    std::vector<geometry_msgs::Pose> grasp_poses;
    geometry_msgs::Pose::Ptr give_pose;
    std::vector<geometry_msgs::Pose> give_poses;

    void read_poses();

    geometry_msgs::Pose::Ptr natural_down_pose;
};