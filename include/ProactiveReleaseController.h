#include "ReleaseController.h"
#include "DataStream.h"

class ProactiveReleaseController : public ReleaseController {
public:
    explicit ProactiveReleaseController(double voltage_difference_to_release)
            : force_data(90, 180), ReleaseController(voltage_difference_to_release) {
        gripper_force_subscriber.shutdown();
        this->set_up_gripper_force_subscriber(nh);
    }

private:
    DataStream force_data;

    void set_up_gripper_force_subscriber(ros::NodeHandle &nh) override {
        gripper_force_subscriber = nh.subscribe("/zhao/baxter_gripper_force/1ms", 1, &ProactiveReleaseController::check_if_release, this);
    }

    void check_if_release(const std_msgs::Float64::ConstPtr &state) override {
//        ROS_INFO_STREAM("check_if_release: " << state->data);
        force_data << state->data;
        if (is_right_gripper_far_enough() && force_data.decreasing_percentage(0.35, 40)) {
            release();
        }
    }
};