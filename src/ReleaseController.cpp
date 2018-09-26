#include "ReleaseController.h"
#include "Gripper.h"

ReleaseController::ReleaseController(double voltage_difference_to_release, double gripper_x_detection)
        : voltage_difference_to_release(voltage_difference_to_release),
          async_spinner(1, &callback_queue),
          gripper_x_detection(gripper_x_detection) {
    nh.setCallbackQueue(&callback_queue);

    gripper = new Gripper("right");

    endpoint_state_subscriber = nh.subscribe("/robot/limb/right/endpoint_state", 1, &ReleaseController::on_endpoint_state, this);
    this->set_up_gripper_force_subscriber(nh);
}

void ReleaseController::set_up_gripper_force_subscriber(ros::NodeHandle &nh) {
    gripper_force_subscriber = nh.subscribe("/zhao/baxter_gripper_force/256ms", 1, &ReleaseController::check_if_release, this);
}

ReleaseController::~ReleaseController() {
    delete gripper;
}

void ReleaseController::check_if_release(const std_msgs::Float64::ConstPtr &state) {
    ROS_INFO_STREAM(state->data);

    if (! gripper->is_open() && right_gripper_x > 0.8 && state->data == 0) {
        ROS_INFO("releasing!!!! object already pulled!!!");
        release();
        return;
    }

    if ( ! is_right_gripper_far_enough()) {
        ROS_INFO_STREAM("[ NO release check ] right_gripper_x < " << this->gripper_x_detection);
        return;
    }
    else {
        ROS_INFO_STREAM("right_gripper_x = " << right_gripper_x);
    }

    if (last_force_value == 0) {
        // last value was never set or 0
        ROS_INFO_STREAM("[set last] last value was never set or 0");
        last_force_value = state->data;
    }
    else {
        check_if_release(last_force_value, state->data);
        last_force_value = state->data;
    }
}

bool ReleaseController::is_right_gripper_far_enough() const {
    return right_gripper_x > this->gripper_x_detection;
}

void ReleaseController::check_if_release(double last_value, double new_value) {
    double diff = fabs(new_value - last_value);
    bool is_stable = diff <= voltage_difference_to_release;
    ROS_INFO_STREAM(diff << "(" << new_value << " - " << last_value << ")");
    if ( ! is_stable) {
        release();
    }
}

void ReleaseController::release() {
    ROS_INFO_STREAM("[timing] RELEASING");
    gripper->release();
    on_release();
}

void ReleaseController::on_release() {
    gripper_force_subscriber.shutdown();
    if (release_callback) {
        release_callback();
    }
}

void ReleaseController::async_spin(std::function<void ()> &on_release_callback) {
    release_callback = on_release_callback;
    async_spin();
}

void ReleaseController::async_spin() {
    async_spinner.start();
}

void ReleaseController::on_endpoint_state(const baxter_core_msgs::EndpointState &state) {
    right_gripper_x = state.pose.position.x;
}
