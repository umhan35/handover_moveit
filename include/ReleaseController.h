#pragma once

#include <ros/subscriber.h>
#include <std_msgs/Float64.h>
#include <ros/callback_queue.h>
#include <ros/spinner.h>
#include <baxter_core_msgs/EndpointState.h>
#include <ros/node_handle.h>

class Gripper;

class ReleaseController {
public:
    static constexpr double PULL_VOLTAGE_DIFF = 0.03;

    explicit ReleaseController(double voltage_difference_to_release, double gripper_x_detection = 0.8);
    ~ReleaseController();

    void async_spin();
    void async_spin(std::function<void ()> &on_release_callback);

protected:
    ros::NodeHandle nh;
    ros::Subscriber gripper_force_subscriber;

    void release();

    bool is_right_gripper_far_enough() const;

private:
    double voltage_difference_to_release;
    double gripper_x_detection;

    ros::CallbackQueue callback_queue;
    ros::AsyncSpinner async_spinner;

    double last_force_value = 0;
    Gripper* gripper;

    virtual void set_up_gripper_force_subscriber(ros::NodeHandle &nh);
    virtual void check_if_release(const std_msgs::Float64::ConstPtr &state);
    void check_if_release(double last_value, double new_value);
    void on_release();

    std::function<void()> release_callback;


    ros::Subscriber endpoint_state_subscriber;
    void on_endpoint_state(const baxter_core_msgs::EndpointState& state);
    double right_gripper_x = 0;
};
