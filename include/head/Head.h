#pragma once

#include <geometry_msgs/Point.h>
#include "HeadPan.h"
#include "Screen.h"
#include "FaceImage.h"

class Head {

public:
    Head(bool enabled, ros::NodeHandle nh);

    void set_enable(bool enabled);

    void async_look_at(geometry_msgs::Point position);

    bool is_done_movement();

private:
    static const double CALIBRATED_PAN_DIFF;

    HeadPan head_pan;

    inline double calculate_eyeball_vertical_angle(double x, double z);
    inline double calculate_pan_radius(const double &x, const double &y);

    ros::Publisher eyeball_publisher;
};