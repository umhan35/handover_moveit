#include <ros/console.h>
#include "head/HeadState.h"

void HeadState::set_pan(float p) {
    pan = p;
}

bool HeadState::is_pan_angle(float target) {
    double min = pan - 0.07;
    double max = pan + 0.07;
//    ROS_INFO_STREAM("pan diff: " << target - state->pan << " (" << target << " - " << state->pan << ")");
    bool result = target >= min && target <= max;
    return result;
}