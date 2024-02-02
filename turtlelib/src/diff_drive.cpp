#include "turtlelib/diff_drive.hpp" // Assume your header is named diff_drive.hpp
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"

namespace turtlelib {


    DiffDrive::DiffDrive(double radius, double width)
        : wheel_radius(radius), track_width(width) {
    }

    void DiffDrive::setRobotState(const robotConfig state) {
        this->cur_config = state;
    }

    void DiffDrive::forwardKinematics(const wheelConfig new_wheel_config) {

    }

    wheelVel DiffDrive::inverseKinematics(const Twist2D twist) const {

        return wheelVel{}; 
    }

}
