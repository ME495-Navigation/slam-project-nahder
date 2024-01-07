// implementing the geometry2d.hpp functions

#include "turtlelib/geometry2d.hpp" //cmake will look for this file in the include directory

namespace turtlelib {

    double normalize_angle(double rad) {
        // modulus the angle with 2*pi
        rad = std::fmod(rad, 2 * PI);

        // adjust if angle is in (-2*pi, -pi]
        if (rad <= -PI) {
            rad += 2 * PI;
        }
        // adjust if angle is in (pi, 2*pi)
        else if (rad > PI) {
            rad -= 2 * PI;
        }

        return rad;
    }

} 