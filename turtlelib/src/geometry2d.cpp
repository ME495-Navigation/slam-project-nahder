// implementing the geometry2d.hpp functions

#include "turtlelib/geometry2d.hpp" //cmake will look for this file in the include directory

namespace turtlelib {

    // wraps an angle to (-PI, PI]
    double normalize_angle(double rad) {
        rad = std::fmod(rad, 2 * PI);

        if (rad <= -PI) {
            rad += 2 * PI;
        }
        else if (rad > PI) {
            rad -= 2 * PI;
        }
        return rad;
    }

    std::ostream & operator<<(std::ostream & os, const Point2D & p) {
        ;
    }
    
    std::istream & operator>>(std::istream & is, Point2D & p) {
        ;
    }

    Vector2D operator-(const Point2D & head, const Point2D & tail) {
        ;
    }

    Point2D operator+(const Point2D & tail, const Vector2D & disp) {
        ;
    }

    std::ostream & operator<<(std::ostream & os, const Vector2D & v) {
        ;
    }

    std::istream & operator>>(std::istream & is, Vector2D & v) {
        ;
    }
} 