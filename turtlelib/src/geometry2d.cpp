// implementing the geometry2d.hpp functions

#include "turtlelib/geometry2d.hpp" //cmake will look for this file in the include directory
#include <ostream>
#include <iostream>
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

    //operator overloading for putting a point into an output stream
    std::ostream & operator<<(std::ostream & os, const Point2D & p) {
        os << "[" << p.x << " " << p.y << "]";
        return os;
    }
    
    //read data from input stream into point object as [x y] or x y
    std::istream & operator>>(std::istream & is, Point2D & p) {
        char c;
        is >> c; //read first character
        if (c == '[') { //if we find a bracket, start reading
            is >> p.x >> p.y >> c; //unpack the input stream element by element
        }
        else { 
            is.putback(c); //if no brackets, we found a number: put it back in the stream
            is >> p.x >> p.y;
        }
        return is;
    }

    // Vector2D operator-(const Point2D & head, const Point2D & tail) {
    //     ;
    // }

    // Point2D operator+(const Point2D & tail, const Vector2D & disp) {
    //     ;    /// \brief input a 2 dimensional point
    // ///   You should be able to read vectors entered as follows:
    // ///   [x y] or x y
    // /// \param is - stream from which to read
    // /// \param p [out] - output vector
    // /// HINT: See operator>> for Vector2D
    // std::istream & operator>>(std::istream & is, Point2D & p) {
    //     ;
    // }
    // }

    // std::ostream & operator<<(std::ostream & os, const Vector2D & v) {
    //     ;
    // }

    // std::istream & operator>>(std::istream & is, Vector2D & v) {
    //     ;
    // }
} 