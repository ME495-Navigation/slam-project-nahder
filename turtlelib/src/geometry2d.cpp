// implementing the geometry2d.hpp functions

#include "turtlelib/geometry2d.hpp" 
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
        return os << "[" << p.x << " " << p.y << "]";
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

    //subtract two points to get the vector tail to head
    Vector2D operator-(const Point2D & head, const Point2D & tail) {
        return Vector2D{head.x-tail.x, head.y-tail.y};
    }

    //add a point and a vector to displace the point
    Point2D operator+(const Point2D & tail, const Vector2D & disp) {
        return Point2D{tail.x + disp.x, tail.y + disp.y};
    }

    //overload the << operator for putting a vector in the output stream
    std::ostream & operator<<(std::ostream & os, const Vector2D & v) {
        return os << "[" << v.x << " " << v.y << "]";
    }

    //read data from input stream into vector object as [x y] or x y
    std::istream & operator>>(std::istream & is, Vector2D & v) {
        char c;
        is >> c; //read first character
        if (c == '[') { //if we find a bracket, start reading
            is >> v.x >> v.y >> c; //unpack the input stream element by element
        }
        else { 
            is.putback(c); //if no brackets, we found a number: put it back in the stream
            is >> v.x >> v.y;
        }
        return is;
    }
    

} 