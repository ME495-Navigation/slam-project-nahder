#include "turtlelib/se2d.hpp"
#include "turtlelib/geometry2d.hpp" 
#include <ostream>
#include <iostream>
#include <cmath> 

namespace turtlelib { 

    //output twist as [w x y]
    std::ostream & operator<<(std::ostream & os, const Twist2D & tw) {
        return os << "[" << tw.omega << " " << tw.x << " " << tw.y << "]";
    }

    //read input stream into twist object
    std::istream & operator>>(std::istream & is, Twist2D & tw) {
        char c;
        is >> c; //read first character
        if (c == '[') { //if we find a bracket, start reading
            is >> tw.omega >> tw.x >> tw.y >> c;
        }
        else { 
            is.putback(c); //if no brackets, we found a number: put it back in the stream
            is >> tw.omega >> tw.x >> tw.y;
        }
        return is;
    }

    //Transform2D constructor member functions, uniform initialization

    //identity transformation
    Transform2D::Transform2D() {} 

    //pure translation
    Transform2D::Transform2D(Vector2D t): vec{Vector2D{t.x,t.y}}, theta{0.0} {}

    //pure rotation
    Transform2D::Transform2D(double rot): vec{Vector2D{0.0,0.0}}, theta{rot} {} 

    //rotation and translation
    Transform2D::Transform2D(Vector2D t, double rot): vec{Vector2D{t.x, t.y}}, theta{rot} {}
    
    //getter method for the translation
    Vector2D Transform2D::translation() const { //const guarantees no member variables will be changed
        return vec;
    }

    //getter method for the rotation
    double Transform2D::rotation() const { 
        return theta;
    }

    //this function lets us pass in a point to a Transform2D object and get the transformed point back
    Point2D Transform2D::operator()(Point2D p) const {

        double x_new{p.x*cos(theta) - p.y*sin(theta) + vec.x};
        double y_new{p.x*sin(theta) + p.y*cos(theta) + vec.y};

        return Point2D{x_new, y_new};
    }


    //vectors are invariant under translation
    Vector2D Transform2D::operator()(Vector2D v) const {

        double x_new{v.x * cos(theta) - v.y * sin(theta)};
        double y_new{v.x * sin(theta) + v.y * cos(theta)};

        return Vector2D{x_new, y_new};
    }


    
}