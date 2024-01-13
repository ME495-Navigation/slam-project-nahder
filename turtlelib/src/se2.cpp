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


    // Transform the frame of a twist object using the 2D adjoint
    Twist2D Transform2D::operator()(Twist2D v) const {

        double xdot_new{vec.y * v.omega + cos(theta) * v.x - sin(theta) * v.y};
        double ydot_new{-vec.x * v.omega + sin(theta) * v.x + cos(theta) * v.y};

        return Twist2D{v.omega, xdot_new, ydot_new};
    }

    
    // invert the transformation object and return it
    Transform2D Transform2D::inv() const {
        
        Vector2D vec_new {
            -vec.x*cos(theta)-vec.y*sin(theta), 
            -vec.y*cos(theta) + vec.x*sin(theta)
        };

        double theta_new{-theta}; 

        return Transform2D{vec_new, theta_new};

    }

    // friend function: defined outside of class scope, has access to all class members
    // appears in function prototype but are not member functions themselves
    std::ostream & operator<<(std::ostream & os, const Transform2D & tf) {
        return os << "deg: " << rad2deg(tf.theta) << " x: " << tf.vec.x << " y: " << tf.vec.y;
    }

    std::istream & operator>>(std::istream & is, Transform2D & tf) {
        Vector2D v;
        double theta;

        char firstChar = is.peek(); 
        if (firstChar == 'd') {
            std::string tmp1, tmp2, tmp3;
            is >> tmp1; 
            if (tmp1 == "deg:") {
                is >> theta >> tmp2 >> v.x >> tmp3 >> v.y;
            }
        } else {
            is >> theta >> v.x >> v.y;
        }
        tf = Transform2D{v, deg2rad(theta)};
        return is;
    }


    // multiply the rhs transform and update current object's fields 
    Transform2D & Transform2D::operator*=(const Transform2D & rhs) {
        vec.x = vec.x + rhs.vec.x*cos(theta) - rhs.vec.y*sin(theta);
        vec.y = vec.y + rhs.vec.y*cos(theta) + rhs.vec.x*sin(theta); 
        theta += rhs.theta; 
        return *this; 
    }

    /// \brief multiply two transforms together, returning their composition
    /// \param lhs - the left hand operand
    /// \param rhs - the right hand operand
    /// \return the composition of the two transforms
    /// HINT: This function should be implemented in terms of *=
    Transform2D operator*(Transform2D lhs, const Transform2D & rhs) {
        lhs *= rhs;
        return lhs;

    }













    


    
}