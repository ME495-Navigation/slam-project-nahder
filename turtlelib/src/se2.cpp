#include "turtlelib/se2d.hpp"
#include "turtlelib/geometry2d.hpp"
#include <ostream>
#include <iostream>
#include <cmath>

namespace turtlelib
{
    // output twist as [w x y]
    std::ostream &operator<<(std::ostream &os, const Twist2D &tw)
    {
        return os << "[" << tw.omega << " " << tw.x << " " << tw.y << "]";
    }

    // read input stream into twist object
    std::istream &operator>>(std::istream &is, Twist2D &tw)
    {
        char c;
        is >> c; 
        if (c == '[')
        { 
            is >> tw.omega >> tw.x >> tw.y >> c;
        }
        else
        {
            is.putback(c); 
            is >> tw.omega >> tw.x >> tw.y;
        }
        return is;
    }

    // Transform2D constructor member functions, uniform initialization
    // identity transformation
    Transform2D::Transform2D() {}

    // pure translation
    Transform2D::Transform2D(Vector2D t)
        : vec{Vector2D{t.x, t.y}}, theta{0.0} {}

    // pure rotation
    Transform2D::Transform2D(double rot)
        : vec{Vector2D{0.0, 0.0}}, theta{rot} {}

    // rotation and translation
    Transform2D::Transform2D(Vector2D t, double rot)
        : vec{Vector2D{t.x, t.y}}, theta{rot} {}

    // getter method for the translation
    Vector2D Transform2D::translation() const 
    {
        return vec;
    }

    // getter method for the rotation
    double Transform2D::rotation() const
    {
        return theta;
    }

    // this function lets us pass in a point to a Transform2D object and get the transformed point back
    Point2D Transform2D::operator()(Point2D p) const
    {

        double x_new{p.x * cos(theta) - p.y * sin(theta) + vec.x};
        double y_new{p.x * sin(theta) + p.y * cos(theta) + vec.y};

        return Point2D{x_new, y_new};
    }

    // vectors are invariant under translation
    Vector2D Transform2D::operator()(Vector2D v) const
    {

        double x_new{v.x * cos(theta) - v.y * sin(theta)};
        double y_new{v.x * sin(theta) + v.y * cos(theta)};

        return Vector2D{x_new, y_new};
    }

    // Transform the frame of a twist object using the 2D adjoint
    Twist2D Transform2D::operator()(Twist2D v) const
    {

        double xdot_new{vec.y * v.omega + cos(theta) * v.x - sin(theta) * v.y};
        double ydot_new{-vec.x * v.omega + sin(theta) * v.x + cos(theta) * v.y};

        return Twist2D{v.omega, xdot_new, ydot_new};
    }

    // invert the transformation object and return it
    Transform2D Transform2D::inv() const
    {

        Vector2D vec_new{
            -vec.x * cos(theta) - vec.y * sin(theta),
            -vec.y * cos(theta) + vec.x * sin(theta)};

        double theta_new{-theta};

        return Transform2D{vec_new, theta_new};
    }

    // friend function: defined outside of class scope, has access to all class members
    // appears in function prototype but are not member functions themselves
    std::ostream &operator<<(std::ostream &os, const Transform2D &tf)
    {
        return os << "deg: " << rad2deg(tf.theta) << " x: " << tf.vec.x << " y: " << tf.vec.y;
    }

    std::istream &operator>>(std::istream &is, Transform2D &tf)
    {
        Vector2D v;
        double theta;

        char firstChar = is.peek();
        if (firstChar == 'd')
        {
            std::string tmp1, tmp2, tmp3;
            is >> tmp1;
            if (tmp1 == "deg:")
            {
                is >> theta >> tmp2 >> v.x >> tmp3 >> v.y;
            }
        }
        else
        {
            is >> theta >> v.x >> v.y;
        }
        tf = Transform2D{v, deg2rad(theta)};
        return is;
    }

    // multiply the rhs transform and update current object's fields
    Transform2D &Transform2D::operator*=(const Transform2D &rhs)
    {
        vec.x = vec.x + rhs.vec.x * cos(theta) - rhs.vec.y * sin(theta);
        vec.y = vec.y + rhs.vec.y * cos(theta) + rhs.vec.x * sin(theta);
        theta += rhs.theta;
        return *this;
    }

    // multiply two transforms and return the result
    Transform2D operator*(Transform2D lhs, const Transform2D &rhs)
    {
        lhs *= rhs;
        return lhs;
    }

    // compute the transformation corresponding to a rigid body following a constant twist (in its
    // body frame) for 1 time unit. 
    // using 1 time unit lets us integrate for arbitrary time lengths by scaling 
    // twist: omega, x, y ( velocities in the body frame)
    Transform2D integrate_twist(const Twist2D & tw) 
    {
        if (almost_equal(tw.omega, 0.0)) { //zero angular displacement => pure translation
            return Transform2D{Vector2D{tw.x, tw.y}, 0.0};

        } else {
            //find the center of rotation in {s}, aligned with {b} frame
            auto x_s = tw.y / tw.omega;
            auto y_s = -tw.x / tw.omega;

            //transform from {b} -> {s} 
            Transform2D T_sb{Vector2D{x_s, y_s}}; 

            //perform the rotation in the new frame {s'}
            Transform2D T_s_sp{tw.omega}; 

            //T_s'b' = T_sb since T_bs and T_b's' are pure translations from p to the body frame
            return T_sb.inv() * T_s_sp * T_sb;
        }
    }


}
