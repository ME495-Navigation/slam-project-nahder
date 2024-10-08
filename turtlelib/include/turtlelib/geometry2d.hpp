#ifndef TURTLELIB_GEOMETRY2D_HPP_INCLUDE_GUARD
#define TURTLELIB_GEOMETRY2D_HPP_INCLUDE_GUARD
/// @file
/// @brief Two-dimensional geometric primitives.

#include <cmath>
#include <iosfwd> // contains forward definitions for iostream objects
namespace turtlelib
{
/// @brief PI.  Not in C++ standard until C++20.
constexpr double PI = 3.14159265358979323846;

/// @brief approximately compare two floating-point numbers using
///        an absolute comparison
/// @param d1 - a number to compare
/// @param d2 - a second number to compare
/// @param epsilon - absolute threshold required for equality
/// @return true if abs(d1 - d2) < epsilon
/// constexpr means that the function can be computed at compile time
/// if given a compile-time constant as input
constexpr bool almost_equal(double d1, double d2, double epsilon = 1.0e-12)
{
  return std::abs(d1 - d2) < epsilon;
}

/// @brief convert degrees to radians
/// @param deg - angle in degrees
/// @returns radians
constexpr double deg2rad(double deg)     //constexpr: computed at compile time, not run time
{
  return deg * PI / 180.0;
}

/// @brief convert radians to degrees
/// @param rad - angle in radians
/// @returns the angle in degrees
constexpr double rad2deg(double rad)
{
  return rad * 180.0 / PI;
}

/// @brief wrap an angle to (-PI, PI]
/// @param rad (angle in radians)
/// @return an angle equivalent to rad but in the range (-PI, PI]

double normalize_angle(double rad);

/// static_assertions test compile time assumptions
static_assert(almost_equal(0, 0), "is_zero failed");
static_assert(almost_equal(0, 1e-13), "is_zero failed");
static_assert(!almost_equal(1, 1.1), "is_zero failed");

static_assert(almost_equal(deg2rad(0.0), 0.0), "deg2rad failed");
static_assert(almost_equal(deg2rad(180.0), PI), "deg2rad failed");
static_assert(almost_equal(deg2rad(270), 3 * PI / 2), "deg2rad failed");
static_assert(almost_equal(deg2rad(rad2deg(2.1)), 2.1), "deg2rad failed");

static_assert(almost_equal(rad2deg(0.0), 0.0), "rad2deg) failed");
static_assert(almost_equal(rad2deg(PI), 180.0), "rad2deg) failed");
static_assert(almost_equal(rad2deg(3 * PI / 2), 270.0), "rad2deg) failed");


/// @brief a 2-Dimensional Point
struct Point2D
{
  /// @brief the x coordinate
  double x = 0.0;

  /// @brief the y coordinate
  double y = 0.0;
};

/// @brief output a 2 dimensional point as [xcomponent ycomponent]
/// @param os - stream to output to
/// @param p - the point to print
std::ostream & operator<<(std::ostream & os, const Point2D & p);

/// @brief input a 2 dimensional point [x y] or x y
/// @param is - stream from which to read
/// @param p [out] - output vector
/// HINT: See operator>> for Vector2D
std::istream & operator>>(std::istream & is, Point2D & p);

/// @brief A 2-Dimensional Vector
struct Vector2D
{
  /// @brief the x coordinate
  double x = 0.0;

  /// @brief the y coordinate
  double y = 0.0;

  /// @brief add a vector to this vector
  /// @param rhs
  /// @return the sum of this vector and rhs
  Vector2D & operator+=(const Vector2D & rhs);

  /// @brief subtract a vector from this vector
  /// @param rhs
  /// @return the difference of this vector and rhs
  Vector2D & operator-=(const Vector2D & rhs);


  /// @brief multiply this vector by a scalar
  /// @param rhs
  /// @return the product of this vector and rhs
  Vector2D & operator*=(const double & rhs);

};

/// @brief add two vectors
/// @param lhs - the left hand vector
/// @param rhs - the right hand vector
Vector2D operator+(Vector2D lhs, const Vector2D & rhs);

/// @brief subtract two vectors
/// @param lhs - the left hand vector
/// @param rhs - the right hand vector
Vector2D operator-(Vector2D lhs, const Vector2D & rhs);

/// @brief multiply a vector by a scalar
/// @param lhs - the left hand vector
/// @param rhs - the right hand scalar
Vector2D operator*(Vector2D lhs, const double & rhs);

/// @brief multiply a vector by a scalar
/// @param lhs - the left hand operand
/// @param rhs - the right hand operand
Vector2D operator*(const double & lhs, Vector2D rhs);

/// @brief dot product of two vectors
/// @param v1 - the first vector
/// @param v2 - the second vector
/// @return the dot product of v1 and v2
double dot(Vector2D v1, Vector2D v2);

/// @brief magnitude of a vector
/// @param v - the vector
/// @return the magnitude of v
double magnitude(Vector2D v);

/// @brief angle between two vectors
/// @param v1 - the first vector
/// @param v2 - the second vector
/// @return the angle between v1 and v2
double angle(Vector2D v1, Vector2D v2);

/// @brief a 2-Dimensional Vector
/// @param v - the vector to normalize
/// @return a normalized vector
Vector2D normalize_vector(Vector2D v);

/// @brief Subtracting one point from another yields a vector
/// @param head point corresponding to the head of the vector
/// @param tail point corresponding to the tail of the vector
/// @return a vector that points from p1 to p2
/// NOTE: this is not implemented in terms of -= because subtracting two Point2D yields a Vector2D
Vector2D operator-(const Point2D & head, const Point2D & tail);

/// @brief Adding a vector to a point yields a new point displaced by the vector
/// @param tail The origin of the vector's tail
/// @param disp The displacement vector
/// @return the point reached by displacing by disp from tail
/// NOTE: this is not implemented in terms of += because of the different types
Point2D operator+(const Point2D & tail, const Vector2D & disp);

/// @brief output a 2 dimensional vector as [xcomponent ycomponent]
/// @param os - stream to output to
/// @param v - the vector to print
std::ostream & operator<<(std::ostream & os, const Vector2D & v);

/// @brief input a 2 dimensional vector [x y] or x y
/// @param is - stream from which to read
/// @param v [out] - output vector
std::istream & operator>>(std::istream & is, Vector2D & v);

}

#endif
