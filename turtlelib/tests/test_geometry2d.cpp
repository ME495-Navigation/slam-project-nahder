#include <catch2/catch_test_macros.hpp>
#include "turtlelib/geometry2d.hpp"
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <sstream> 
#include <iostream>

//https://github.com/catchorg/Catch2/blob/devel/docs/tutorial.md#top
//https://github.com/catchorg/Catch2/blob/devel/docs/comparing-floating-point-numbers.md

namespace turtlelib { 
    bool is_within_range(double angle) {
        return angle >= -PI && angle <= PI;
    }

    TEST_CASE("Normalize angle for PI", "[normalize]") {
        double normalized_angle = normalize_angle(PI);
        REQUIRE_THAT(normalized_angle, Catch::Matchers::WithinRel(PI)); // Changed to expect PI
        REQUIRE(is_within_range(normalized_angle));
    }

    TEST_CASE("Normalize a vector", "[normalize]") {
        Vector2D v{3.0, 4.0};
        Vector2D vhat = normalize_vector(v);
        REQUIRE_THAT(vhat.x, Catch::Matchers::WithinRel(0.6));
        REQUIRE_THAT(vhat.y, Catch::Matchers::WithinRel(0.8));
        REQUIRE_THAT(vhat.x*vhat.x + vhat.y*vhat.y, Catch::Matchers::WithinRel(1.0));
    }

    TEST_CASE("Normalize angle for -PI", "[normalize]") {
        double normalized_angle = normalize_angle(-PI);
        REQUIRE_THAT(normalized_angle, Catch::Matchers::WithinRel(PI));
        REQUIRE(is_within_range(normalized_angle));
    }

    TEST_CASE("Normalize angle for 0", "[normalize]") {
        double normalized_angle = normalize_angle(0.0);
        REQUIRE_THAT(normalized_angle, Catch::Matchers::WithinRel(0.0));
        REQUIRE(is_within_range(normalized_angle));
    }

    TEST_CASE("Normalize angle for -PI/4", "[normalize]") {
        double normalized_angle = normalize_angle(-PI / 4);
        REQUIRE_THAT(normalized_angle, Catch::Matchers::WithinRel(-PI / 4));
        REQUIRE(is_within_range(normalized_angle));
    }

    TEST_CASE("Normalize angle for 3PI/2", "[normalize]") {
        double normalized_angle = normalize_angle(3 * PI / 2);
        REQUIRE_THAT(normalized_angle, Catch::Matchers::WithinRel(-PI / 2)); 
        REQUIRE(is_within_range(normalized_angle));
    }

    TEST_CASE("Normalize angle for -5PI/2", "[normalize]") {
        double normalized_angle = normalize_angle(-5 * PI / 2);
        REQUIRE_THAT(normalized_angle, Catch::Matchers::WithinRel(-PI / 2)); 
        REQUIRE(is_within_range(normalized_angle));
    }

    TEST_CASE("Output point 2D", "[output<<]") {
        Point2D p{5.1, -2.5};
        std::ostringstream os;
        os << p;
        REQUIRE(os.str() == "[5.1 -2.5]");
    }
    
    TEST_CASE("Input point 2D, brackets", "[input>>]") {
        Point2D p;
        std::istringstream is("[1.2 3.4]");
        is >> p;
        REQUIRE(p.x == 1.2);
        REQUIRE(p.y == 3.4);
    }

    TEST_CASE("Input point 2D, no brackets", "[input>>]") {
        Point2D p; 
        std::istringstream is("1.2 3.4");
        is >> p;
        REQUIRE(p.x == 1.2);
        REQUIRE(p.y == 3.4); 
    }

    TEST_CASE("Subtracting two points", "[vector-]") {
        Point2D p1{1.2, 5.6}; 
        Point2D p2{0.5, 0.2};
        Vector2D v = p1-p2;
        REQUIRE_THAT(v.x, Catch::Matchers::WithinRel(0.7)); 
        REQUIRE_THAT(v.y, Catch::Matchers::WithinRel(5.4)); 
    }

    TEST_CASE("Adding a point and a vector", "[vector+]") {
        Point2D p{9.3, -2.5}; 
        Vector2D v{15,18};
        p = p + v; 
        REQUIRE_THAT(p.x, Catch::Matchers::WithinRel(24.3)); 
        REQUIRE_THAT(p.y, Catch::Matchers::WithinRel(15.5)); 
    }

    TEST_CASE("Output vector 2D", "[output<<]") {
        Vector2D v{10.5, -19.4};
        std::ostringstream os;
        os << v;
        REQUIRE(os.str() == "[10.5 -19.4]");
    }

    TEST_CASE("Input vector 2D, brackets", "[input>>]") {
        Vector2D v;
        std::istringstream is("[21.5 -31.5]");
        is >> v;
        REQUIRE(v.x == 21.5);
        REQUIRE(v.y == -31.5);
    }

    TEST_CASE("Input vector 2D, no brackets", "[input>>]") {
        Vector2D v;
        std::istringstream is("5.3 2.8");
        is >> v;
        REQUIRE(v.x == 5.3);
        REQUIRE(v.y == 2.8);
    }

    TEST_CASE("Add two vectors", "[vector+]") {
        Vector2D v1{3.2, 2.6};
        Vector2D v2{2.8, 3.4};
        Vector2D v = v1 + v2;
        REQUIRE_THAT(v.x, Catch::Matchers::WithinRel(6.0)); // 3.2 + 2.8
        REQUIRE_THAT(v.y, Catch::Matchers::WithinRel(6.0)); // 2.6 + 3.4
    }

    TEST_CASE("Add a vector to another vector", "[vector+=]") {
        Vector2D v1{5.1, 1.1};
        Vector2D v2{0.9, 4.4};
        v1 += v2;
        REQUIRE_THAT(v1.x, Catch::Matchers::WithinRel(6.0)); // 5.1 + 0.9
        REQUIRE_THAT(v1.y, Catch::Matchers::WithinRel(5.5)); // 1.1 + 4.4
    }

    TEST_CASE("Subtract a vector from another vector", "[vector-=]") {
        Vector2D v1{5.6, 3.7};
        Vector2D v2{1.7, 4.2};
        v1 -= v2;
        REQUIRE_THAT(v1.x, Catch::Matchers::WithinRel(3.9)); // 5.6 - 1.7
        REQUIRE_THAT(v1.y, Catch::Matchers::WithinRel(-0.5)); // 3.7 - 4.2
    }

    TEST_CASE("Subtract two vectors", "[vector-]") {
        Vector2D v1{6.3, 5.2};
        Vector2D v2{3.1, 2.0};
        Vector2D v = v1 - v2;
        REQUIRE_THAT(v.x, Catch::Matchers::WithinRel(3.2)); // 6.3 - 3.1
        REQUIRE_THAT(v.y, Catch::Matchers::WithinRel(3.2)); // 5.2 - 2.0
    }

    TEST_CASE("Dot product of two vectors", "[vector_dot]") {
        Vector2D v1{3, 4};
        Vector2D v2{1, 2};
        double result = dot(v1, v2);
        REQUIRE(result == 11); // 3*1 + 4*2
    }

    TEST_CASE("Angle between two vectors", "[vector_angle]") {
        Vector2D v1{1, 0};
        Vector2D v2{0, 1};
        double result = angle(v1, v2);
        REQUIRE_THAT(result, Catch::Matchers::WithinAbs(1.5708, 1e-4)); // 90 degrees in radians
    }

    TEST_CASE("Multiplying a vector and a scalar", "[vector*]") {
        Vector2D v{5, 7};
        double scalar{3};
        Vector2D result = v * scalar;
        REQUIRE_THAT(result.x, Catch::Matchers::WithinRel(15.0)); // 5 * 3
        REQUIRE_THAT(result.y, Catch::Matchers::WithinRel(21.0)); // 7 * 3
    }

    TEST_CASE("Multiplying a vector by a scalar on itself", "[vector*=]") {
        Vector2D v{2, 6};
        double scalar{4};
        v *= scalar;
        REQUIRE_THAT(v.x, Catch::Matchers::WithinRel(8.0)); // 2 * 4
        REQUIRE_THAT(v.y, Catch::Matchers::WithinRel(24.0)); // 6 * 4
    }

    // Testing scalar * vector multiplication
    TEST_CASE("Multiplying a scalar and a vector", "[scalar*vector]") {
        Vector2D v{4, 5};
        double scalar{2};
        Vector2D result = scalar * v;
        REQUIRE_THAT(result.x, Catch::Matchers::WithinRel(8.0)); // 2 * 4
        REQUIRE_THAT(result.y, Catch::Matchers::WithinRel(10.0)); // 2 * 5
    }

    // Testing vector magnitude
    TEST_CASE("Magnitude of a vector", "[vector_magnitude]") {
        Vector2D v{3, 4};
        double mag = magnitude(v);
        REQUIRE_THAT(mag, Catch::Matchers::WithinRel(5.0)); // sqrt(3^2 + 4^2)
    }

    // Testing dot product
    TEST_CASE("Dot product of perpendicular vectors", "[vector_dot_perpendicular]") {
        Vector2D v1{1, 0};
        Vector2D v2{0, 1};
        double dot_product = dot(v1, v2);
        REQUIRE(dot_product == 0); // Dot product of perpendicular vectors is 0
    }

    // Testing angle between vectors
    TEST_CASE("Angle between unit vectors along axes", "[vector_angle_unit]") {
        Vector2D v1{1, 0};
        Vector2D v2{0, 1};
        double angle_radians = angle(v1, v2);
        REQUIRE_THAT(angle_radians, Catch::Matchers::WithinAbs(M_PI / 2, 1e-4)); // 90 degrees in radians
    }

    // Testing normalization of a vector
    TEST_CASE("Normalization of a vector", "[vector_normalize]") {
        Vector2D v{4, 3};
        Vector2D normalized_v = normalize_vector(v);
        double mag = magnitude(normalized_v);
        REQUIRE_THAT(mag, Catch::Matchers::WithinAbs(1.0, 1e-4)); // Magnitude of a normalized vector should be 1
    }

    // Testing subtraction of a vector from another vector
    TEST_CASE("Subtracting vectors", "[vector-]") {
        Vector2D v1{7, 3};
        Vector2D v2{2, 1};
        Vector2D result = v1 - v2;
        REQUIRE_THAT(result.x, Catch::Matchers::WithinRel(5.0)); // 7 - 2
        REQUIRE_THAT(result.y, Catch::Matchers::WithinRel(2.0)); // 3 - 1
    }



}