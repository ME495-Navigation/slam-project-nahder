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

}