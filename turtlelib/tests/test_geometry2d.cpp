#include <catch2/catch_test_macros.hpp>
#include "turtlelib/geometry2d.hpp"
#include <catch2/matchers/catch_matchers_floating_point.hpp>

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
}