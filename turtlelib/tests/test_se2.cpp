#include <catch2/catch_test_macros.hpp>
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <sstream> 
#include <iostream>

namespace turtlelib { 

    TEST_CASE("Output twist se2", "[output<<]") {
        Twist2D tw{20.1, 1.5, 5.1};
        std::ostringstream os;
        os << tw;
        REQUIRE(os.str() == "[20.1 1.5 5.1]");
    }
    
    TEST_CASE("Input twist se2, brackets", "[input>>]") {
        Twist2D tw;
        std::istringstream is("[92.1 -5.5 1.5]");
        is >> tw;
        REQUIRE(tw.omega == 92.1);
        REQUIRE(tw.x == -5.5);
        REQUIRE(tw.y == 1.5);
    }

    TEST_CASE("Input twist se2, no brackets", "[input>>]") {
        Twist2D tw;
        std::istringstream is("1.3 2.1 5.3");
        is >> tw;
        REQUIRE(tw.omega == 1.3);
        REQUIRE(tw.x == 2.1);
        REQUIRE(tw.y == 5.3);
    }

    TEST_CASE("Testing identity constructor", "[constructor]") {
        Transform2D t;
        REQUIRE(t.translation().x == 0.0);
        REQUIRE(t.translation().y == 0.0);
        REQUIRE(t.rotation() == 0.0);
    }

    TEST_CASE("Testing translation constructor", "[constructor]") {
        Transform2D t{Vector2D{1.0, 2.0}};
        REQUIRE(t.translation().x == 1.0);
        REQUIRE(t.translation().y == 2.0);
        REQUIRE(t.rotation() == 0.0);
    }

    TEST_CASE("Testing rotation constructor", "[constructor]") {
        Transform2D t{1.0};
        REQUIRE(t.translation().x == 0.0);
        REQUIRE(t.translation().y == 0.0);
        REQUIRE(t.rotation() == 1.0);
    }

    TEST_CASE("Testing translation and rotation constructor", "[constructor]") {
        Transform2D t{Vector2D{1.0, 2.0}, 1.0};
        REQUIRE(t.translation().x == 1.0);
        REQUIRE(t.translation().y == 2.0);
        REQUIRE(t.rotation() == 1.0);
    }

    TEST_CASE("Testing 2D transform to a point", "[xform]") {
        Point2D p{2.7, 1.2}; 
        Transform2D xform{Vector2D{12.3, 5.0}, PI}; 
        p = xform(p); 
        REQUIRE_THAT(p.x, Catch::Matchers::WithinRel(9.6)); 
        REQUIRE_THAT(p.y, Catch::Matchers::WithinRel(3.8)); 
    }

    TEST_CASE("Testing 2D transform to a vector", "[xform]") {
        //expect translation invariance
        Vector2D v{2.7, 1.2}; 
        Transform2D xform{Vector2D{12.3, 5.0}, PI/2}; 

        v = xform(v); 
        REQUIRE_THAT(v.x, Catch::Matchers::WithinRel(-1.2)); 
        REQUIRE_THAT(v.y, Catch::Matchers::WithinRel(2.7)); 
    }

    TEST_CASE("Testing changing the ref frame of a twist", "[xform]") {
        Twist2D twist{5.0, 2.0, 12.0}; 
        Transform2D xform{Vector2D{10.0, 15.0}, PI/2}; 
        Twist2D twist_new = xform(twist);
        REQUIRE_THAT(twist_new.omega, Catch::Matchers::WithinRel(5.0));
        REQUIRE_THAT(twist_new.x, Catch::Matchers::WithinRel(63.0)); 
        REQUIRE_THAT(twist_new.y, Catch::Matchers::WithinRel(-48.0)); 
    }

    TEST_CASE("Testing inverting a transformation matrix", "[xform]") {
        Transform2D xform{Vector2D{3.0, 5.3}, PI/4}; 
        Transform2D inv_xform = xform.inv();

        REQUIRE_THAT(inv_xform.translation().x, Catch::Matchers::WithinAbs(-5.86899,1e-5));
        REQUIRE_THAT(inv_xform.translation().y, Catch::Matchers::WithinAbs(-1.62634,1e-5));
        REQUIRE(inv_xform.rotation() == -PI/4);
    }

    TEST_CASE("Testing multiplying two transforms", "[xform]") {
        Transform2D xform{Vector2D{3.0, 5.3}, PI/4};    
        Transform2D xform2{Vector2D{10.0, -12.3}, PI/6}; 
        xform *= xform2; 

        REQUIRE_THAT(xform.translation().x, Catch::Matchers::WithinAbs(18.76848, 1e-5));
        REQUIRE_THAT(xform.translation().y, Catch::Matchers::WithinAbs(3.67365, 1e-5));
        REQUIRE(xform.rotation() == (PI/4+PI/6));

    }

    TEST_CASE("Multiplying by inverse transform", "[xform]") {
        Transform2D xform{Vector2D{4.0, -5.0}, PI/2};
        Transform2D inverse = xform.inv();

        xform *= inverse;
        REQUIRE_THAT(xform.translation().x, Catch::Matchers::WithinAbs(0.0, 1e-5));
        REQUIRE_THAT(xform.translation().y, Catch::Matchers::WithinAbs(0.0, 1e-5));
        REQUIRE_THAT(xform.rotation(), Catch::Matchers::WithinAbs(0.0, 1e-5));
    }

    TEST_CASE("Testing multiplying two transforms with rotation and translation", "[xform]") {
        Transform2D xform1{Vector2D{2.0, 3.0}, PI / 4};
        Transform2D xform2{Vector2D{1.0, -1.0}, PI / 6};

        xform1 *= xform2;

        double expected_rot = PI / 4 + PI / 6;

        Vector2D expected_trans = Vector2D{
            2.0 + (cos(PI / 4) * 1.0 - sin(PI / 4) * -1.0),
            3.0 + (sin(PI / 4) * 1.0 + cos(PI / 4) * -1.0)
        };
        REQUIRE_THAT(xform1.translation().x, Catch::Matchers::WithinAbs(expected_trans.x, 1e-5));
        REQUIRE_THAT(xform1.translation().y, Catch::Matchers::WithinAbs(expected_trans.y, 1e-5));
        REQUIRE_THAT(xform1.rotation(), Catch::Matchers::WithinAbs(expected_rot, 1e-5));
    }

    TEST_CASE("Output Transform 2D", "[output<<]") {
        Transform2D xform{Vector2D{2.5, 3.5}, PI};
        std::ostringstream os;
        os << xform;
        REQUIRE(os.str() == "deg: 180 x: 2.5 y: 3.5");
    }

    TEST_CASE("Input Transform2D, multi-line", "[input>>]") {
        Transform2D tf{Vector2D{1.0, 2.0}, PI/2};
        std::istringstream is("60\n2.5\n3.5");
        is >> tf;
        REQUIRE(tf.rotation() == deg2rad(60));
        REQUIRE(tf.translation().x == 2.5);
        REQUIRE(tf.translation().y == 3.5);
    }

    TEST_CASE("Multiplying two transforms", "[xform]") {
        Transform2D xform1{Vector2D{2.0, 3.0}, PI / 4};
        Transform2D xform2{Vector2D{1.0, -1.0}, PI / 6};
        Transform2D xform3 = xform1 * xform2;

        REQUIRE_THAT(xform3.translation().x, Catch::Matchers::WithinAbs(2.0 + (cos(PI / 4) * 1.0 - sin(PI / 4) * -1.0), 1e-5));
        REQUIRE_THAT(xform3.translation().y, Catch::Matchers::WithinAbs(3.0 + (sin(PI / 4) * 1.0 + cos(PI / 4) * -1.0), 1e-5));
        REQUIRE_THAT(xform3.rotation(), Catch::Matchers::WithinAbs(PI / 4 + PI / 6, 1e-5));
    }

    TEST_CASE("Multiplying two transforms 2", "[xform]") {
        Transform2D xform1{Vector2D{-1.5, 12.3}, PI / 8};
        Transform2D xform2{Vector2D{10.3,1.5}, PI / 3};
        Transform2D xform3 = xform1 * xform2;

        REQUIRE_THAT(xform3.translation().x, Catch::Matchers::WithinAbs(7.441934, 1e-5));
        REQUIRE_THAT(xform3.translation().y, Catch::Matchers::WithinAbs(17.627459, 1e-5));
        REQUIRE_THAT(xform3.rotation(), Catch::Matchers::WithinAbs(1.439897, 1e-5));
    }

    TEST_CASE("Multiplying two transforms 3", "[xform]") {
        Transform2D xform1{Vector2D{5.12,-21.3}, PI / 12};
        Transform2D xform2{Vector2D{1.1,2.3}, -PI / 5.3};
        Transform2D xform3 = xform1 * xform2;

        REQUIRE_THAT(xform3.translation().x, Catch::Matchers::WithinAbs(5.587234, 1e-5));
        REQUIRE_THAT(xform3.translation().y, Catch::Matchers::WithinAbs(-18.79366, 1e-5));
        REQUIRE_THAT(xform3.rotation(), Catch::Matchers::WithinAbs(-0.330953, 1e-5));
    }



}
    

 


