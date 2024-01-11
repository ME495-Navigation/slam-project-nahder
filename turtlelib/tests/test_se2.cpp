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


    
    
}
    

 


