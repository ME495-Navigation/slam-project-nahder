#include <catch2/catch_test_macros.hpp>
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/svg.hpp"
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <sstream> 
#include <iostream>
#include <fstream>

namespace turtlelib { 
TEST_CASE("SVG File Creation Test", "[SVG]") {
    SVG svg;

    svg.addPoint(Point2D{0.0, 0.0}, "red", 1.0, 1.0);
    svg.addPoint(Point2D{1.0, 0.0}, "green", 1.0, 1.0);
    svg.addPoint(Point2D{0.0, 1.0}, "blue", 1.0, 1.0);
    svg.addPoint(Point2D{1.0, 1.0}, "black", 1.0, 1.0);

    svg.addVector(Point2D{0.0, 0.0}, Point2D{1.0, 0.0}, "red", 5.0);
    svg.addVector(Point2D{0.0, 0.0}, Point2D{0.0, 1.0}, "purple", 5.0);

    svg.addCoordinateFrame(Point2D{0.0,0.0}, Point2D{1.0,0.0}, 
                            Point2D{0.0,0.0}, Point2D{0.0,1.0}, "test");

    std::string filename = "test_output.svg";
    svg.write_to_file(filename);

    std::ifstream file(filename);
    REQUIRE(file.good() == true); 
    }


}