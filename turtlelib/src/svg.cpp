
#include "turtlelib/se2d.hpp"
#include "turtlelib/geometry2d.hpp" 
#include "turtlelib/svg.hpp"
#include <ostream>
#include <sstream> 
#include <iostream>
#include <cmath> 
#include <fstream>
namespace turtlelib { 

    //define constructor member functions, uniform initialization
    SVG::SVG() {
        std::ostringstream svgHeader;
        svgHeader << "<svg width=\"8.500000in\" height=\"11.000000in\" viewBox=\"0 0 816.000000 1056.000000\" xmlns=\"http://www.w3.org/2000/svg\">\n";
        svgHeader << "<defs>\n";
        svgHeader << "  <marker style=\"overflow:visible\" id=\"Arrow1Sstart\" refX=\"0.0\" refY=\"0.0\" orient=\"auto\">\n";
        svgHeader << "       <path transform=\"scale(0.2) translate(6,0)\" style=\"fill-rule:evenodd;fill:context-stroke;stroke:context-stroke;stroke-width:1.0pt\" d=\"M 0.0,0.0 L 5.0,-5.0 L -12.5,0.0 L 5.0,5.0 L 0.0,0.0 z \"/>\n";
        svgHeader << "    </marker>\n";
        svgHeader << "</defs>\n";
        svgContent = svgHeader.str();
    }
    

    void SVG::addPoint(Point2D p, std::string color, float strokeWidth, float radius) {
        
        p = cvt_world_to_svg(tf_world_to_svg, p);

        std::ostringstream svgElement;
        svgElement << "<circle cx=\"" << p.x << "\" cy=\"" << p.y 
                << "\" r=\"" << radius 
                << "\" stroke=\"" << color 
                << "\" fill=\"" << color 
                << "\" stroke-width=\"" << strokeWidth << "\" />\n";

        svgContent += svgElement.str();
    }

    Point2D SVG::cvt_world_to_svg(const Transform2D &tf, Point2D p) {
        Point2D p_svg = tf(p);
        p_svg.y *= -1;
        return p_svg;
    }

    void SVG::close_SVG() {
        svgContent += "</svg>";
    }

    // void SVG::addVector(Point2D head, Point2D tail,
    //                 std::string color = "black", float strokeWidth = 5.0) {
                    
    
    // }
                    
    void SVG::write_to_file(const std::string& filename) {
        close_SVG();
        std::ofstream file;
        file.open(filename);
        file << svgContent; 
        file.close();    }
}