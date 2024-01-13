
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
        p = cvt_svg_to_world(tf_svg_to_world, p);

        std::ostringstream svgElement;
        svgElement << "<circle cx=\"" << p.x << "\" cy=\"" << p.y 
                << "\" r=\"" << radius 
                << "\" stroke=\"" << color 
                << "\" fill=\"" << color 
                << "\" stroke-width=\"" << strokeWidth << "\" />\n";
        svgContent += svgElement.str();
    }

    void SVG::addVector(Point2D head, Point2D tail,
                        std::string color, float strokeWidth) {
        head = cvt_svg_to_world(tf_svg_to_world, head);
        tail = cvt_svg_to_world(tf_svg_to_world, tail);

        std::ostringstream svgElement;
        svgElement << "<line x1=\"" << tail.x << "\" y1=\"" << tail.y
                << "\" x2=\"" << head.x << "\" y2=\"" << head.y
                << "\" stroke=\"" << color
                << "\" stroke-width=\"" << strokeWidth
                << "\" marker-start=\"url(#Arrow1Sstart)\" />\n";

        svgContent += svgElement.str();
    }

    void SVG::addCoordinateFrame(Point2D p1, Point2D p2, Point2D p3, Point2D p4,
                                std::string color1, std::string color2, float strokeWidth) {
        std::ostringstream svgElement;
        svgElement << "<g>\n";

        svgContent += svgElement.str();

        addVector(p1, p2, color1, strokeWidth); 
        addVector(p3, p4, color2, strokeWidth); 

        svgContent += "</g>\n";
    }
    

    Point2D SVG::cvt_svg_to_world(const Transform2D &tf, Point2D p) {
        Point2D p_svg = tf(scale_vb_to_world(p));
        
        p_svg.y = vb_height - p_svg.y;
        return p_svg;
    }

    Point2D SVG::scale_vb_to_world(Point2D p) {
        return Point2D{p.x * 96.0, p.y * 96.0};
    }

    void SVG::close_SVG() {
        svgContent += "</svg>";
    }
                    
    void SVG::write_to_file(const std::string& filename) {
        close_SVG();
        std::ofstream file;
        file.open(filename);
        file << svgContent; 
        file.close();    
    }
}