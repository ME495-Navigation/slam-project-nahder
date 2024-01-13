#include "turtlelib/geometry2d.hpp"
#include "turtlelib/svg.hpp"
#include "turtlelib/se2d.hpp"
#include <ostream>
#include <iostream>


int main(void){

    turtlelib::Transform2D T_ab, T_ba, T_bc, T_cb, T_ac, T_ca;

    std::cout << "Enter transform T_{a,b}:" << std::endl;
    std::cin >> T_ab;

    std::cout << "Enter transform T_{b,c}:" << std::endl;
    std::cin >> T_bc;
    
    T_ba = T_ab.inv();
    T_cb = T_bc.inv();
    T_ac = T_ab * T_bc;
    T_ca = T_ac.inv();

    std::cout << "T_{a,b} = " << T_ab << std::endl;
    std::cout << "T_{b,a} = " << T_ba << std::endl;
    std::cout << "T_{b,c} = " << T_bc << std::endl;
    std::cout << "T_{c,b} = " << T_cb << std::endl;
    std::cout << "T_{a,c} = " << T_ac << std::endl;
    std::cout << "T_{c,a} = " << T_ca << std::endl;

    return 0;
    
}