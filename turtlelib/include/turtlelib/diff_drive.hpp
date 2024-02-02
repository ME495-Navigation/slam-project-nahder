

#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"

namespace turtlelib { 

    //robot configuration
    struct robotConfig { 
        double x{0.0};
        double y{0.0};
        double theta{0.0};
    };

    //wheel position
    struct wheelConfig {
        double left_wheel_pos{0.0};
        double right_wheel_pos{0.0};
    };

    //wheel velocities
    struct wheelVel {
        double left_wheel_vel{0.0};
        double right_wheel_vel{0.0};
    };

    class DiffDrive {

    public:
        DiffDrive(double radius, double width);

        //update the current configuration
        void setRobotState(const robotConfig state);
        
        //fk: determine the POSITION of the robot based on the known wheel motion
        //update the current configuration
        void forwardKinematics(const wheelConfig new_wheel_config);


        //ik: compute the wheel velocities required to achieve a desired twist
        wheelVel inverseKinematics(const Twist2D twist) const;

    private:
        robotConfig cur_config;
        wheelConfig cur_wheel_config;
        wheelVel cur_wheel_vel;
        
        double wheel_radius{0.0};
        double track_width{0.0};
    };

}