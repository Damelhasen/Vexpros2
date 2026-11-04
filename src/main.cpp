#include "main.h"
#include "lemlib/api.hpp"

// left motor group
pros::MotorGroup left_motor_group({1, 11}, pros::MotorGears::blue);
// right motor group
pros::MotorGroup right_motor_group({-20, -10}, pros::MotorGears::blue);

pros::Motor claw (2, pros::MotorGears::green, pros::MotorUnits::degrees);
pros::MotorGroup lift({3, 4}, pros::MotorGears::blue, pros::MotorUnits::degrees);
// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motor_group, // left motor group
                              &right_motor_group, // right motor group
                              10, // 10 inch track width
                              lemlib::Omniwheel::NEW_4, // using new 4" omnis
                              360, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);

// imu
pros::Imu imu(9);
// vertical tracking wheel encoder: measures forward/backward (Y) movement
pros::Rotation odomEncoder(18);
// Keep each wheel's offset matched to its physical mounting position.
lemlib::TrackingWheel odomWheel(&odomEncoder, lemlib::Omniwheel::NEW_275, -5.75);

// odometry settings
lemlib::OdomSensors sensors(&odomWheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2
                            nullptr, // no horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);

pros::Controller controller(pros::E_CONTROLLER_MASTER);

// initialize function. Runs on program startup
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // DEBUGGING: Check raw sensor readings if the chassis pose is NaN.
            pros::lcd::print(3, "IMU: %.2f", imu.get_heading());
            pros::lcd::print(4, "Rotation: %d", odomEncoder.get_position());
            // delay to save resources
            pros::delay(20);
        }
    });
}

void opcontrol() {
    // loop forever
    while (true) {
         // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // delay to save resources
            pros::delay(20);
        // left stick Y controls forward/backward; right stick X controls turning
        int throttle = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        
        // move the robot
        chassis.arcade(-throttle, -turn);
        
        
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
        //claw stuff
            claw.move(100);

        // delay to save resources
        pros::delay(25);

        
    }

    
    
    
 
}
}