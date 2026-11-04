#pragma once

#include "lemlib/api.hpp"

inline pros::Controller controller(pros::E_CONTROLLER_MASTER);

inline pros::MotorGroup leftMotors({10, 20}, pros::MotorGearset::blue);
inline pros::MotorGroup rightMotors({11, 1}, pros::MotorGearset::blue);

inline lemlib::Drivetrain drivetrain(
    &leftMotors,
    &rightMotors,
    13.5,
    lemlib::Omniwheel::NEW_275,
    600,
    2
);

inline lemlib::ControllerSettings lateralController(
    10, 0, 3, 3, 1, 100, 3, 500, 20
);
inline lemlib::ControllerSettings angularController(
    2, 0, 10, 3, 1, 100, 3, 500, 0
);

inline pros::Imu imu(9);
inline pros::Rotation odomEncoder(18);
inline lemlib::TrackingWheel odomWheel(
    &odomEncoder,
    lemlib::Omniwheel::NEW_275,
    -5.75
);

inline lemlib::OdomSensors sensors(
    &odomWheel,
    nullptr,
    nullptr,
    nullptr,
    &imu
);

inline lemlib::Chassis chassis(
    drivetrain,
    lateralController,
    angularController,
    sensors
);
