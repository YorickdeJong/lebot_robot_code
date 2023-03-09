#pragma once
#include <string>

enum MotorType
{
    FOURWHEELS,
    TWOWHEELS
};

/*Add position of motor with corresponding pin*/
//Used in Motor classes
static std::string LOCATION_MOTOR_1 = "RB";
static int PIN_MOTOR_1 = 1;

static std::string LOCATION_MOTOR_2 = "RF";
static int PIN_MOTOR_2 = 2;

static std::string LOCATION_MOTOR_3 = "LF";
static int PIN_MOTOR_3 = 3;

static std::string LOCATION_MOTOR_4 = "LB";
static int PIN_MOTOR_4 = 4;

static int MINIMUM_RANGE_MOTOR = 0;
static int MAXIMUM_RANGE_MOTOR = 255;
static int NORMALISED_RANGE = 100;

//Used in actions classes
static MotorType MOTOR_TYPE = FOURWHEELS; 