#pragma once
#include <unordered_map>
#include "motorInterface.h"
#include "motor2Wheels.h"
#include "motor4Wheels.h"



class MotorFactory
{
    public:
        MotorFactory();
        ~MotorFactory() = default;
        
        std::unique_ptr<MotorInterface> Create(MotorType type);

    private:
        std::unordered_map<MotorType, std::unique_ptr<MotorInterface>, std::hash<int>> m_motor;
};



