#pragma once
#include <stdexcept>
#include <iostream>

#include "../../motorDriver/include/adafruitmotorhat.h"
#include "MotorDetails.h"
#include "../../magicNumbers/magicNumbers.h"

class MotorInterface
{
    public:
        virtual ~MotorInterface() = default;
        
        virtual void SetMotorVelocity(const int velocity) = 0;
        virtual void SetMotorInitialVelocity() = 0;
        virtual float Scale(const int velocity) = 0;
        virtual std::unique_ptr<MotorInterface> Instantiate() = 0;
        
        //Couple motor position to correct motor
        virtual std::shared_ptr<AdafruitDCMotor> GetLeftFrontMotor() = 0;
        virtual std::shared_ptr<AdafruitDCMotor> GetRightFrontMotor() = 0;
        virtual std::shared_ptr<AdafruitDCMotor> GetLeftBackMotor() = 0;
        virtual std::shared_ptr<AdafruitDCMotor> GetRightBackMotor() = 0;

        //Movement commands
        virtual void CoupleCommandToMovementInXDirection(const int motorPin, const std::string& position, 
                                                const std::string& positionInput, AdafruitDCMotor::Command commandOne, 
                                                AdafruitDCMotor::Command commandTwo) = 0;

        virtual void CoupleCommandToMovementInZDirection(const int motorPin, const std::string& position, 
                                                const std::string& positionInput, AdafruitDCMotor::Command commandOne, 
                                                AdafruitDCMotor::Command commandTwo) = 0;   

        virtual void SetCommandToMovementInXDirection(const int motorPin, const std::string& position, 
                                                AdafruitDCMotor::Command commandOne, AdafruitDCMotor::Command commandTwo) = 0;

        virtual void SetCommandToMovementInZDirection(const int motorPin, const std::string& position, 
                                                AdafruitDCMotor::Command commandOne, AdafruitDCMotor::Command commandTwo) = 0;  

        virtual void CoupleCommandToMotorRelease(const int motorPin, const std::string& position, 
                                                const std::string& positionInput, AdafruitDCMotor::Command releaseCommand) = 0;   

        virtual void SetCommandToMotorRelease(const int motorPin, const std::string& position, AdafruitDCMotor::Command releaseCommand) = 0; 
        
        

};

