#pragma once
#include "motorInterface.h"

class Motor2Wheels : public MotorInterface
{
    public:
        Motor2Wheels();
        ~Motor2Wheels() = default;
        
        void SetMotorVelocity(const int velocity) override;
        void SetMotorInitialVelocity() override;
        float Scale(const int velocity) override;
        std::unique_ptr<MotorInterface> Instantiate() override;

        //Couple motor position to correct motor
        std::shared_ptr<AdafruitDCMotor> GetLeftFrontMotor() override;
        std::shared_ptr<AdafruitDCMotor> GetRightFrontMotor() override;
        std::shared_ptr<AdafruitDCMotor> GetLeftBackMotor() override;
        std::shared_ptr<AdafruitDCMotor> GetRightBackMotor() override;

        void CoupleCommandToMovementInXDirection(const int motorPin, const std::string& position, 
                                                const std::string& positionInput, AdafruitDCMotor::Command commandOne, 
                                                AdafruitDCMotor::Command commandTwo) override;

        void CoupleCommandToMovementInZDirection(const int motorPin, const std::string& position, 
                                                const std::string& positionInput, AdafruitDCMotor::Command commandOne, 
                                                AdafruitDCMotor::Command commandTwo) override;   

        void SetCommandToMovementInXDirection(const int motorPin, const std::string& position, 
                                                AdafruitDCMotor::Command commandOne, AdafruitDCMotor::Command commandTwo) override;

        void SetCommandToMovementInZDirection(const int motorPin, const std::string& position, 
                                                AdafruitDCMotor::Command commandOne, AdafruitDCMotor::Command commandTwo) override;  

        void CoupleCommandToMotorRelease(const int motorPin, const std::string& position, 
                                                const std::string& positionInput, AdafruitDCMotor::Command releaseCommand) override;   

        void SetCommandToMotorRelease(const int motorPin, const std::string& position, AdafruitDCMotor::Command releaseCommand) override; 
        
        
        
    private:
        void SetMotorsToPosition();
        void CoupleMotorPosition(const int motorPin, const std::string& position); 
        void CoupleMotorToInitialVelocity(const std::string& position); 
        void CoupleMotorToVelocity(const int velocity, const std::string& position);
        
    private:
        AdafruitMotorHAT m_hat; // connect using the default device address 0x60

        //L = left, R = right, F = front,  B = Back
        std::shared_ptr<AdafruitDCMotor> m_motorLF; 
        std::shared_ptr<AdafruitDCMotor> m_motorRF; 
        std::shared_ptr<AdafruitDCMotor> m_motorLB; 
        std::shared_ptr<AdafruitDCMotor> m_motorRB; 

};



