#include "motorFactory.h"


MotorFactory::MotorFactory()
{
    /*Instantiates motor types*/
    m_motor[FOURWHEELS] = std::make_unique<Motor4Wheels>();
    // m_motor[TWOWHEELS] = std::make_unique<Motor2Wheels>();
}

std::unique_ptr<MotorInterface> MotorFactory::Create(MotorType type)
{
    /*Returns correct motor type polymorphically*/
    return m_motor[type] -> Instantiate();
}

