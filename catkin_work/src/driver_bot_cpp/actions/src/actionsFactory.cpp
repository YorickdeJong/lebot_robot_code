#include "actionsFactory.h"


ActionsFactory::ActionsFactory()
{
    /*Instantiates action types*/
    // m_actions[TWOWHEELS] = std::make_unique<Actions2Wheels>();
    m_actions[FOURWHEELS] = std::make_unique<Actions4Wheels>();

}

std::unique_ptr<ActionsInterface> ActionsFactory::Create(MotorType type)
{
    std::cout << "Actions INTERFACE TYPE: " << type << "Constructed" << std::endl;
    /*Returns correct action type polymorphically*/
    return m_actions[type] -> Instantiate();
}

