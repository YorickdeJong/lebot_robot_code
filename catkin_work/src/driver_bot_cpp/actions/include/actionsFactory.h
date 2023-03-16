#pragma once 
#include "actionsInterface.h"
#include "actions2Wheels.h"
#include "actions4Wheels.h"


class ActionsFactory
{
    public:
        ActionsFactory();
        ~ActionsFactory() = default;
        
        std::unique_ptr<ActionsInterface> Create(MotorType type);

    private:
        std::unordered_map<MotorType, std::unique_ptr<ActionsInterface>, std::hash<int>> m_actions;
};

