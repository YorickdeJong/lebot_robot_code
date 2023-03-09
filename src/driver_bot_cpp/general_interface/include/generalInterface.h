#pragma once
#include <ros/ros.h>
#include <string>
#include <stdlib.h>
#include <sys/wait.h> /* for wait */
#include "terminalCommands.h"

class GeneralInterface
{
    public:
        GeneralInterface();
        void OptionsMenu();
};