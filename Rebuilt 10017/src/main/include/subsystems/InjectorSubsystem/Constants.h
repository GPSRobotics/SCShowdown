#pragma once

#include <units/time.h>
#include <units/angle.h>

namespace InjectorConstants  {

    //Ports
    constexpr int kInjectorPort = 7;

    constexpr int kInjectorDefaultPower = 0.0;
    constexpr double kHoldingPower = -0.2;
    //Injector States
    enum InjectorStates {
        kInjectorOff,
        kInjectorPowerMode
    };
}