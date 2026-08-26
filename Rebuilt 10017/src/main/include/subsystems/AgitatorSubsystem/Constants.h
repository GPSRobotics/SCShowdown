#pragma once

#include <units/time.h>
#include <units/angle.h>

namespace AgitatorConstants  {

    //Ports
    constexpr int kAgitatorPort = 8;

    constexpr int kAgitatorDefaultPower = 0.3;
    constexpr double kHoldingPower = -0.2;
    //Agitator States
    enum AgitatorStates {
        kAgitatorOff,
        kAgitatorPowerMode
    };
}