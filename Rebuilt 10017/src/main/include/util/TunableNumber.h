#pragma once

#include <string>
#include <string_view>

#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTableInstance.h>

#include "util/TuningMode.h"

// number that can be changed live from the Tuning tab in elastic while tuning mode is on. goes back
// to the code value when the robot restarts, so copy good values into the constants files
class TunableNumber {
 public:
  TunableNumber(std::string_view key, double defaultValue)
      : m_entry{nt::NetworkTableInstance::GetDefault()
                    .GetDoubleTopic("/Tuning/" + std::string{key})
                    .GetEntry(defaultValue)},
        m_defaultValue{defaultValue},
        m_lastValue{defaultValue} {
    m_entry.Set(defaultValue);
    TuningMode::IsOn();  // makes sure the tuning mode switch shows up in elastic
  }

  double Get() const { return TuningMode::IsOn() ? m_entry.Get() : m_defaultValue; }

  // true once each time the value changes, including turning tuning mode on or off
  bool HasChanged() {
    double value = Get();
    if (value == m_lastValue) {
      return false;
    }
    m_lastValue = value;
    return true;
  }

 private:
  nt::DoubleEntry m_entry;
  double m_defaultValue;
  double m_lastValue;
};
