#pragma once

#include <networktables/BooleanTopic.h>
#include <networktables/NetworkTableInstance.h>

// switch on the Tuning tab in elastic. while its off, tunable numbers use the code values and test
// autos dont show up, so nothing changes by accident at a competition. starts off every reboot
namespace TuningMode {

inline bool IsOn() {
  static nt::BooleanEntry entry = [] {
    nt::BooleanEntry newEntry = nt::NetworkTableInstance::GetDefault()
                                    .GetBooleanTopic("/Tuning/Tuning Mode")
                                    .GetEntry(false);
    newEntry.Set(false);
    return newEntry;
  }();
  return entry.Get();
}

}  // namespace TuningMode
