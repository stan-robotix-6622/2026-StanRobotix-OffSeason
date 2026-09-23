#pragma once

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <units/time.h>

namespace stan {

class KrakenSync {
 public:
  static bool sync(ctre::phoenix6::hardware::TalonFX* iMotor,
                   ctre::phoenix6::hardware::CANcoder* iEncoder,
                   units::time::millisecond_t iTimeout = 250_ms) {
    if (iMotor == nullptr || iEncoder == nullptr) {
      return false;
    }
    auto& posSignal = iEncoder->GetPosition();
    posSignal.WaitForUpdate(iTimeout);
    if (posSignal.GetStatus().IsOK()) {
      auto status = iMotor->SetPosition(posSignal.GetValue());
      return status.IsOK();
    }
    return false;
  }

  static bool sync(ctre::phoenix6::hardware::TalonFX& iMotor,
                   ctre::phoenix6::hardware::CANcoder& iEncoder,
                   units::time::millisecond_t iTimeout = 250_ms) {
    return sync(&iMotor, &iEncoder, iTimeout);
  }
};

} // namespace stan
