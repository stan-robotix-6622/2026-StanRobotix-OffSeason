#pragma once

#include <optional>
#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>
#include "RobotContainer.h"

class Robot : public frc::TimedRobot {
 public:
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void TeleopInit() override;

 private:
  std::optional<frc2::CommandPtr> mAutonomousCommand;
  RobotContainer mContainer;
};
