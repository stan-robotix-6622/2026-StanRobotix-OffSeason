#pragma once

#include <functional>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/DriveTrain.h"

class DriftLeftCommand : public frc2::CommandHelper<frc2::Command, DriftLeftCommand> {
 public:
  /**
   * Commande pour initier un drift vers la gauche.
   * @param driveTrain Pointeur vers le sous-système DriveTrain
   * @param throttleSupplier Fonction fournissant l'intensité du drift (par défaut 1.0)
   */
  explicit DriftLeftCommand(DriveTrain* driveTrain, std::function<double()> throttleSupplier = [] { return 1.0; });

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

 private:
  DriveTrain* mDriveTrain;
  std::function<double()> mThrottleSupplier;
};
