#include "RobotContainer.h"

#include <cmath>
#include <frc/MathUtil.h>
#include <frc2/command/Commands.h>
#include "commands/DriftLeftCommand.h"
#include "commands/DriftRightCommand.h"
#include "commands/DriveTeleopCommand.h"

RobotContainer::RobotContainer() {
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  // Commande par défaut pour se déplacer (Avancer, Reculer, Tourner, Freiner)
  mDriveTrain.SetDefaultCommand(DriveTeleopCommand(
      &mDriveTrain,
      [this] {
        // Accélérateur : Trigger droit ou stick gauche Y vers l'avant
        double trigger = mDriverController.GetRightTriggerAxis();
        if (trigger > OperatorConstants::kJoystickDeadband) {
          return trigger;
        }
        double stickY = -mDriverController.GetLeftY();
        return stickY > OperatorConstants::kJoystickDeadband ? stickY : 0.0;
      },
      [this] {
        // Frein : Trigger gauche
        return mDriverController.GetLeftTriggerAxis();
      },
      [this] {
        // Direction : Stick gauche X
        return frc::ApplyDeadband(mDriverController.GetLeftX(), OperatorConstants::kJoystickDeadband);
      },
      [this] {
        // Marche arrière : Bouton B ou stick gauche Y vers l'arrière
        if (mDriverController.GetHID().GetBButton()) {
          return true;
        }
        return (-mDriverController.GetLeftY()) < -OperatorConstants::kJoystickDeadband;
      },
      [this] {
        // Drift dynamique activable en cours de conduite via le stick ou bumper
        return mDriverController.LeftBumper().Get() || mDriverController.RightBumper().Get();
      }
  ));

  // Fonction fournissant la puissance du drift
  auto driftThrottleSupplier = [this] {
    double trigger = mDriverController.GetRightTriggerAxis();
    return trigger > 0.1 ? trigger : 1.0;
  };

  // Commande 1 : Drift à gauche (maintenir LB)
  mDriverController.LeftBumper().WhileTrue(
      DriftLeftCommand(&mDriveTrain, driftThrottleSupplier).ToPtr()
  );

  // Commande 2 : Drift à droite (maintenir RB)
  mDriverController.RightBumper().WhileTrue(
      DriftRightCommand(&mDriveTrain, driftThrottleSupplier).ToPtr()
  );

  // Zérotage des modules Steer BL et BR
  mDriverController.POVLeft().OnTrue(mDriveTrain.GetZeroBLCommand());
  mDriverController.POVRight().OnTrue(mDriveTrain.GetZeroBRCommand());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::None();
}
