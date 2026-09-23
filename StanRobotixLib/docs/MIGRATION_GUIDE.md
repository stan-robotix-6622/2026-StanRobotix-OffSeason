# Guide de Migration

Ce guide présente la transition du code existant de l'équipe vers **StanRobotixLib**, avec des comparaisons avant / après concrètes.

---

## 1. Contrôle Moteur

### Code Antérieur
Chaque sous-système déclarait manuellement les configurations de courant et de mode neutre :

```cpp
// SubRoller.cpp (ancien code)
mRollerMotor = new ctre::phoenix6::hardware::TalonFX{CANid::kRoller};
ctre::phoenix6::configs::TalonFXConfiguration config{};
config.CurrentLimits.SupplyCurrentLimitEnable = true;
config.CurrentLimits.SupplyCurrentLimit = 40_A;
config.CurrentLimits.StatorCurrentLimitEnable = true;
config.CurrentLimits.StatorCurrentLimit = 60_A;
config.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
mRollerMotor->GetConfigurator().Apply(config);
```

### Avec StanRobotixLib
Les sécurités sont appliquées automatiquement à l'initialisation :

```cpp
// Code avec StanRobotixLib
mRoller = new stan::StanMotor{CANid::kRoller, stan::MotorType::kTalonFX};
```

---

## 2. Synchronisation de Position Absolue

### Code Antérieur
L'attente et la recopie de l'encodeur absolu étaient codées manuellement :

```cpp
// SubPivot.cpp (ancien code)
mEncoder->GetPosition().WaitForUpdate(250_ms);
mPivotMotor->SetPosition(mEncoder->GetPosition().GetValue());
```

### Avec StanRobotixLib
L'utilitaire `KrakenSync` gère le timeout et la validation du signal :

```cpp
// Code avec StanRobotixLib
stan::KrakenSync::sync(mPivotMotor->getTalonFX(), mEncoder, 250_ms);
```

---

## 3. Commandes d'Actionneurs

### Code Antérieur
Utilisation de classes de commandes dédiées (`CommandHelper`) :

```cpp
// SetIntakeSpeedCommand.h (ancien code)
class SetIntakeSpeedCommand : public frc2::CommandHelper<frc2::Command, SetIntakeSpeedCommand> {
 public:
  SetIntakeSpeedCommand(SubIntake* iIntake, double iSpeed)
      : mIntake{iIntake}, mSpeed{iSpeed} {
    AddRequirements(mIntake);
  }
  void Initialize() override { mIntake->SetSpeed(mSpeed); }
  void End(bool iInterrupted) override { mIntake->SetSpeed(0.0); }
 private:
  SubIntake* mIntake;
  double mSpeed;
};
```

### Avec StanRobotixLib
Le mécanisme fournit directement des fabriques de `frc2::CommandPtr` :

```cpp
// Dans RobotContainer.cpp
mDriverController.bindHold(mDriverController.a(), mIntake.runRoller(0.8));
```

---

## 4. Châssis Directionnel (CarDrive)

### Code Antérieur
`SubCarDrive` nécessitait plus de 150 lignes pour instancier les moteurs, gérer les calculs d'angles et le pont en H.

### Avec StanRobotixLib
Le sous-système s'instancie directement avec sa structure de configuration :

```cpp
// RobotContainer.h
stan::StanCarDrive mCarDrive{
    CANid::kFLDrive, CANid::kFLSteer, CANid::kFLEncoder,
    CANid::kFRDrive, CANid::kFRSteer, CANid::kFREncoder};

// RobotContainer.cpp
mCarDrive.SetDefaultCommand(mCarDrive.driveCommand(
    [this] { return mDriverController.getRightTrigger(); },
    [this] { return mDriverController.getLeftTrigger(); },
    [this] { return mDriverController.getRightX(); }));
```

---

## 5. Entrées Manette et Deadband

### Code Antérieur
Application manuelle de `frc::ApplyDeadband` sur chaque axe dans les lambdas :

```cpp
// RobotContainer.cpp (ancien code)
double steer = frc::ApplyDeadband(mDriverController.GetRightX(), OperatorConstants::kJoystickDeadband);
```

### Avec StanRobotixLib
`StanXboxController` applique la zone morte automatiquement :

```cpp
// RobotContainer.cpp avec StanRobotixLib
double steer = mDriverController.getRightX();
```
