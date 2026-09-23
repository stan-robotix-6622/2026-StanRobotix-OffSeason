# Mécanismes & Sous-Systèmes

Ce document présente les sous-systèmes fournis par **StanRobotixLib**, leurs paramètres de configuration et leurs modèles d'utilisation.

---

## 1. StanRoller

`StanRoller` gère les mécanismes rotatifs continus sans asservissement angulaire précis (rouleaux d'admission, convoyeurs, indexeurs).

### 1.1 Fonctionnalités
- Commande en rapport cyclique (de -1.0 à 1.0) ou en tension explicite (`units::voltage::volt_t`).
- Mesure en temps réel du courant statorique (`getCurrent()`) pour la détection d'absorption d'éléments de jeu.
- Fabrique de commande `runRoller(double iSpeed)`.

### 1.2 Exemple
```cpp
#include <stan/StanRoller.h>
#include <units/voltage.h>

stan::StanRoller intakeRoller{CANid::kIntake, stan::MotorType::kTalonFX};

// Commande directe en tension
intakeRoller.setVoltage(10.0_V);

// Liaison manette : maintien actif sur gâchette
mDriverController.bindHold(
    mDriverController.rightTrigger(),
    intakeRoller.runRoller(0.8));
```

---

## 2. StanFlywheel

`StanFlywheel` pilote les volants d'inertie nécessitant une régulation de vitesse en boucle fermée à **1 kHz** avec détection de seuil de tir.

### 2.1 Configuration (`FlywheelConfig`)
| Paramètre | Type | Description |
|---|---|---|
| `kMaxVelocity` | `turns_per_second_t` | Vitesse maximale nominale du mécanisme. |
| `kTolerance` | `turns_per_second_t` | Écart admissible pour valider la consigne (`atDesiredVelocity()`). |
| `kP`, `kI`, `kD` | `double` | Coefficients du régulateur embarqué. |
| `kS`, `kV` | `double` | Coefficients de friction statique et de vitesse. |

### 2.2 Exemple
```cpp
#include <stan/StanFlywheel.h>

stan::FlywheelConfig flywheelConfig{
    .kMaxVelocity = 90_tps,
    .kTolerance = 1.5_tps,
    .kP = 0.12,
    .kI = 0.0,
    .kD = 0.005,
    .kS = 0.06,
    .kV = 0.11};

stan::StanFlywheel shooterFlywheel{CANid::kShooter, stan::MotorType::kTalonFX, flywheelConfig};

// Séquence de tir autonome
frc2::CommandPtr shootSequence = frc2::cmd::Sequence(
    shooterFlywheel.spinVelocity(75_tps),
    shooterFlywheel.waitUntilReady(),
    intakeRoller.runRoller(1.0).WithTimeout(0.5_s),
    shooterFlywheel.stopCommand(),
    intakeRoller.runRoller(0.0));
```

---

## 3. StanPivot

`StanPivot` pilote les articulations angulaires (bras, poignets) sujettes à la gravité, avec synchronisation absolue par `CANcoder`.

### 3.1 Fonctionnalités
- Régulation de position à 1 kHz sur le capteur interne du moteur.
- Synchronisation absolue au démarrage via `KrakenSync`.
- Feedforward gravitationnel (`Arm_Cosine` via `kG`).
- Butées logicielles (*soft limits*) appliquées au contrôleur.

### 3.2 Exemple
```cpp
#include <stan/StanPivot.h>

stan::PivotConfig armConfig{
    .kMinAngle = -10_deg,
    .kMaxAngle = 110_deg,
    .kTolerance = 1.5_deg,
    .kGearRatio = 64.0,
    .kP = 45.0,
    .kD = 0.8,
    .kG = 0.35,
    .kContinuousWrap = false};

stan::StanPivot armPivot{CANid::kArmMotor, stan::MotorType::kTalonFX, CANid::kArmCANcoder, armConfig};

// Commandes de positionnement
mDriverController.bindPress(mDriverController.a(), armPivot.goToAngle(0_deg));
mDriverController.bindPress(mDriverController.y(), armPivot.goToAngle(90_deg));
```

---

## 4. StanElevator

`StanElevator` pilote les chariots linéaires verticaux ou horizontaux.

### 4.1 Configuration (`ElevatorConfig`)
| Paramètre | Type | Description |
|---|---|---|
| `kMinHeight` | `meter_t` | Position basse maximale. |
| `kMaxHeight` | `meter_t` | Position haute maximale. |
| `kTolerance` | `meter_t` | Tolérance d'atteinte de consigne. |
| `kMetersPerTurn` | `double` | Déplacement linéaire par rotation de sortie. |
| `kGearRatio` | `double` | Rapport de réduction moteur / mécanisme. |
| `kG` | `double` | Tension de compensation de gravité au maintien. |

### 4.2 Exemple
```cpp
#include <stan/StanElevator.h>

stan::ElevatorConfig elevatorConfig{
    .kMinHeight = 0.0_m,
    .kMaxHeight = 1.8_m,
    .kTolerance = 0.02_m,
    .kMetersPerTurn = 0.05,
    .kGearRatio = 12.0,
    .kP = 30.0,
    .kD = 0.5,
    .kG = 0.4};

stan::StanElevator elevator{CANid::kElevatorMotor, stan::MotorType::kTalonFX, elevatorConfig};

mDriverController.bindPress(mDriverController.leftBumper(), elevator.goToHeight(0.2_m));
mDriverController.bindPress(mDriverController.rightBumper(), elevator.goToHeight(1.5_m));
```

---

## 5. StanSwerveDrivetrain

`StanSwerveDrivetrain` fournit un châssis holonomique à 4 modules swerve avec odométrie intégrée.

### 5.1 Fonctionnalités
- Discrétisation cinématique automatique (`ChassisSpeeds::Discretize`).
- Presets de réducteurs intégrés (`SDS MK4`, `MK4i`, `MAXSwerve`).
- Verrouillage en croix des roues (`lockWheels()`).
- Commutation Brake / Coast.
- Injection de mesures de vision pour recalage d'odométrie.

### 5.2 Exemple
```cpp
#include <stan/StanSwerveBuilder.h>

auto drivetrain = stan::StanSwerveBuilder::createSDS_MK4i_L2(
    0.6_m, 0.6_m,
    CANid::kFLDrive, CANid::kFLSteer, CANid::kFLEncoder, -45.0_deg,
    CANid::kFRDrive, CANid::kFRSteer, CANid::kFREncoder, 120.0_deg,
    CANid::kBLDrive, CANid::kBLSteer, CANid::kBLEncoder, 85.0_deg,
    CANid::kBRDrive, CANid::kBRSteer, CANid::kBREncoder, -110.0_deg,
    CANid::kPigeon2);

// Commande par défaut de déplacement relatif au terrain
drivetrain.SetDefaultCommand(drivetrain.driveCommand(
    [this] { return mDriverController.getLeftY(); },
    [this] { return mDriverController.getLeftX(); },
    [this] { return mDriverController.getRightX(); },
    true));

// Verrouillage défensif en croix sur le bouton X
mDriverController.bindHold(mDriverController.x(), drivetrain.lockWheels());
```

---

## 6. StanCarDrive

`StanCarDrive` pilote un châssis directionnel type voiture (deux roues directrices avant et propulsion arrière), identique à la cinématique de `Mid-Robot`.

### 6.1 Exemple
```cpp
#include <stan/StanCarDrive.h>

stan::CarDriveConfig carConfig{
    .kSteerGearRatio = 12.8,
    .kMaxSteerAngle = 45.0_deg,
    .kSteerP = 24.0,
    .kSteerD = 0.2};

stan::StanCarDrive carDrive{
    CANid::kFLDrive, CANid::kFLSteer, CANid::kFLEncoder,
    CANid::kFRDrive, CANid::kFRSteer, CANid::kFREncoder,
    carConfig};

// Commande par défaut (accélérateur, frein, direction)
carDrive.SetDefaultCommand(carDrive.driveCommand(
    [this] { return mDriverController.getRightTrigger(); },
    [this] { return mDriverController.getLeftTrigger(); },
    [this] { return mDriverController.getRightX(); },
    false));
```
