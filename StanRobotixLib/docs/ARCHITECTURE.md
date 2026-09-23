# Architecture

Ce document décrit les principes architecturaux et techniques de **StanRobotixLib**, la bibliothèque C++20 officielle de l'équipe **Stan Robotix 6622**.

---

## 1. Vue d'Ensemble

StanRobotixLib est organisée en modules découplés selon une hiérarchie en couches :

```
[ Entrées Pilote (StanXboxController) ]
                  │
                  ▼
[ Mécanismes & Châssis (Roller, Flywheel, Pivot, Elevator, Swerve, CarDrive) ]
                  │
                  ▼
[ Couche Contrôle & Télémétrie (StanTunablePID, NetworkTables) ]
                  │
                  ▼
[ Abstraction Matérielle (StanMotor, KrakenSync) ]
                  │
                  ▼
[ Pilotes Constructeurs (CTRE Phoenix 6, REVLib) ]
```

### Objectifs Techniques
- **Zéro surcoût d'abstraction** : Pas de tables virtuelles inutiles ni d'encapsulation masquant les fonctions natives du constructeur.
- **Régulation onboard 1 kHz** : Asservissements exécutés directement sur le microcontrôleur des variateurs.
- **Sécurités par défaut** : Mode frein (`Brake`) et limite de courant (40 A) configurés dès l'instanciation.

---

## 2. Matériel & Moteurs

### 2.1 Conception de `StanMotor`
`StanMotor` unifie le pilotage des variateurs CTRE et REV sans ajouter de couche d'indirection opaque :
- **Types pris en charge** : `kTalonFX`, `kSparkMax`, `kSparkFlex`.
- **Accès natif direct** : Des méthodes d'inspection (`getTalonFX()`, `getSparkMax()`, etc.) retournent les pointeurs bruts vers les objets réels du constructeur pour accéder aux configurations avancées.
- **Compilation conditionnelle** : Grâce au garde `#if __has_include(<rev/SparkMax.h>)`, un projet utilisant uniquement CTRE (comme `Mid-Robot`) compile sans exiger la dépendance REVLib.

```cpp
stan::StanMotor intakeMotor{CANid::kIntake, stan::MotorType::kTalonFX};

// Accès direct à l'API constructeur si nécessaire
ctre::phoenix6::hardware::TalonFX* rawTalon = intakeMotor.getTalonFX();
```

---

## 3. Régulation & Fréquences de Contrôle

### 3.1 Régulation Décentralisée à 1 kHz
- Le processeur du RoboRIO exécute sa boucle de commandes à **50 Hz** (20 ms).
- Les variateurs de vitesse (Talon FX, SparkMax) exécutent leur boucle fermée PID et Feedforward à **1 kHz** (1 ms).

Les mécanismes `StanFlywheel`, `StanPivot`, `StanElevator` et `StanSwerveDrivetrain` délèguent systématiquement le calcul d'asservissement au processeur interne du moteur.

### 3.2 Synchronisation `KrakenSync`
Sur les moteurs Talon FX (Kraken X60), l'asservissement haute fréquence repose sur le capteur interne `RotorSensor`. L'utilisation d'un `CANcoder` absolu déporté nécessiterait la licence Phoenix Pro pour la fusion matérielle.

`KrakenSync` applique une synchronisation absolue au démarrage :
1. La position absolue du `CANcoder` est lue avec un délai maximal de 250 ms.
2. La valeur mesurée est injectée dans le `RotorSensor` du Talon FX.
3. Le régulateur Slot 0 tourne ensuite à 1 kHz sur le capteur interne en appliquant le ratio de réduction mécanique configuré.

```cpp
stan::KrakenSync::sync(mPivotMotor->getTalonFX(), mPivotCANcoder, 250_ms);
```

---

## 4. Gestion Mémoire & Cycle de Vie

Conformément aux standards de code 6622 (`rules.md`) :
- **Pointeurs bruts et typage explicite** : Les périphériques sont déclarés sous la forme `Type* mName;` avec alignement du pointeur à gauche.
- **Allocation déterministe** : Les allocations dynamiques (`new`) sont effectuées dans les constructeurs et libérées (`delete`) dans les destructeurs.
- **Zéro allocation en boucle 20 ms** : Aucun objet dynamique (`Pose2d`, `ChassisSpeeds`, chaînes de caractères) n'est alloué dans la méthode `Periodic()`.

---

## 5. Télémétrie & Tableau de Bord

Chaque mécanisme sépare distinctement la logique de mise à jour :

```cpp
void Subsystem::Periodic() {
  updateConfigsFromDashboard();
  updateTelemetry();
}
```

- **Télémétrie passive** : `updateTelemetry()` est strictement en lecture seule. Aucune commande moteur ni mutation d'état n'y est autorisée.
- **Appels CAN non-bloquants** : Les méthodes bloquantes (`Apply()`, `Configure()`) sont interdites dans `Periodic()`. Les paramètres modifiés à chaud via `StanTunablePID` utilisent des mécanismes d'écriture conditionnelle sur drapeau de modification (*dirty flag*).
- **NetworkTables 4** : Les *topics* (publishers et subscribers) sont pré-alloués lors de la construction.

---

## 6. Modèle Command-Based

StanRobotixLib utilise le modèle *Modern Command-Based* de WPILib :
- Toutes les actions de mécanismes sont exposées sous forme de fabriques renvoyant des `frc2::CommandPtr` par valeur.
- Les liaisons manette sont gérées via `StanXboxController` avec application automatique d'une zone morte (*deadband*) de 0.1 et limitation du taux de variation (*slew rate*).
