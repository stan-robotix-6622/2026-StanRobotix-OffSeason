# Démarrage Rapide

Ce guide détaille l'intégration et l'utilisation initiale de **StanRobotixLib** dans un projet robotique C++ WPILib.

---

## 1. Prérequis

- **WPILib** : Version 2025 ou 2026 (C++20).
- **Vendordeps requises** :
  - `Phoenix6` (pour les moteurs CTRE Talon FX, CANcoder, Pigeon 2).
  - `WPILibNewCommands` (inclus par défaut).
  - `REVLib` (uniquement si des SparkMax ou SparkFlex sont utilisés).

---

## 2. Installation de la Vendordep

### Étape 2.1 : Génération des En-Têtes
Depuis la racine de `StanRobotixLib/` :
```bash
./gradlew headersZip publish
```
Cette tâche produit l'archive `build/libs/StanRobotixLib-cpp-1.0.0-headers.zip` et la déploie dans le dépôt Maven local (`build/repos/releases/`).

### Étape 2.2 : Ajout du Descripteur Vendordep
Copiez le fichier `StanRobotixLib.json` dans le dossier `vendordeps/` de votre projet robot :
```bash
cp StanRobotixLib/StanRobotixLib.json <chemin-du-projet-robot>/vendordeps/
```

### Étape 2.3 : Déclaration du Dépôt Maven
Dans le fichier `build.gradle` de votre projet robot, assurez-vous que le bloc `repositories` référence le chemin de la bibliothèque :
```groovy
repositories {
    mavenLocal()
    maven {
        name = "StanRobotixLibLocal"
        url = "${rootDir}/../../StanRobotixLib/build/repos/releases"
    }
}
```

### Étape 2.4 : Vérification
Lancez les tests de compilation du robot :
```bash
./gradlew test
```
L'affichage de `BUILD SUCCESSFUL` confirme la disponibilité des en-têtes.

---

## 3. Exemple d'Intégration Minimale

### 3.1 En-Tête (`RobotContainer.h`)
```cpp
#pragma once

#include <frc2/command/CommandPtr.h>
#include <stan/StanRoller.h>
#include <stan/StanXboxController.h>

class RobotContainer {
 public:
  RobotContainer();

 private:
  void ConfigureBindings();

  stan::StanXboxController mDriverController{0};
  stan::StanRoller mIntake{1, stan::MotorType::kTalonFX};
};
```

### 3.2 Implémentation (`RobotContainer.cpp`)
```cpp
#include "RobotContainer.h"
#include <units/voltage.h>

RobotContainer::RobotContainer() {
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  // Commande par défaut : maintien à l'arrêt
  mIntake.SetDefaultCommand(mIntake.runRoller(0.0));

  // Action sur maintien du bouton A
  mDriverController.bindHold(
      mDriverController.a(),
      mIntake.runRoller(0.8));

  // Action sur pression du bouton B (arrêt)
  mDriverController.bindPress(
      mDriverController.b(),
      mIntake.stopCommand());
}
```

---

## 4. Vérification et Compilation

Pour valider le code pendant le développement, utilisez toujours la compilation hôte native :
```bash
./gradlew test
```
*Note : Réservez `./gradlew build` à la validation finale avant déploiement sur le RoboRIO.*
