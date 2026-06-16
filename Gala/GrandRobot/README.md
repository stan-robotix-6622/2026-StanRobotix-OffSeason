# 2026-StanRobotix-OffSeason ![star](https://img.shields.io/github/stars/stan-robotix-6622/2026-StanRobotix-OffSeason)

Dépôt officiel de l'équipe [Stan Robotix 6622](https://stanrobotix6622.com/) pour la hors saison 2026

## Conventions de structure

- Branches : `(Projet)-Nom de Mécanisme/Commande-(Fonctionnalité)`
  - `Projet` n'est pas nécessaire dans un dépôt pour un projet spécifique (FRC) mais doit être inclus lors de la hors saison
  - `Fonctionnalité` n'est pas nécessaire mais peut être utile pour différencier et expliquer les branches
- Arborescence d'un projet :
  - Code spécifique au projet : `src/main/`
  - Code utile mais non-spécifique : `src/stanbrairy/`

## Pour ajouter un projet

1. Cloner le dépot :
```
git clone https://github.com/stan-robotix-6622/2026-StanRobotix-OffSeason.git
```
2. Créer une nouvelle branche pour le projet :
```
git switch -c NomDuProjet
```
3. Créer un nouveau projet avec le créateur le projet WPILib :
    - Ouvrir le dépot dans VS Code WPILib
    - Entrer dans la barre en haut de l'application :
    ```
    >WPILib: Create a new project
    ```

## Les conventions de code

- Programmer en anglais
- Variables dans les méthodes :
  - Préfixe i (input) pour les variables d'entrée (e.g. iVelocity)
  - Préfixe o (output) pour les variables de sorties (e.g. oAngle)
- Variables locales :
  - Préfixe w (work) (e.g. wDeltaValue, wPIDOutput)
- Dans une classe :
  - Préfixe m (my) pour les attributs (e.g. mAccelerometer)
  - Le `*` des pointeurs est collé au type de variable (e.g. `int* number`)
  - Le nom des méthodes commence par une minuscule puis on utilise des majuscules pour les autres mots (e.g. setVoltage())
- Nomenclature :
  - Subsystems: `Sub` suivit du type de subsystème (e.g. Drivetrain, Intake, IMU...)
  - Commandes: ce que fait la commande (e.g. SetElevatorPosition, ActivateShooter, ControlIntake...)
- Préfixe k (constant) pour les variables constantes (e.g. kP, kLeftMotorCanID, kGearRatio...)
- Mettre des commentaires mais ne pas en abuser // Ceci est un abus

## Rappel des commandes de base de Git

```
git clone
git status
git add --all
git add "CheminDuFichier"
git rm "CheminDuFichier"
git commit -m "MessageQuiADuSens"
git switch NomDeLaBranche
git branch NomDeLaBranche
git push
git pull
git fetch
gitk
```

## Gestion

La gestion du dépôt (modification du README ou modifications majeures) est laissée au chef de l'équipe et aux mentors.

Chef d'équipe : Timothée Laberge<br>
Mentors : Raphaël Pothier, André Wojcik et Micha Reneault<br>

Contacts :<br>
timothee.laberge@stanislas.qc.ca <br>
raphael.pothier@stanislas.qc.ca <br>
wojcik.andre2.0@gmail.com <br>
micha.reneault@gmail.com <br>
