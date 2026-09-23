# Référence API

Ce document recense les classes, structures, méthodes et constantes de **StanRobotixLib**.

---

## 1. Matériel

### 1.1 `enum class stan::MotorType`
Définit la famille de contrôleur moteur :
- `kTalonFX` : CTRE Talon FX (Kraken X60, Falcon 500).
- `kSparkMax` : REV SparkMax (NEO).
- `kSparkFlex` : REV SparkFlex (NEO Vortex).

### 1.2 `enum class stan::IdleMode`
Définit le mode neutre :
- `kBrake` : Frein électromagnétique actif (valeur par défaut).
- `kCoast` : Roue libre.

### 1.3 `class stan::StanMotor`
Abstraction matérielle directe pour moteurs CTRE et REV.

#### Constructeurs & Destructeur
- `StanMotor(int iCanId, MotorType iType, std::string_view iCanBus = "rio")` : Initialise le moteur avec `Brake` et limite de courant 40 A activés.
- `~StanMotor()` : Libère la mémoire des objets alloués.

#### Méthodes de Commande
- `void set(double iSpeed)` : Commande en rapport cyclique [-1.0, 1.0].
- `void setVoltage(units::voltage::volt_t iVoltage)` : Commande en tension.
- `void setVelocity(units::angular_velocity::turns_per_second_t iVelocity)` : Asservissement en vitesse onboard 1 kHz.
- `void setPosition(units::angle::turn_t iPosition)` : Asservissement en position onboard 1 kHz.
- `void stopMotor()` : Coupe l'alimentation du moteur.
- `void setIdleMode(IdleMode iMode)` : Commute entre `kBrake` et `kCoast`.
- `void setInverted(bool iInverted)` : Définit le sens de rotation.
- `void setCurrentLimit(units::current::ampere_t iLimit)` : Modifie la limite de courant.

#### Méthodes de Télémétrie
- `units::voltage::volt_t getVoltage() const` : Tension appliquée.
- `units::current::ampere_t getCurrent() const` : Courant statorique mesuré.
- `units::angular_velocity::turns_per_second_t getVelocity() const` : Vitesse mesurée.
- `units::angle::turn_t getPosition() const` : Position mesurée.
- `MotorType getType() const` : Type de variateur.
- `int getCanId() const` : Identifiant CAN.

#### Accès Natifs
- `ctre::phoenix6::hardware::TalonFX* getTalonFX()`
- `rev::spark::SparkMax* getSparkMax()`
- `rev::spark::SparkFlex* getSparkFlex()`
- `rev::spark::SparkClosedLoopController* getSparkClosedLoopController()`
- `rev::spark::SparkRelativeEncoder* getSparkRelativeEncoder()`

### 1.4 `class stan::KrakenSync`
Synchronisation statique de position absolue :
- `static bool sync(ctre::phoenix6::hardware::TalonFX* iMotor, ctre::phoenix6::hardware::CANcoder* iEncoder, units::time::millisecond_t iTimeout = 250_ms)`
- `static bool sync(ctre::phoenix6::hardware::TalonFX& iMotor, ctre::phoenix6::hardware::CANcoder& iEncoder, units::time::millisecond_t iTimeout = 250_ms)`  
  Copie la position du `CANcoder` dans le `RotorSensor` du Talon FX. Renvoie `true` si l'opération a réussi.

---

## 2. Mécanismes

### 2.1 `class stan::StanRoller`
Sous-système pour rouleaux d'admission et convoyeurs.

- `StanRoller(StanMotor* iMotor)` : Constructeur par injection.
- `StanRoller(int iCanId, MotorType iType, std::string_view iCanBus = "rio")` : Constructeur propriétaire.
- `void setSpeed(double iSpeed)` : Commande en vitesse relative [-1.0, 1.0].
- `void setVoltage(units::voltage::volt_t iVoltage)` : Commande en tension directe.
- `void stop()` : Arrêt.
- `double getSpeed() const` : Vitesse de consigne actuelle.
- `units::current::ampere_t getCurrent() const` : Courant mesuré.
- `units::voltage::volt_t getVoltage() const` : Tension mesurée.
- `StanMotor* getMotor()` : Pointeur vers le moteur interne.
- `frc2::CommandPtr runRoller(double iSpeed)` : Commande de fonctionnement continu.
- `frc2::CommandPtr stopCommand()` : Commande d'arrêt.

### 2.2 `class stan::StanFlywheel`
Sous-système pour volants d'inertie et lanceurs.

- `StanFlywheel(StanMotor* iMotor, const FlywheelConfig& iConfig)`
- `StanFlywheel(int iCanId, MotorType iType, const FlywheelConfig& iConfig)`
- `void setVelocity(units::angular_velocity::turns_per_second_t iVelocity)` : Consigne de vitesse.
- `void stop()` : Arrêt.
- `bool atDesiredVelocity() const` : Vérifie si la vitesse est dans l'intervalle `[cible - tolérance, cible + tolérance]`.
- `units::angular_velocity::turns_per_second_t getVelocity() const` : Vitesse mesurée.
- `units::angular_velocity::turns_per_second_t getTargetVelocity() const` : Vitesse cible.
- `frc2::CommandPtr spinVelocity(units::angular_velocity::turns_per_second_t iVelocity)` : Commande de mise en vitesse.
- `frc2::CommandPtr waitUntilReady()` : Commande bloquante jusqu'à atteinte du régime cible.
- `frc2::CommandPtr stopCommand()` : Commande d'arrêt.

### 2.3 `class stan::StanPivot`
Sous-système pour articulations angulaires (bras, poignets).

- `StanPivot(StanMotor* iMotor, ctre::phoenix6::hardware::CANcoder* iEncoder, const PivotConfig& iConfig)`
- `StanPivot(int iMotorCanId, MotorType iType, int iEncoderCanId, const PivotConfig& iConfig)`
- `void setTargetAngle(units::angle::degree_t iAngle)` : Consigne angulaire.
- `units::angle::degree_t getAngle() const` : Angle mesuré.
- `units::angle::degree_t getTargetAngle() const` : Angle cible.
- `bool atTargetAngle() const` : Vérifie l'atteinte de la consigne.
- `bool syncEncoder()` : Déclenche manuellement la synchronisation `KrakenSync`.
- `frc2::CommandPtr goToAngle(units::angle::degree_t iAngle)` : Commande de positionnement.
- `frc2::CommandPtr syncEncoderCommand()` : Commande de recalage absolu.

### 2.4 `class stan::StanElevator`
Sous-système pour ascenseurs linéaires.

- `StanElevator(StanMotor* iMotor, const ElevatorConfig& iConfig)`
- `StanElevator(int iCanId, MotorType iType, const ElevatorConfig& iConfig)`
- `void setTargetHeight(units::length::meter_t iHeight)` : Consigne de hauteur linéaire.
- `units::length::meter_t getHeight() const` : Hauteur mesurée.
- `units::length::meter_t getTargetHeight() const` : Hauteur cible.
- `bool atTargetHeight() const` : Vérifie l'atteinte de la hauteur cible.
- `frc2::CommandPtr goToHeight(units::length::meter_t iHeight)` : Commande de levée/descente.

---

## 3. Châssis & Propulsion

### 3.1 `class stan::StanSwerveBuilder`
Constructeur d'assemblage pour châssis holonomique 4 modules :
- `static StanSwerveDrivetrain createSDS_MK4_L2(...)`
- `static StanSwerveDrivetrain createSDS_MK4i_L2(...)`
- `static StanSwerveDrivetrain createSDS_MK4i_L3(...)`
- `static StanSwerveDrivetrain createMAXSwerve(...)`

### 3.2 `class stan::StanSwerveDrivetrain`
Sous-système de propulsion swerve :
- `void drive(units::velocity::meters_per_second_t iVx, units::velocity::meters_per_second_t iVy, units::angular_velocity::radians_per_second_t iOmega, bool iFieldRelative = true)`
- `void setIdleMode(IdleMode iMode)` : Commute les modules en Brake ou Coast.
- `frc::Pose2d getPose() const` : Pose estimée sur le terrain.
- `void resetPose(const frc::Pose2d& iPose)` : Réinitialise l'odométrie.
- `void addVisionMeasurement(const frc::Pose2d& iVisionPose, units::time::second_t iTimestamp)` : Injection de mesure caméra.
- `frc2::CommandPtr driveCommand(...)` : Commande de pilotage continu.
- `frc2::CommandPtr lockWheels()` : Verrouillage défensif en croix.

### 3.3 `class stan::StanCarDrive`
Sous-système directionnel type voiture (2 roues directrices avant + différentiel arrière) :
- `void drive(double iThrottle, double iBrake, double iSteer, bool iReverse = false)`
- `void setSteerAngle(units::angle::degree_t iAngle)`
- `void zeroFL()`, `void zeroFR()`
- `frc2::CommandPtr driveCommand(...)` : Commande de pilotage continu.

---

## 4. Contrôle & Télémétrie

### 4.1 `class stan::StanTunablePID`
Synchronisation dynamique de gains avec NetworkTables 4 :
- `StanTunablePID(std::string_view iTableName, StanMotor* iMotor, double iDefaultP, double iDefaultI, double iDefaultD, double iDefaultS = 0.0, double iDefaultV = 0.0)`
- `void periodic()` : Écoute les topics `/Tuning/{TableName}/*` et met à jour le variateur en cas de modification.
- `double getP() const`, `double getI() const`, `double getD() const`, `double getS() const`, `double getV() const`

### 4.2 `namespace stan::TuningPresets`
Valeurs d'initialisation recommandées :
- `kFlywheelBaseline` : Gains types pour lanceur à volant d'inertie.
- `kPivotBaseline` : Gains types pour articulation angulaire avec compensation de gravité.
- `kRollerBaseline` : Gains types pour entraînement continu.

---

## 5. Entrées Pilote

### 5.1 `class stan::StanXboxController`
Dérive de `frc2::CommandXboxController` :
- `explicit StanXboxController(int iPort, double iDeadband = 0.1)`
- `double getLeftX() const`, `double getLeftY() const` : Axes avec deadband appliqué.
- `double getRightX() const`, `double getRightY() const` : Axes avec deadband appliqué.
- `double getLeftTrigger() const`, `double getRightTrigger() const`
- `void bindHold(const frc2::Trigger& iTrigger, frc2::CommandPtr iCommand)` : Maintien actif d'une commande.
- `void bindToggle(const frc2::Trigger& iTrigger, frc2::CommandPtr iCommand)` : Bascule active/inactive.
- `void bindPress(const frc2::Trigger& iTrigger, frc2::CommandPtr iCommand)` : Déclenchement impulsionnel à l'appui.
