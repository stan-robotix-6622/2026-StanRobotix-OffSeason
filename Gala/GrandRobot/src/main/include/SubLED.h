// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <iostream>
#include <array>
#include <frc/AddressableLED.h>
#include <frc/LEDPattern.h>
#include <chrono>
#include <frc/Timer.h>

#include "Constants.h"

class SubLED : public frc2::SubsystemBase {
 public:

 enum Mode
    {
        immobile,
        moving,
        waving,
        talking,
        test
    };

  SubLED();


  void addGradiant(frc::Color iStartingColor, frc::Color iEndingColor, int iVectorSize, std::vector<frc::Color> * iModifiedVector);

  void setMouthLEDs(int iMouthSize);

  void pulseLEDs(frc::Color iPulseColor);

  void setWhite();

  void setMode(Mode iMode);
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  bool isImmobile = false;
  bool isMoving = false;
  bool isWaving = false;
  bool isTalking = false;

 private:

 frc::AddressableLED m_led{LEDsConstants::kLEDPort};  // Ce code est un code test, le port sur lequel se trouvera les LED est encore à changer.
    frc::LEDPattern * m_RedBlueLEDPattern;
    std::vector<frc::Color> * m_RedBlueGradiant;
    std::array<frc::AddressableLED::LEDData, LEDsConstants::kLength> m_ledBuffer;  // Reuse the buffer
    // Store what the last hue of the first pixel is
    int firstPixelHue = 0;
    int m_MouthSize = 0;
    bool m_MouthSizeIncreasing = true;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
