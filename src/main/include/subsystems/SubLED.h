// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
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


  void addGradiant(frc::Color iStartingColor, frc::Color iEndingColor, int iNumberOfSteps, std::vector<frc::Color>& iModifiedVector);

	frc::LEDPattern getPulseLEDsPattern(frc::Color iPulseColor);

	void setWhite();

	void setMode(Mode iMode);

	Mode mMode;

    void Periodic();
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

 frc::AddressableLED m_led{LEDsConstants::kLEDPort};
	frc::LEDPattern m_RedBlueLEDPattern = frc::LEDPattern::Off();
	frc::LEDPattern m_OrangePulseLEDPattern = frc::LEDPattern::Off();
	std::vector<frc::Color> m_RedBlueGradiant;
	std::array<frc::AddressableLED::LEDData, LEDsConstants::kLength> m_ledBuffer; // Reuse the buffer
};
