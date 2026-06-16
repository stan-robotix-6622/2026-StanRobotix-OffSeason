// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/AddressableLED.h>
#include <frc/LEDPattern.h>
#include <frc2/command/SubsystemBase.h>

#include <array>

#include "Constants.h"

class SubLEDs : public frc2::SubsystemBase {
 public:
	enum Mode {
		immobile,
		moving,
		deploying,
		talking,
		test
	};

	SubLEDs();

	void addGradiant(frc::Color iStartingColor, frc::Color iEndingColor, int iNumberOfSteps, std::vector<frc::Color>& iModifiedVector);

	frc::LEDPattern getPulseLEDsPattern(frc::Color iPulseColor);

	void setWhite();

	void setMode(Mode iMode);
	Mode getMode();

	void Periodic() override;
	
	private:
	// Must be a PWM header, not MXP or DIO
	frc::AddressableLED m_led{LEDsConstants::kLEDPort};
	frc::LEDPattern m_RedBlueLEDPattern = frc::LEDPattern::Off();
	frc::LEDPattern m_OrangePulseLEDPattern = frc::LEDPattern::Off();
	std::vector<frc::Color> m_RedBlueGradiant;
	std::array<frc::AddressableLED::LEDData, LEDsConstants::kLength> m_ledBuffer; // Reuse the buffer

	Mode mMode;
};