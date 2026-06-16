// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/AddressableLED.h>
#include <frc/LEDPattern.h>
#include <frc2/command/SubsystemBase.h>

#include <array>
#include <ranges>
#include <vector>

#include "Constants.h"

class SubLEDs : public frc2::SubsystemBase {
 public:
	enum Mode {
		immobile,
		moving,
		movingWithSmallRobot,
		deploying,
		test
	};

	SubLEDs();

	void addGradiant(frc::Color iStartingColor, frc::Color iEndingColor, int iNumberOfSteps, std::vector<frc::Color>& iModifiedVector);
	void setWhite();

	void setMode(Mode iMode);
	Mode getMode();

	void Periodic() override;

 private:
	// Must be a PWM header, not MXP or DIO
	frc::AddressableLED mLed{LEDsConstants::kLEDPort};
	frc::LEDPattern mWhiteLEDPattern = frc::LEDPattern::Solid(frc::Color("#FFFFFF")).AtBrightness(0.5);
	frc::LEDPattern mRedBlueLEDPattern = frc::LEDPattern::Off();
	frc::LEDPattern mRedFromMiddleLEDPattern = frc::LEDPattern::Off();
	frc::LEDPattern mBlueFromMiddleLEDPattern = frc::LEDPattern::Off();
	frc::LEDPattern mOrangePulseLEDPattern = frc::LEDPattern::Off();
	frc::LEDPattern mOrangeBlinkingLEDPattern = frc::LEDPattern::Off();
	std::vector<frc::Color> mOrangePulseGradiant;
	std::vector<frc::Color> mRedBlueGradiant;
	std::vector<frc::Color> mRedGradiant;
	std::vector<frc::Color> mBlueGradiant;
	std::array<frc::AddressableLED::LEDData, LEDsConstants::kLength> mLedBuffer; // Reuse the buffer
	std::ranges::drop_view<std::ranges::ref_view<std::array<frc::AddressableLED::LEDData, LEDsConstants::kLength>>> mLeft =
					std::ranges::drop_view(mLedBuffer, int(LEDsConstants::kLength / 2));
	std::ranges::take_view<std::ranges::ref_view<std::array<frc::AddressableLED::LEDData, LEDsConstants::kLength>>> mRight =
					std::ranges::take_view(mLedBuffer, int(LEDsConstants::kLength / 2));

	Mode mMode;
};
