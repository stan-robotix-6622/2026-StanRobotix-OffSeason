// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "LED.h"

LED::LED()
{
	m_led.SetLength(LEDsConstants::kLength);
	m_led.Start();

	addGradiant(frc::Color("#ff0000"), frc::Color("#000000"), 9, m_RedBlueGradiant);
	addGradiant(frc::Color("#0000ff"), frc::Color("#000000"), 9, m_RedBlueGradiant);
	m_RedBlueLEDPattern = frc::LEDPattern::Gradient(frc::LEDPattern::kContinuous, m_RedBlueGradiant).ScrollAtAbsoluteSpeed(0.5_mps, LEDsConstants::kLedSpacing);

	m_OrangePulseLEDPattern = getPulseLEDsPattern(frc::Color("#FFA500"));
}

void LED::addGradiant(frc::Color iStartingColor, frc::Color iEndingColor, int iNumberOfSteps, std::vector<frc::Color>& iModifiedVector)
{
	double startingR = iStartingColor.red;
	double startingG = iStartingColor.green;
	double startingB = iStartingColor.blue;
	double endingR = iEndingColor.red;
	double endingG = iEndingColor.green;
	double endingB = iEndingColor.blue;
	for (int i = 0; i <= iNumberOfSteps; i++) {
		iModifiedVector.emplace_back(frc::Color(startingR - (startingR - endingR) / iNumberOfSteps * i,
		                                        startingG - (startingG - endingG) / iNumberOfSteps * i,
		                                        startingB - (startingB - endingB) / iNumberOfSteps * i));
	}
}

frc::LEDPattern LED::getPulseLEDsPattern(frc::Color iPulseColor)
{
	std::vector<frc::Color> m_PulseGradiant;
	addGradiant(frc::Color("#000000"), iPulseColor, 9, m_PulseGradiant);
	addGradiant(iPulseColor, frc::Color("#000000"), 9, m_PulseGradiant);
	frc::LEDPattern m_PulseLEDPattern{frc::LEDPattern::Gradient(frc::LEDPattern::kDiscontinuous, m_PulseGradiant).ScrollAtAbsoluteSpeed(0.5_mps, LEDsConstants::kLedSpacing)};

	return m_PulseLEDPattern;
}

void LED::setWhite()
{
	for (int i = 0; i < LEDsConstants::kLength; i++) {
		m_ledBuffer[i].SetRGB(255, 255, 255);
	}
}

void LED::setMode(Mode iMode)
{
	switch (iMode) {
		case immobile:
			isImmobile = true;
			isMoving = false;
			m_OrangePulseLEDPattern.ApplyTo(m_ledBuffer);
			break;
		case moving:
			isImmobile = false;
			isMoving = true;
			m_RedBlueLEDPattern.ApplyTo(m_ledBuffer);
			break;
		case test:
			isImmobile = false;
			isMoving = false;
			setWhite();
	};
	m_led.SetData(m_ledBuffer);
}
