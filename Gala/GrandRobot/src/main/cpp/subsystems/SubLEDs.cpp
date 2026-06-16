// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubLEDs.h"

SubLEDs::SubLEDs()
{
	mLed.SetLength(LEDsConstants::kLength);
	mLed.SetData(mLedBuffer);
	mLed.Start();

	addGradiant(frc::Color("#ff0000"), frc::Color("#000000"), int(LEDsConstants::kLength / 2), mRedBlueGradiant);
	addGradiant(frc::Color("#0000ff"), frc::Color("#000000"), int(LEDsConstants::kLength / 2), mRedBlueGradiant);
	mRedBlueLEDPattern = frc::LEDPattern::Gradient(frc::LEDPattern::kContinuous, mRedBlueGradiant).ScrollAtAbsoluteSpeed(0.5_mps, LEDsConstants::kLedSpacing);

	mOrangePulseLEDPattern = getPulseLEDsPattern(frc::Color("#FFA500"));
}

void SubLEDs::Periodic()
{
	switch (mMode) {
		case immobile:
			mOrangePulseLEDPattern.ApplyTo(mLedBuffer);
			break;
		case moving:
			mRedBlueLEDPattern.ApplyTo(mLedBuffer);
			break;
		case test:
			setWhite();
			break;
		default:
			setWhite();
			break;
	};
	mLed.SetData(mLedBuffer);
}

void SubLEDs::addGradiant(frc::Color iStartingColor, frc::Color iEndingColor, int iNumberOfSteps, std::vector<frc::Color>& iModifiedVector)
{
	double startingR = iStartingColor.red;
	double startingG = iStartingColor.green;
	double startingB = iStartingColor.blue;
	double endingR = iEndingColor.red;
	double endingG = iEndingColor.green;
	double endingB = iEndingColor.blue;
	for (int i = 0; i <= iNumberOfSteps; i++) {
		iModifiedVector.emplace_back(frc::Color(startingR - (startingR - endingR) / (iNumberOfSteps - 1) * i,
		                                        startingG - (startingG - endingG) / (iNumberOfSteps - 1) * i,
		                                        startingB - (startingB - endingB) / (iNumberOfSteps - 1) * i));
	}
}

frc::LEDPattern SubLEDs::getPulseLEDsPattern(frc::Color iPulseColor)
{
	std::vector<frc::Color> mPulseGradiant;
	addGradiant(frc::Color("#000000"), iPulseColor, int(LEDsConstants::kLength / 2), mPulseGradiant);
	addGradiant(iPulseColor, frc::Color("#000000"), int(LEDsConstants::kLength / 2), mPulseGradiant);
	frc::LEDPattern mPulseLEDPattern{frc::LEDPattern::Gradient(frc::LEDPattern::kDiscontinuous, mPulseGradiant).ScrollAtAbsoluteSpeed(0.5_mps, LEDsConstants::kLedSpacing)};

	return mPulseLEDPattern;
}

void SubLEDs::setWhite()
{
	for (int i = 0; i < LEDsConstants::kLength; i++) {
		mLedBuffer[i].SetRGB(255, 255, 255);
	}
}

void SubLEDs::setMode(Mode iMode)
{
	mMode = iMode;
}

SubLEDs::Mode SubLEDs::getMode()
{
	return mMode;
}