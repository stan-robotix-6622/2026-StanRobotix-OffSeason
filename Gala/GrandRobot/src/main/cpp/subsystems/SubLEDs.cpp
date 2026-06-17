// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubLEDs.h"

SubLEDs::SubLEDs()
{
	mLed.SetLength(LEDsConstants::kLength);
	mLed.SetData(mLedBuffer);
	mLed.Start();

	addGradiant(frc::Color("#000000"), frc::Color("#ff0000"), int(LEDsConstants::kLength * 3 / 8), mRedBlueGradiant);
	addGradiant(frc::Color("#ff0000"), frc::Color("#000000"), int(LEDsConstants::kLength / 8), mRedBlueGradiant);
	addGradiant(frc::Color("#000000"), frc::Color("#0000ff"), int(LEDsConstants::kLength * 3 / 8), mRedBlueGradiant);
	addGradiant(frc::Color("#0000ff"), frc::Color("#000000"), int(LEDsConstants::kLength / 8), mRedBlueGradiant);
	mRedBlueLEDPattern = frc::LEDPattern::Gradient(frc::LEDPattern::kContinuous, mRedBlueGradiant).ScrollAtAbsoluteSpeed(LEDsConstants::kScrollingSpeed, LEDsConstants::kLedSpacing);

	addGradiant(frc::Color("#000000"), frc::Color("#ff0000"), int(LEDsConstants::kLength * 3 / 8) + 2, mRedGradiant);
	addGradiant(frc::Color("#ff0000"), frc::Color("#000000"), int(LEDsConstants::kLength / 8) + 1, mRedGradiant);
	addGradiant(frc::Color("#000000"), frc::Color("#0000ff"), int(LEDsConstants::kLength * 3 / 8) - 2, mBlueGradiant);
	addGradiant(frc::Color("#0000ff"), frc::Color("#000000"), int(LEDsConstants::kLength / 8) - 1, mBlueGradiant);
	mRedFromMiddleLEDPattern = frc::LEDPattern::Gradient(frc::LEDPattern::kContinuous, mRedGradiant).ScrollAtAbsoluteSpeed(LEDsConstants::kScrollingSpeed * 3, LEDsConstants::kLedSpacing).OffsetBy(int(LEDsConstants::kLength / 2));
	mBlueFromMiddleLEDPattern = frc::LEDPattern::Gradient(frc::LEDPattern::kContinuous, mBlueGradiant).ScrollAtAbsoluteSpeed(LEDsConstants::kScrollingSpeed * 3, LEDsConstants::kLedSpacing).Reversed();

	addGradiant(frc::Color("#000000"), frc::Color("#FFA500"), int(LEDsConstants::kLength / 2), mOrangePulseGradiant);
	addGradiant(frc::Color("#FFA500"), frc::Color("#000000"), int(LEDsConstants::kLength / 2), mOrangePulseGradiant);
	mOrangePulseLEDPattern = frc::LEDPattern::Gradient(frc::LEDPattern::kContinuous, mOrangePulseGradiant).ScrollAtAbsoluteSpeed(LEDsConstants::kScrollingSpeed, LEDsConstants::kLedSpacing);

	mOrangeBlinkingLEDPattern = frc::LEDPattern::Solid(frc::Color("#FFA500")).Blink(0.5_s);
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
		case movingWithSmallRobot:
			mOrangeBlinkingLEDPattern.ApplyTo(mLedBuffer);
			break;
		case deploying:
			mRedFromMiddleLEDPattern.ApplyTo(mLeft);
			mBlueFromMiddleLEDPattern.ApplyTo(mRight);
			break;
		case test:
			mWhiteLEDPattern.ApplyTo(mLedBuffer);
			break;
		case off:
			mOffLEDPattern.ApplyTo(mLedBuffer);
			break;
		default:
			mOffLEDPattern.ApplyTo(mLedBuffer);
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
	for (int i = 0; i < iNumberOfSteps; i++) {
		iModifiedVector.emplace_back(frc::Color(startingR - (startingR - endingR) / (iNumberOfSteps - 1) * i,
		                                        startingG - (startingG - endingG) / (iNumberOfSteps - 1) * i,
		                                        startingB - (startingB - endingB) / (iNumberOfSteps - 1) * i));
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
