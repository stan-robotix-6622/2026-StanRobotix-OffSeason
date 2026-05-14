#pragma once

#include <array>
#include <frc/AddressableLED.h>
#include <frc/LEDPattern.h>

#include "Constants.h"

class LED {
 public:
    enum Mode
    {
        immobile,
        moving,
        waving,
        talking,
        test
    };

    LED();

    void addGradiant(frc::Color iStartingColor, frc::Color iEndingColor, int iNumberOfSteps, std::vector<frc::Color>& iModifiedVector);

    frc::LEDPattern getPulseLEDsPattern(frc::Color iPulseColor);

    void setWhite();
    
    void setMode(Mode iMode);

    bool isImmobile = false;
    bool isMoving = false;

 private:
    // Must be a PWM header, not MXP or DIO
    frc::AddressableLED m_led{LEDsConstants::kLEDPort};
    frc::LEDPattern m_RedBlueLEDPattern = frc::LEDPattern::Off();
    frc::LEDPattern m_OrangePulseLEDPattern = frc::LEDPattern::Off();
    std::vector<frc::Color> m_RedBlueGradiant;
    std::array<frc::AddressableLED::LEDData, LEDsConstants::kLength> m_ledBuffer;  // Reuse the buffer
};
