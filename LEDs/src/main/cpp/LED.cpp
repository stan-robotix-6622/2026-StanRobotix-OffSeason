#include "LED.h"

// #include <frc/RobotBase.h>

LED::LED()
{
  m_led.SetLength(LEDsConstants::kLength);
  m_led.Start();

  // if (frc::RobotBase::IsSimulation())
  // {
  //   mRobotIsSimulated = true;
  //   mLedSim.SetRunning(true);
  //   mLedSim.SetInitialized(true);
  //   mLedSim.SetLength(LEDsConstants::kLength);
  // }
  
  m_RedBlueGradiant = new std::vector<frc::Color>{};
  addGradiant(frc::Color("#ff0000"), frc::Color("#000000"), 9, m_RedBlueGradiant);
  addGradiant(frc::Color("#0000ff"), frc::Color("#000000"), 9, m_RedBlueGradiant);
  m_RedBlueLEDPattern = new frc::LEDPattern{frc::LEDPattern::Gradient(frc::LEDPattern::kContinuous, *m_RedBlueGradiant).ScrollAtAbsoluteSpeed(0.5_mps, LEDsConstants::kLedSpacing)};
}

void LED::addGradiant(frc::Color iStartingColor, frc::Color iEndingColor, int iVectorSize, std::vector<frc::Color> * iModifiedVector)
{
  int startingR = iStartingColor.red;
  int startingG = iStartingColor.green;
  int startingB = iStartingColor.blue;
  int endingR = iEndingColor.red;
  int endingG = iEndingColor.green;
  int endingB = iEndingColor.blue;
  for (int i = 0; i <= iVectorSize; i++)
  {
    iModifiedVector->emplace_back(frc::Color(startingR - (startingR - endingR) / iVectorSize * i,
                                              startingG - (startingG - endingG) / iVectorSize * i,
                                              startingB - (startingB - endingB) / iVectorSize * i));
  }
}


void LED::pulseLEDs(frc::Color iPulseColor)
{
  std::vector<frc::Color> * m_PulseGradiant = new std::vector<frc::Color>{};
  addGradiant(frc::Color("#000000"), iPulseColor, 9, m_PulseGradiant);
  addGradiant(iPulseColor, frc::Color("#000000"), 9, m_PulseGradiant);
  frc::LEDPattern * m_PulseLEDPattern = new frc::LEDPattern{frc::LEDPattern::Gradient(frc::LEDPattern::kDiscontinuous, *m_PulseGradiant).ScrollAtAbsoluteSpeed(0.5_mps, LEDsConstants::kLedSpacing)};

  m_PulseLEDPattern->ApplyTo(m_ledBuffer);
}

void LED::setWhite()
{
  for (int i = 0; i < LEDsConstants::kLength; i++)
  {
    m_ledBuffer[i].SetRGB(255, 255, 255);
  }
  m_led.SetData(m_ledBuffer);

  // if (mRobotIsSimulated) {
  //   // printf("Simulating");
  //   std::vector<HAL_AddressableLEDData*> wData; 
  //   for (unsigned int i = 0; i < m_ledBuffer.size(); i++)
  //   {
  //     wData.emplace_back(new HAL_AddressableLEDData{m_ledBuffer[i].b, m_ledBuffer[i].g, m_ledBuffer[i].r, m_ledBuffer[i].padding});
  //     std::cout << m_ledBuffer[i].b << " " <<m_ledBuffer[i].g << " " <<m_ledBuffer[i].r << " " <<m_ledBuffer[i].padding << "\n";
  //   }
  //   mLedSim.SetData(wData.front(), wData.size());
  // }
}

void LED::setMode(Mode iMode){
  switch (iMode)
  {
    case immobile:
      isImmobile = true; 
      isMoving = false;
      pulseLEDs(frc::Color("#FFA500"));
      break;
    case moving:
      isImmobile = false; 
      isMoving = true;
      m_RedBlueLEDPattern->ApplyTo(m_ledBuffer);

      break;
    case test:
      isImmobile = false; 
      isMoving = false;
      setWhite();
  };
  m_led.SetData(m_ledBuffer);
}