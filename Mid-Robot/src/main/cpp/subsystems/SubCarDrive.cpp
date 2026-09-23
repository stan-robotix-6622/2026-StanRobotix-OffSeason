#include "subsystems/SubCarDrive.h"
#include "Constants.h"

SubCarDrive::SubCarDrive()
    : stan::StanCarDrive{CarDriveConstants::createConfig()} {}
