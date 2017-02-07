#include "calibration_tools.h"

uint8_t calibrationFlags = 0;

inline uint8_t isRunningCalibrationProcedure()
{
	return calibrationFlags;
}

inline uint8_t isFindingPoles() 
{
	return (calibrationFlags & CALIBRATION_FIND_POLES);
}
inline uint8_t isFindingCurrentZeroes()
{
	return (calibrationFlags & CALIBRATION_FIND_CURRENT_ZEROES);
}

inline uint8_t isLegalCalibrationProcedure(uint8_t procedure)
{
	//ensure procedure is not out of bounds
	//ensure procedure has only 1 bit true
	return ((procedure == 1 || procedure % 2 == 0) && procedure <= CALIBRATION_FIND_CURRENT_ZEROES);
}