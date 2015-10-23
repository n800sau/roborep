/***************************************************************************//**
* @file
* Portable %CMUcam4_esp interface library.
*
* @version @n 1.0
* @date @n 8/3/2012
*
* @authors @n Kwabena W. Agyeman & Christopher J. Leaf
* @copyright @n (c) 2012 Kwabena W. Agyeman & Christopher J. Leaf
* @n All rights reserved - Please see the end of the file for the terms of use
*
* @par Update History:
* @n v0.1 - Beta code - 3/20/2012
* @n v0.9 - Original release - 4/18/2012
* @n v1.0 - Documented and updated release - 8/3/2012
*******************************************************************************/

#include "CMUcam4_esp.h"

#define STRTOK(buf, name) \
	pch = strtok(buf, " "); \
	if(pch) \
		name = atoi(pch); \
	else \
		return CMUCAM4_RETURN_FAILURE; \

/*******************************************************************************
* Constructor Functions
*******************************************************************************/

CMUcam4_esp::CMUcam4_esp()
{
	_state = DEACTIVATED;
}

/*******************************************************************************
* Helper Functions
*******************************************************************************/

int ICACHE_FLASH_ATTR CMUcam4_esp::getPixel(CMUcam4_esp_image_data_t * pointer,
					  int row, int column)
{
	if((pointer==NULL)||
	(row<CMUCAM4_MIN_BINARY_ROW)||(CMUCAM4_MAX_BINARY_ROW<row)||
	(column<CMUCAM4_MIN_BINARY_COLUMN)||(CMUCAM4_MAX_BINARY_COLUMN<column))
	{
		return CMUCAM4_RETURN_FAILURE;
	}

	return ((pointer->pixels[(row * CMUCAM4_ID_T_C) + (column / 8)]
	>> (7 - (column & 7))) & 1);
}

int ICACHE_FLASH_ATTR CMUcam4_esp::setPixel(CMUcam4_esp_image_data_t * pointer,
					  int row, int column, int value)
{
	int bitIndex; int byteIndex;

	if((pointer==NULL)||
	(row<CMUCAM4_MIN_BINARY_ROW)||(CMUCAM4_MAX_BINARY_ROW<row)||
	(column<CMUCAM4_MIN_BINARY_COLUMN)||(CMUCAM4_MAX_BINARY_COLUMN<column))
	{
		return CMUCAM4_RETURN_FAILURE;
	}

	bitIndex = (7 - (column & 7));
	byteIndex = ((row * CMUCAM4_ID_T_C) + (column / 8));

	pointer->pixels[byteIndex] =
	(((~(1<<bitIndex))&(pointer->pixels[byteIndex]))|((value?1:0)<<bitIndex));

	return CMUCAM4_RETURN_SUCCESS;
}

int ICACHE_FLASH_ATTR CMUcam4_esp::andPixels(CMUcam4_esp_image_data_t * destination,
					   CMUcam4_esp_image_data_t * source0,
					   CMUcam4_esp_image_data_t * source1)
{
	size_t index;

	if((destination == NULL) || (source0 == NULL) || (source1 == NULL))
	{
		return CMUCAM4_RETURN_FAILURE;
	}

	for(index = 0; index < CMUCAM4_ID_T_LENGTH; index++)
	{
		destination->pixels[index] =
		(source0->pixels[index] & source1->pixels[index]);
	}

	return CMUCAM4_RETURN_SUCCESS;
}

int ICACHE_FLASH_ATTR CMUcam4_esp::orPixels(CMUcam4_esp_image_data_t * destination,
					  CMUcam4_esp_image_data_t * source0,
					  CMUcam4_esp_image_data_t * source1)
{
	size_t index;

	if((destination == NULL) || (source0 == NULL) || (source1 == NULL))
	{
		return CMUCAM4_RETURN_FAILURE;
	}

	for(index = 0; index < CMUCAM4_ID_T_LENGTH; index++)
	{
		destination->pixels[index] =
		(source0->pixels[index] | source1->pixels[index]);
	}

	return CMUCAM4_RETURN_SUCCESS;
}

int ICACHE_FLASH_ATTR CMUcam4_esp::xorPixels(CMUcam4_esp_image_data_t * destination,
					   CMUcam4_esp_image_data_t * source0,
					   CMUcam4_esp_image_data_t * source1)
{
	size_t index;

	if((destination == NULL) || (source0 == NULL) || (source1 == NULL))
	{
		return CMUCAM4_RETURN_FAILURE;
	}

	for(index = 0; index < CMUCAM4_ID_T_LENGTH; index++)
	{
		destination->pixels[index] =
		(source0->pixels[index] ^ source1->pixels[index]);
	}

	return CMUCAM4_RETURN_SUCCESS;
}

int ICACHE_FLASH_ATTR CMUcam4_esp::notPixels(CMUcam4_esp_image_data_t * destination)
{
	size_t index;

	if(destination == NULL)
	{
		return CMUCAM4_RETURN_FAILURE;
	}

	for(index = 0; index < CMUCAM4_ID_T_LENGTH; index++)
	{
		destination->pixels[index] =
		(~destination->pixels[index]);
	}

	return CMUCAM4_RETURN_SUCCESS;
}

int ICACHE_FLASH_ATTR CMUcam4_esp::isReadOnly(CMUcam4_esp_directory_entry_t * pointer)
{
	CMUcam4_esp_entry_attributes_t * attributes;

	if(pointer == NULL)
	{
		return CMUCAM4_RETURN_FAILURE;
	}

	attributes = ((CMUcam4_esp_entry_attributes_t *) pointer->attributes);
	return (attributes->readOnly == 'R');
}

int ICACHE_FLASH_ATTR CMUcam4_esp::isHidden(CMUcam4_esp_directory_entry_t * pointer)
{
	CMUcam4_esp_entry_attributes_t * attributes;

	if(pointer == NULL)
	{
		return CMUCAM4_RETURN_FAILURE;
	}

	attributes = ((CMUcam4_esp_entry_attributes_t *) pointer->attributes);
	return (attributes->hidden == 'H');
}

int ICACHE_FLASH_ATTR CMUcam4_esp::isSystem(CMUcam4_esp_directory_entry_t * pointer)
{
	CMUcam4_esp_entry_attributes_t * attributes;

	if(pointer == NULL)
	{
		return CMUCAM4_RETURN_FAILURE;
	}

	attributes = ((CMUcam4_esp_entry_attributes_t *) pointer->attributes);
	return (attributes->system == 'S');
}

int ICACHE_FLASH_ATTR CMUcam4_esp::isVolumeID(CMUcam4_esp_directory_entry_t * pointer)
{
	CMUcam4_esp_entry_attributes_t * attributes;

	if(pointer == NULL)
	{
		return CMUCAM4_RETURN_FAILURE;
	}

	attributes = ((CMUcam4_esp_entry_attributes_t *) pointer->attributes);
	return (attributes->volumeID == 'V');
}

int ICACHE_FLASH_ATTR CMUcam4_esp::isDirectory(CMUcam4_esp_directory_entry_t * pointer)
{
	CMUcam4_esp_entry_attributes_t * attributes;

	if(pointer == NULL)
	{
		return CMUCAM4_RETURN_FAILURE;
	}

	attributes = ((CMUcam4_esp_entry_attributes_t *) pointer->attributes);
	return (attributes->directory == 'D');
}

int ICACHE_FLASH_ATTR CMUcam4_esp::isArchive(CMUcam4_esp_directory_entry_t * pointer)
{
	CMUcam4_esp_entry_attributes_t * attributes;

	if(pointer == NULL)
	{
		return CMUCAM4_RETURN_FAILURE;
	}

	attributes = ((CMUcam4_esp_entry_attributes_t *) pointer->attributes);
	return (attributes->archive == 'A');
}

/*******************************************************************************
* State Functions
*******************************************************************************/

int ICACHE_FLASH_ATTR CMUcam4_esp::begin()
{
	int rs;
	int retVal0, retVal1;

	_state = DEACTIVATED;

	Serial.end();
	// Get the firmware version.
	Serial.write("\rGV\r");

	Serial1.println("wait for string");
	_setReadTimeout(CMUCAM4_NON_FS_TIMEOUT);
	_waitForString("\rCMUcam4_esp v");
	rs = _readText();
	if(rs == CMUCAM4_RETURN_SUCCESS) {
		retVal0 = atoi(_resBuffer);
		char *pos = strchr(_resBuffer, '.');
		retVal1 = (pos) ? atoi(pos+1) : -1;
		if(retVal0 == 0 || retVal1 < 0) {
			rs = CMUCAM4_UNEXPECTED_RESPONCE;
		} else {
			_version = ((_CMUcam4_esp_version) ((retVal0 * 100) + retVal1));
			Serial1.println(_version);
		}
	}

	if(rs == CMUCAM4_RETURN_SUCCESS) {
		rs = _waitForIdle();
		if(rs == CMUCAM4_RETURN_SUCCESS) {
			_state = ACTIVATED;
		}
	}
	Serial.end();
	Serial1.print("rs=");
	Serial1.println(rs);
	return rs;

}

int ICACHE_FLASH_ATTR CMUcam4_esp::end()
{
	if(_state == DEACTIVATED)
	{
		return CMUCAM4_NOT_ACTIVATED;
	}

	_state = DEACTIVATED;
	Serial.end();
}

/*******************************************************************************
* System Level Commands
*******************************************************************************/

int ICACHE_FLASH_ATTR CMUcam4_esp::getVersion()
{
	return (_state == ACTIVATED) ? _version : CMUCAM4_NOT_ACTIVATED;
}

int ICACHE_FLASH_ATTR CMUcam4_esp::resetSystem()
{
	return (_state == ACTIVATED) ? begin() : CMUCAM4_NOT_ACTIVATED;
}

int ICACHE_FLASH_ATTR CMUcam4_esp::sleepDeeply()
{
	return _commandWrapper("SD\r", CMUCAM4_NON_FS_TIMEOUT);
}

int ICACHE_FLASH_ATTR CMUcam4_esp::sleepLightly()
{
	return _commandWrapper("SL\r", CMUCAM4_NON_FS_TIMEOUT);
}

/*******************************************************************************
* Camera Module Commands
*******************************************************************************/

int ICACHE_FLASH_ATTR CMUcam4_esp::cameraBrightness(int brightness)
{
	return (snprintf(_cmdBuffer, CMUCAM4_CMD_BUFFER_SIZE,
	"CB %d\r", brightness) < CMUCAM4_CMD_BUFFER_SIZE)
	? _voidCommandWrapper(_cmdBuffer, CMUCAM4_NON_FS_TIMEOUT)
	: CMUCAM4_COMMAND_OVERFLOW;
}

int ICACHE_FLASH_ATTR CMUcam4_esp::cameraContrast(int contrast)
{
	return (snprintf(_cmdBuffer, CMUCAM4_CMD_BUFFER_SIZE,
	"CC %d\r", contrast) < CMUCAM4_CMD_BUFFER_SIZE)
	? _voidCommandWrapper(_cmdBuffer, CMUCAM4_NON_FS_TIMEOUT)
	: CMUCAM4_COMMAND_OVERFLOW;
}

int ICACHE_FLASH_ATTR CMUcam4_esp::cameraRegisterRead(int reg)
{
	return (snprintf(_cmdBuffer, CMUCAM4_CMD_BUFFER_SIZE,
	"CR %d\r", reg) < CMUCAM4_CMD_BUFFER_SIZE)
	? _intCommandWrapper(_cmdBuffer, CMUCAM4_NON_FS_TIMEOUT)
	: CMUCAM4_COMMAND_OVERFLOW;
}

int ICACHE_FLASH_ATTR CMUcam4_esp::cameraRegisterWrite(int reg, int value, int mask)
{
	return (snprintf(_cmdBuffer, CMUCAM4_CMD_BUFFER_SIZE,
	"CW %d %d %d\r", reg, value, mask) < CMUCAM4_CMD_BUFFER_SIZE)
	? _voidCommandWrapper(_cmdBuffer, CMUCAM4_NON_FS_TIMEOUT)
	: CMUCAM4_COMMAND_OVERFLOW;
}

/*******************************************************************************
* Camera Sensor Auto Control Commands
*******************************************************************************/

int ICACHE_FLASH_ATTR CMUcam4_esp::autoGainControl(int active)
{
	return (snprintf(_cmdBuffer, CMUCAM4_CMD_BUFFER_SIZE,
	"AG %d\r", active) < CMUCAM4_CMD_BUFFER_SIZE)
	? _voidCommandWrapper(_cmdBuffer, CMUCAM4_NON_FS_TIMEOUT)
	: CMUCAM4_COMMAND_OVERFLOW;
}

int ICACHE_FLASH_ATTR CMUcam4_esp::autoWhiteBalance(int active)
{
	return (snprintf(_cmdBuffer, CMUCAM4_CMD_BUFFER_SIZE,
	"AW %d\r", active) < CMUCAM4_CMD_BUFFER_SIZE)
	? _voidCommandWrapper(_cmdBuffer, CMUCAM4_NON_FS_TIMEOUT)
	: CMUCAM4_COMMAND_OVERFLOW;
}

/*******************************************************************************
* Camera Format Commands
*******************************************************************************/

int ICACHE_FLASH_ATTR CMUcam4_esp::horizontalMirror(int active)
{
	return (snprintf(_cmdBuffer, CMUCAM4_CMD_BUFFER_SIZE,
	"HM %d\r", active) < CMUCAM4_CMD_BUFFER_SIZE)
	? _voidCommandWrapper(_cmdBuffer, CMUCAM4_NON_FS_TIMEOUT)
	: CMUCAM4_COMMAND_OVERFLOW;
}

int ICACHE_FLASH_ATTR CMUcam4_esp::verticalFlip(int active)
{
	return (snprintf(_cmdBuffer, CMUCAM4_CMD_BUFFER_SIZE,
	"VF %d\r", active) < CMUCAM4_CMD_BUFFER_SIZE)
	? _voidCommandWrapper(_cmdBuffer, CMUCAM4_NON_FS_TIMEOUT)
	: CMUCAM4_COMMAND_OVERFLOW;
}

/*******************************************************************************
* Camera Effect Commands
*******************************************************************************/

int ICACHE_FLASH_ATTR CMUcam4_esp::blackAndWhiteMode(int active)
{
	return (snprintf(_cmdBuffer, CMUCAM4_CMD_BUFFER_SIZE,
	"BW %d\r", active) < CMUCAM4_CMD_BUFFER_SIZE)
	? _voidCommandWrapper(_cmdBuffer, CMUCAM4_NON_FS_TIMEOUT)
	: CMUCAM4_COMMAND_OVERFLOW;
}

int ICACHE_FLASH_ATTR CMUcam4_esp::negativeMode(int active)
{
	return (snprintf(_cmdBuffer, CMUCAM4_CMD_BUFFER_SIZE,
	"NG %d\r", active) < CMUCAM4_CMD_BUFFER_SIZE)
	? _voidCommandWrapper(_cmdBuffer, CMUCAM4_NON_FS_TIMEOUT)
	: CMUCAM4_COMMAND_OVERFLOW;
}

/*******************************************************************************
* Auxiliary I/O Commands
*******************************************************************************/

int ICACHE_FLASH_ATTR CMUcam4_esp::getButtonState()
{
	return _intCommandWrapper("GB\r", CMUCAM4_NON_FS_TIMEOUT);
}

long ICACHE_FLASH_ATTR CMUcam4_esp::getButtonDuration()
{
	int errorValue; int resultValue; long returnValue;

	if(errorValue = _commandWrapper("GD\r", CMUCAM4_NON_FS_TIMEOUT))
	{
		return errorValue;
	}

	_receiveData();

	returnValue = atol(_resBuffer);

	_waitForIdle();
	return resultValue;
}

int ICACHE_FLASH_ATTR CMUcam4_esp::getButtonPressed()
{
	return _intCommandWrapper("GP\r", CMUCAM4_NON_FS_TIMEOUT);
}

int ICACHE_FLASH_ATTR CMUcam4_esp::getButtonReleased()
{
	return _intCommandWrapper("GR\r", CMUCAM4_NON_FS_TIMEOUT);
}

int ICACHE_FLASH_ATTR CMUcam4_esp::panInput()
{
	return _intCommandWrapper("PI\r", CMUCAM4_NON_FS_TIMEOUT);
}

int ICACHE_FLASH_ATTR CMUcam4_esp::panOutput(int direction, int output)
{
	return (snprintf(_cmdBuffer, CMUCAM4_CMD_BUFFER_SIZE,
	"PO %d %d\r", direction, output) < CMUCAM4_CMD_BUFFER_SIZE)
	? _voidCommandWrapper(_cmdBuffer, CMUCAM4_NON_FS_TIMEOUT)
	: CMUCAM4_COMMAND_OVERFLOW;
}

int ICACHE_FLASH_ATTR CMUcam4_esp::tiltInput()
{
	return _intCommandWrapper("TI\r", CMUCAM4_NON_FS_TIMEOUT);
}

int ICACHE_FLASH_ATTR CMUcam4_esp::tiltOutput(int direction, int output)
{
	return (snprintf(_cmdBuffer, CMUCAM4_CMD_BUFFER_SIZE,
	"TO %d %d\r", direction, output) < CMUCAM4_CMD_BUFFER_SIZE)
	? _voidCommandWrapper(_cmdBuffer, CMUCAM4_NON_FS_TIMEOUT)
	: CMUCAM4_COMMAND_OVERFLOW;
}

int ICACHE_FLASH_ATTR CMUcam4_esp::getInputs()
{
	return _intCommandWrapper("GI\r", CMUCAM4_NON_FS_TIMEOUT);
}

int ICACHE_FLASH_ATTR CMUcam4_esp::setOutputs(int directions, int outputs)
{
	return (snprintf(_cmdBuffer, CMUCAM4_CMD_BUFFER_SIZE,
	"SO %d %d\r", directions, outputs) < CMUCAM4_CMD_BUFFER_SIZE)
	? _voidCommandWrapper(_cmdBuffer, CMUCAM4_NON_FS_TIMEOUT)
	: CMUCAM4_COMMAND_OVERFLOW;
}

int ICACHE_FLASH_ATTR CMUcam4_esp::LEDOff()
{
	return _voidCommandWrapper("L0\r", CMUCAM4_NON_FS_TIMEOUT);
}

int ICACHE_FLASH_ATTR CMUcam4_esp::LEDOn(long frequency)
{
	return (snprintf(_cmdBuffer, CMUCAM4_CMD_BUFFER_SIZE,
	"L1 %ld\r", frequency) < CMUCAM4_CMD_BUFFER_SIZE)
	? _voidCommandWrapper(_cmdBuffer, CMUCAM4_NON_FS_TIMEOUT)
	: CMUCAM4_COMMAND_OVERFLOW;
}

/*******************************************************************************
* Servo Commands
*******************************************************************************/

int ICACHE_FLASH_ATTR CMUcam4_esp::getServoPosition(int servo)
{
	return (snprintf(_cmdBuffer, CMUCAM4_CMD_BUFFER_SIZE,
	"GS %d\r", servo) < CMUCAM4_CMD_BUFFER_SIZE)
	? _intCommandWrapper(_cmdBuffer, CMUCAM4_NON_FS_TIMEOUT)
	: CMUCAM4_COMMAND_OVERFLOW;
}

int ICACHE_FLASH_ATTR CMUcam4_esp::setServoPosition(int servo, int active, int pulseLength)
{
	return (snprintf(_cmdBuffer, CMUCAM4_CMD_BUFFER_SIZE,
	"SS %d %d %d\r", servo, active, pulseLength) < CMUCAM4_CMD_BUFFER_SIZE)
	? _voidCommandWrapper(_cmdBuffer, CMUCAM4_NON_FS_TIMEOUT)
	: CMUCAM4_COMMAND_OVERFLOW;
}

int ICACHE_FLASH_ATTR CMUcam4_esp::automaticPan(int active, int reverse)
{
	return (snprintf(_cmdBuffer, CMUCAM4_CMD_BUFFER_SIZE,
	"AP %d %d\r", active, reverse) < CMUCAM4_CMD_BUFFER_SIZE)
	? _voidCommandWrapper(_cmdBuffer, CMUCAM4_NON_FS_TIMEOUT)
	: CMUCAM4_COMMAND_OVERFLOW;
}

int ICACHE_FLASH_ATTR CMUcam4_esp::automaticTilt(int active, int reverse)
{
	return (snprintf(_cmdBuffer, CMUCAM4_CMD_BUFFER_SIZE,
	"AT %d %d\r", active, reverse) < CMUCAM4_CMD_BUFFER_SIZE)
	? _voidCommandWrapper(_cmdBuffer, CMUCAM4_NON_FS_TIMEOUT)
	: CMUCAM4_COMMAND_OVERFLOW;
}

int ICACHE_FLASH_ATTR CMUcam4_esp::autoPanParameters(int proportionalGain, int derivativeGain)
{
	return (snprintf(_cmdBuffer, CMUCAM4_CMD_BUFFER_SIZE,
	"PP %d %d\r", proportionalGain, derivativeGain) < CMUCAM4_CMD_BUFFER_SIZE)
	? _voidCommandWrapper(_cmdBuffer, CMUCAM4_NON_FS_TIMEOUT)
	: CMUCAM4_COMMAND_OVERFLOW;
}

int ICACHE_FLASH_ATTR CMUcam4_esp::autoTiltParameters(int proportionalGain, int derivativeGain)
{
	return (snprintf(_cmdBuffer, CMUCAM4_CMD_BUFFER_SIZE,
	"TP %d %d\r", proportionalGain, derivativeGain) < CMUCAM4_CMD_BUFFER_SIZE)
	? _voidCommandWrapper(_cmdBuffer, CMUCAM4_NON_FS_TIMEOUT)
	: CMUCAM4_COMMAND_OVERFLOW;
}

/*******************************************************************************
* Television Commands
*******************************************************************************/

int ICACHE_FLASH_ATTR CMUcam4_esp::monitorOff()
{
	return _voidCommandWrapper("M0\r", CMUCAM4_NON_FS_TIMEOUT);
}

int ICACHE_FLASH_ATTR CMUcam4_esp::monitorOn()
{
	return _voidCommandWrapper("M1\r", CMUCAM4_NON_FS_TIMEOUT);
}

int ICACHE_FLASH_ATTR CMUcam4_esp::monitorFreeze(int active)
{
	return (snprintf(_cmdBuffer, CMUCAM4_CMD_BUFFER_SIZE,
	"MF %d\r", active) < CMUCAM4_CMD_BUFFER_SIZE)
	? _voidCommandWrapper(_cmdBuffer, CMUCAM4_NON_FS_TIMEOUT)
	: CMUCAM4_COMMAND_OVERFLOW;
}

int ICACHE_FLASH_ATTR CMUcam4_esp::monitorSignal(int active)
{
	return (snprintf(_cmdBuffer, CMUCAM4_CMD_BUFFER_SIZE,
	"MS %d\r", active) < CMUCAM4_CMD_BUFFER_SIZE)
	? _voidCommandWrapper(_cmdBuffer, CMUCAM4_NON_FS_TIMEOUT)
	: CMUCAM4_COMMAND_OVERFLOW;
}

/*******************************************************************************
* Color Tracking Commands
*******************************************************************************/

int ICACHE_FLASH_ATTR CMUcam4_esp::getTrackingParameters(CMUcam4_esp_tracking_parameters_t * pointer)
{
	int errorValue; int resultValue;

	if(pointer == NULL)
	{
		return CMUCAM4_RETURN_FAILURE;
	}

	if(errorValue = _commandWrapper("GT\r", CMUCAM4_NON_FS_TIMEOUT))
	{
		return errorValue;
	}

	_receiveData();


	char * pch;
	STRTOK(_resBuffer, pointer->redMin)
	STRTOK(NULL, pointer->redMax)
	STRTOK(NULL, pointer->greenMin)
	STRTOK(NULL, pointer->greenMax)
	STRTOK(NULL, pointer->blueMin)
	STRTOK(NULL, pointer->blueMax)

	_waitForIdle();
	return resultValue ? CMUCAM4_RETURN_SUCCESS : CMUCAM4_UNEXPECTED_RESPONCE;
}

int ICACHE_FLASH_ATTR CMUcam4_esp::getTrackingWindow(CMUcam4_esp_tracking_window_t * pointer)
{
	int errorValue; int resultValue;

	if(pointer == NULL)
	{
		return CMUCAM4_RETURN_FAILURE;
	}

	if(errorValue = _commandWrapper("GW\r", CMUCAM4_NON_FS_TIMEOUT))
	{
		return errorValue;
	}

	_receiveData();
	char *pch;
	STRTOK(_resBuffer, pointer->topLeftX)
	STRTOK(NULL, pointer->topLeftY)
	STRTOK(NULL, pointer->bottomRightX)
	STRTOK(NULL, pointer->bottomRightY)

	return _waitForIdle();
}

int ICACHE_FLASH_ATTR CMUcam4_esp::setTrackingParameters()
{
	return _voidCommandWrapper("ST\r", CMUCAM4_NON_FS_TIMEOUT);
}

int ICACHE_FLASH_ATTR CMUcam4_esp::setTrackingParameters(int redMin, int redMax,
								   int greenMin, int greenMax,
								   int blueMin, int blueMax)
{
	return (snprintf(_cmdBuffer, CMUCAM4_CMD_BUFFER_SIZE,
	"ST %d %d %d %d %d %d\r",
	redMin, redMax, greenMin, greenMax, blueMin, blueMax)
	< CMUCAM4_CMD_BUFFER_SIZE)
	? _voidCommandWrapper(_cmdBuffer, CMUCAM4_NON_FS_TIMEOUT)
	: CMUCAM4_COMMAND_OVERFLOW;
}

int ICACHE_FLASH_ATTR CMUcam4_esp::setTrackingWindow()
{
	return _voidCommandWrapper("SW\r", CMUCAM4_NON_FS_TIMEOUT);
}

int ICACHE_FLASH_ATTR CMUcam4_esp::setTrackingWindow(int topLeftX, int topLeftY,
							   int bottomRightX, int bottomRightY)
{
	return (snprintf(_cmdBuffer, CMUCAM4_CMD_BUFFER_SIZE,
	"SW %d %d %d %d\r",
	topLeftX, topLeftY, bottomRightX, bottomRightY)
	< CMUCAM4_CMD_BUFFER_SIZE)
	? _voidCommandWrapper(_cmdBuffer, CMUCAM4_NON_FS_TIMEOUT)
	: CMUCAM4_COMMAND_OVERFLOW;
}

int ICACHE_FLASH_ATTR CMUcam4_esp::idleCamera()
{
	int errorValue;
	char cmdBuffer[CMUCAM4_IC_LENGTH];

	if(_state == DEACTIVATED)
	{
		return CMUCAM4_NOT_ACTIVATED;
	}

	if(snprintf(cmdBuffer, CMUCAM4_IC_LENGTH,
		CMUCAM4_IC_STRING, (_version / 100), (_version % 100))
		>= CMUCAM4_IC_LENGTH)
	{
		return CMUCAM4_COMMAND_OVERFLOW;
	}

	_setReadTimeout(CMUCAM4_IDLE_TIMEOUT);
	Serial.write((uint8_t) '\0');
	Serial.write((uint8_t) '\0');
	Serial.write((uint8_t) '\0');
	Serial.write("\rGV\r");
	_waitForString(cmdBuffer);
	return CMUCAM4_RETURN_SUCCESS;
}

int ICACHE_FLASH_ATTR CMUcam4_esp::trackColor()
{
	return _commandWrapper("TC\r", CMUCAM4_NON_FS_TIMEOUT);
}

int ICACHE_FLASH_ATTR CMUcam4_esp::trackColor(int redMin, int redMax,
						int greenMin, int greenMax,
						int blueMin, int blueMax)
{
	return (snprintf(_cmdBuffer, CMUCAM4_CMD_BUFFER_SIZE,
	"TC %d %d %d %d %d %d\r",
	redMin, redMax, greenMin, greenMax, blueMin, blueMax)
	< CMUCAM4_CMD_BUFFER_SIZE)
	? _commandWrapper(_cmdBuffer, CMUCAM4_NON_FS_TIMEOUT)
	: CMUCAM4_COMMAND_OVERFLOW;
}

int ICACHE_FLASH_ATTR CMUcam4_esp::trackWindow(int redRange, int greenRange, int blueRange)
{
	return (snprintf(_cmdBuffer, CMUCAM4_CMD_BUFFER_SIZE,
	"TW %d %d %d\r", redRange, greenRange, blueRange)
	< CMUCAM4_CMD_BUFFER_SIZE)
	? _commandWrapper(_cmdBuffer, CMUCAM4_NON_FS_TIMEOUT)
	: CMUCAM4_COMMAND_OVERFLOW;
}

int ICACHE_FLASH_ATTR CMUcam4_esp::getHistogram(int channel, int bins)
{
	return (snprintf(_cmdBuffer, CMUCAM4_CMD_BUFFER_SIZE,
	"GH %d %d\r", channel, bins)
	< CMUCAM4_CMD_BUFFER_SIZE)
	? _commandWrapper(_cmdBuffer, CMUCAM4_NON_FS_TIMEOUT)
	: CMUCAM4_COMMAND_OVERFLOW;
}

int ICACHE_FLASH_ATTR CMUcam4_esp::getMean()
{
	return _commandWrapper("GM\r", CMUCAM4_NON_FS_TIMEOUT);
}

int ICACHE_FLASH_ATTR CMUcam4_esp::getTypeFDataPacket(CMUcam4_esp_image_data_t * pointer)
{
	int errorValue;

	if(pointer == NULL)
	{
		return CMUCAM4_RETURN_FAILURE;
	}

	if(errorValue = _responceWrapper('F'))
	{
		return errorValue;
	}

	if(strcmp(_resBuffer, "F ") != 0)
	{
		return CMUCAM4_UNEXPECTED_RESPONCE;
	}

	_readBinary(pointer->pixels, CMUCAM4_ID_T_LENGTH, CMUCAM4_ID_T_LENGTH, 0);

	return (_readWithTimeout() == '\r')
	? CMUCAM4_RETURN_SUCCESS : CMUCAM4_UNEXPECTED_RESPONCE;
}

int ICACHE_FLASH_ATTR CMUcam4_esp::getTypeHDataPacket(CMUcam4_esp_histogram_data_1_t * pointer)
{
	int errorValue; char * buffer = (_resBuffer + sizeof('H')); size_t counter;

	if(pointer == NULL)
	{
		return CMUCAM4_RETURN_FAILURE;
	}

	if(errorValue = _responceWrapper('H'))
	{
		return errorValue;
	}

	for(counter = 0; counter < CMUCAM4_HD_1_T_LENGTH; counter++)
	{
		if((*buffer) == '\0')
		{
			return CMUCAM4_UNEXPECTED_RESPONCE;
		}

		pointer->bins[counter] = ((uint8_t) strtol(buffer, &buffer, 10));
	}

	if((*buffer) != '\0')
	{
		return CMUCAM4_UNEXPECTED_RESPONCE;
	}

	return CMUCAM4_RETURN_SUCCESS;
}

int ICACHE_FLASH_ATTR CMUcam4_esp::getTypeHDataPacket(CMUcam4_esp_histogram_data_2_t * pointer)
{
	int errorValue; char * buffer = (_resBuffer + sizeof('H')); size_t counter;

	if(pointer == NULL)
	{
		return CMUCAM4_RETURN_FAILURE;
	}

	if(errorValue = _responceWrapper('H'))
	{
		return errorValue;
	}

	for(counter = 0; counter < CMUCAM4_HD_2_T_LENGTH; counter++)
	{
		if((*buffer) == '\0')
		{
			return CMUCAM4_UNEXPECTED_RESPONCE;
		}

		pointer->bins[counter] = ((uint8_t) strtol(buffer, &buffer, 10));
	}

	if((*buffer) != '\0')
	{
		return CMUCAM4_UNEXPECTED_RESPONCE;
	}

	return CMUCAM4_RETURN_SUCCESS;
}

int ICACHE_FLASH_ATTR CMUcam4_esp::getTypeHDataPacket(CMUcam4_esp_histogram_data_4_t * pointer)
{
	int errorValue; char * buffer = (_resBuffer + sizeof('H')); size_t counter;

	if(pointer == NULL)
	{
		return CMUCAM4_RETURN_FAILURE;
	}

	if(errorValue = _responceWrapper('H'))
	{
		return errorValue;
	}

	for(counter = 0; counter < CMUCAM4_HD_4_T_LENGTH; counter++)
	{
		if((*buffer) == '\0')
		{
			return CMUCAM4_UNEXPECTED_RESPONCE;
		}

		pointer->bins[counter] = ((uint8_t) strtol(buffer, &buffer, 10));
	}

	if((*buffer) != '\0')
	{
		return CMUCAM4_UNEXPECTED_RESPONCE;
	}

	return CMUCAM4_RETURN_SUCCESS;
}

int ICACHE_FLASH_ATTR CMUcam4_esp::getTypeHDataPacket(CMUcam4_esp_histogram_data_8_t * pointer)
{
	int errorValue; char * buffer = (_resBuffer + sizeof('H')); size_t counter;

	if(pointer == NULL)
	{
		return CMUCAM4_RETURN_FAILURE;
	}

	if(errorValue = _responceWrapper('H'))
	{
		return errorValue;
	}

	for(counter = 0; counter < CMUCAM4_HD_8_T_LENGTH; counter++)
	{
		if((*buffer) == '\0')
		{
			return CMUCAM4_UNEXPECTED_RESPONCE;
		}

		pointer->bins[counter] = ((uint8_t) strtol(buffer, &buffer, 10));
	}

	if((*buffer) != '\0')
	{
		return CMUCAM4_UNEXPECTED_RESPONCE;
	}

	return CMUCAM4_RETURN_SUCCESS;
}

int ICACHE_FLASH_ATTR CMUcam4_esp::getTypeHDataPacket(CMUcam4_esp_histogram_data_16_t * pointer)
{
	int errorValue; char * buffer = (_resBuffer + sizeof('H')); size_t counter;

	if(pointer == NULL)
	{
		return CMUCAM4_RETURN_FAILURE;
	}

	if(errorValue = _responceWrapper('H'))
	{
		return errorValue;
	}

	for(counter = 0; counter < CMUCAM4_HD_16_T_LENGTH; counter++)
	{
		if((*buffer) == '\0')
		{
			return CMUCAM4_UNEXPECTED_RESPONCE;
		}

		pointer->bins[counter] = ((uint8_t) strtol(buffer, &buffer, 10));
	}

	if((*buffer) != '\0')
	{
		return CMUCAM4_UNEXPECTED_RESPONCE;
	}

	return CMUCAM4_RETURN_SUCCESS;
}

int ICACHE_FLASH_ATTR CMUcam4_esp::getTypeHDataPacket(CMUcam4_esp_histogram_data_32_t * pointer)
{
	int errorValue; char * buffer = (_resBuffer + sizeof('H')); size_t counter;

	if(pointer == NULL)
	{
		return CMUCAM4_RETURN_FAILURE;
	}

	if(errorValue = _responceWrapper('H'))
	{
		return errorValue;
	}

	for(counter = 0; counter < CMUCAM4_HD_32_T_LENGTH; counter++)
	{
		if((*buffer) == '\0')
		{
			return CMUCAM4_UNEXPECTED_RESPONCE;
		}

		pointer->bins[counter] = ((uint8_t) strtol(buffer, &buffer, 10));
	}

	if((*buffer) != '\0')
	{
		return CMUCAM4_UNEXPECTED_RESPONCE;
	}

	return CMUCAM4_RETURN_SUCCESS;
}

int ICACHE_FLASH_ATTR CMUcam4_esp::getTypeHDataPacket(CMUcam4_esp_histogram_data_64_t * pointer)
{
	int errorValue; char * buffer = (_resBuffer + sizeof('H')); size_t counter;

	if(pointer == NULL)
	{
		return CMUCAM4_RETURN_FAILURE;
	}

	if(errorValue = _responceWrapper('H'))
	{
		return errorValue;
	}

	for(counter = 0; counter < CMUCAM4_HD_64_T_LENGTH; counter++)
	{
		if((*buffer) == '\0')
		{
			return CMUCAM4_UNEXPECTED_RESPONCE;
		}

		pointer->bins[counter] = ((uint8_t) strtol(buffer, &buffer, 10));
	}

	if((*buffer) != '\0')
	{
		return CMUCAM4_UNEXPECTED_RESPONCE;
	}

	return CMUCAM4_RETURN_SUCCESS;
}

int ICACHE_FLASH_ATTR CMUcam4_esp::getTypeSDataPacket(CMUcam4_esp_statistics_data_t * pointer)
{
	int errorValue;

	if(pointer == NULL)
	{
		return CMUCAM4_RETURN_FAILURE;
	}

	if(errorValue = _responceWrapper('S'))
	{
		return errorValue;
	}

	char * pch;
	pch = strtok(_resBuffer, " ");
	if(!(pch && *pch == 'S'))
		return CMUCAM4_RETURN_FAILURE;
	STRTOK(NULL, pointer->RMean)
	STRTOK(NULL, pointer->GMean)
	STRTOK(NULL, pointer->BMean)
	STRTOK(NULL, pointer->RMedian)
	STRTOK(NULL, pointer->GMedian)
	STRTOK(NULL, pointer->BMedian)
	STRTOK(NULL, pointer->RMode)
	STRTOK(NULL, pointer->GMode)
	STRTOK(NULL, pointer->BMode)
	STRTOK(NULL, pointer->RStDev)
	STRTOK(NULL, pointer->GStDev)
	STRTOK(NULL, pointer->BStDev)
	return CMUCAM4_RETURN_SUCCESS;
}

int ICACHE_FLASH_ATTR CMUcam4_esp::getTypeTDataPacket(CMUcam4_esp_tracking_data_t * pointer)
{
	int errorValue;

	if(pointer == NULL)
	{
		return CMUCAM4_RETURN_FAILURE;
	}

	if(errorValue = _responceWrapper('T'))
	{
		return errorValue;
	}

	char * pch;
	pch = strtok(_resBuffer, " ");
	if(!(pch && *pch == 'T'))
		return CMUCAM4_RETURN_FAILURE;
	STRTOK(NULL, pointer->mx)
	STRTOK(NULL, pointer->my)
	STRTOK(NULL, pointer->x1)
	STRTOK(NULL, pointer->y1)
	STRTOK(NULL, pointer->x2)
	STRTOK(NULL, pointer->y2)
	STRTOK(NULL, pointer->pixels)
	STRTOK(NULL, pointer->confidence)

	return CMUCAM4_RETURN_SUCCESS;
}

int ICACHE_FLASH_ATTR CMUcam4_esp::pollMode(int active)
{
	return (snprintf(_cmdBuffer, CMUCAM4_CMD_BUFFER_SIZE,
	"PM %d\r", active) < CMUCAM4_CMD_BUFFER_SIZE)
	? _voidCommandWrapper(_cmdBuffer, CMUCAM4_NON_FS_TIMEOUT)
	: CMUCAM4_COMMAND_OVERFLOW;
}

int ICACHE_FLASH_ATTR CMUcam4_esp::lineMode(int active)
{
	return (snprintf(_cmdBuffer, CMUCAM4_CMD_BUFFER_SIZE,
	"LM %d\r", active) < CMUCAM4_CMD_BUFFER_SIZE)
	? _voidCommandWrapper(_cmdBuffer, CMUCAM4_NON_FS_TIMEOUT)
	: CMUCAM4_COMMAND_OVERFLOW;
}

int ICACHE_FLASH_ATTR CMUcam4_esp::switchingMode(int active)
{
	return (snprintf(_cmdBuffer, CMUCAM4_CMD_BUFFER_SIZE,
	"SM %d\r", active) < CMUCAM4_CMD_BUFFER_SIZE)
	? _voidCommandWrapper(_cmdBuffer, CMUCAM4_NON_FS_TIMEOUT)
	: CMUCAM4_COMMAND_OVERFLOW;
}

int ICACHE_FLASH_ATTR CMUcam4_esp::testMode(int active)
{
	return (snprintf(_cmdBuffer, CMUCAM4_CMD_BUFFER_SIZE,
	"TM %d\r", active) < CMUCAM4_CMD_BUFFER_SIZE)
	? _voidCommandWrapper(_cmdBuffer, CMUCAM4_NON_FS_TIMEOUT)
	: CMUCAM4_COMMAND_OVERFLOW;
}

int ICACHE_FLASH_ATTR CMUcam4_esp::colorTracking(int active)
{
	return (snprintf(_cmdBuffer, CMUCAM4_CMD_BUFFER_SIZE,
	"CT %d\r", active) < CMUCAM4_CMD_BUFFER_SIZE)
	? _voidCommandWrapper(_cmdBuffer, CMUCAM4_NON_FS_TIMEOUT)
	: CMUCAM4_COMMAND_OVERFLOW;
}

int ICACHE_FLASH_ATTR CMUcam4_esp::histogramTracking(int active)
{
	return (snprintf(_cmdBuffer, CMUCAM4_CMD_BUFFER_SIZE,
	"HT %d\r", active) < CMUCAM4_CMD_BUFFER_SIZE)
	? _voidCommandWrapper(_cmdBuffer, CMUCAM4_NON_FS_TIMEOUT)
	: CMUCAM4_COMMAND_OVERFLOW;
}

int ICACHE_FLASH_ATTR CMUcam4_esp::invertedFilter(int active)
{
	return (snprintf(_cmdBuffer, CMUCAM4_CMD_BUFFER_SIZE,
	"IF %d\r", active) < CMUCAM4_CMD_BUFFER_SIZE)
	? _voidCommandWrapper(_cmdBuffer, CMUCAM4_NON_FS_TIMEOUT)
	: CMUCAM4_COMMAND_OVERFLOW;
}

int ICACHE_FLASH_ATTR CMUcam4_esp::noiseFilter(int threshold)
{
	return (snprintf(_cmdBuffer, CMUCAM4_CMD_BUFFER_SIZE,
	"NF %d\r", threshold) < CMUCAM4_CMD_BUFFER_SIZE)
	? _voidCommandWrapper(_cmdBuffer, CMUCAM4_NON_FS_TIMEOUT)
	: CMUCAM4_COMMAND_OVERFLOW;
}

/*******************************************************************************
* File System Commands
*******************************************************************************/

int ICACHE_FLASH_ATTR CMUcam4_esp::changeAttributes(const char * fileOrDirectoryPathName,
							  const char * attributes)
{
	if((fileOrDirectoryPathName == NULL) || (attributes == NULL))
	{
		return CMUCAM4_RETURN_FAILURE;
	}

	return (snprintf(_cmdBuffer, CMUCAM4_CMD_BUFFER_SIZE,
	"CA \"%s\" \"%s\"\r", fileOrDirectoryPathName, attributes)
	< CMUCAM4_CMD_BUFFER_SIZE)
	? _voidCommandWrapper(_cmdBuffer, CMUCAM4_FS_TIMEOUT)
	: CMUCAM4_COMMAND_OVERFLOW;
}

int ICACHE_FLASH_ATTR CMUcam4_esp::changeDirectory(const char * directoryPathAndName)
{
	if(directoryPathAndName == NULL)
	{
		return CMUCAM4_RETURN_FAILURE;
	}

	return (snprintf(_cmdBuffer, CMUCAM4_CMD_BUFFER_SIZE,
	"CD \"%s\"\r", directoryPathAndName)
	< CMUCAM4_CMD_BUFFER_SIZE)
	? _voidCommandWrapper(_cmdBuffer, CMUCAM4_FS_TIMEOUT)
	: CMUCAM4_COMMAND_OVERFLOW;
}

int ICACHE_FLASH_ATTR CMUcam4_esp::diskInformation(CMUcam4_esp_disk_information_t * pointer)
{
	int errorValue;

	if(pointer == NULL)
	{
		return CMUCAM4_RETURN_FAILURE;
	}

	if(errorValue = _commandWrapper("DI\r", CMUCAM4_FS_TIMEOUT))
	{
		return errorValue;
	}

	_receiveData();
	memset(pointer->volumeLabel, '\0', CMUCAM4_VL_LENGTH + 1);
	memset(pointer->fileSystemType, '\0', CMUCAM4_FST_LENGTH + 1);

	strncpy(pointer->volumeLabel, _resBuffer, CMUCAM4_VL_LENGTH);
	pointer->volumeLabel[CMUCAM4_VL_LENGTH] = 0;
	strncpy(pointer->fileSystemType, _resBuffer + CMUCAM4_VL_LENGTH + 1, CMUCAM4_FST_LENGTH);
	pointer->volumeLabel[CMUCAM4_FST_LENGTH] = 0;

	char *pch = strtok(_resBuffer + CMUCAM4_VL_LENGTH + CMUCAM4_FST_LENGTH + 2, " ");
	if(pch)
		pointer->diskSignature = strtol(pch, NULL, 16);
	else
		return CMUCAM4_RETURN_FAILURE;
	pch = strtok(NULL, " ");
	if(pch)
		pointer->volumeIdentification = strtol(pch, NULL, 16);
	else
		return CMUCAM4_RETURN_FAILURE;
	STRTOK(NULL, pointer->countOfDataSectors)
	STRTOK(NULL, pointer->bytesPerSector)
	STRTOK(NULL, pointer->sectorsPerCluster)
	STRTOK(NULL, pointer->countOfClusters)

	return _waitForIdle();
}

int ICACHE_FLASH_ATTR CMUcam4_esp::diskSpace(CMUcam4_esp_disk_space_t * pointer)
{
	int errorValue; int resultValue;

	if(pointer == NULL)
	{
		return CMUCAM4_RETURN_FAILURE;
	}

	if(errorValue = _commandWrapper("DS\r", CMUCAM4_FS_TIMEOUT))
	{
		return errorValue;
	}

	_receiveData();
	char *pch;
	STRTOK(_resBuffer, pointer->freeSectorCount)
	STRTOK(NULL, pointer->usedSectorCount)

	return _waitForIdle();
}

int ICACHE_FLASH_ATTR CMUcam4_esp::formatDisk()
{
	return _voidCommandWrapper("FM\r", CMUCAM4_FS_TIMEOUT);
}

long ICACHE_FLASH_ATTR CMUcam4_esp::listDirectory(CMUcam4_esp_directory_entry_t * pointer,
							size_t size, unsigned long offset)
{
	int errorValue; unsigned long directorySize;

	if(errorValue = _commandWrapper("LS\r", CMUCAM4_FS_TIMEOUT))
	{
		return errorValue;
	}

	for(directorySize = 0; 1; directorySize++)
	{
		_receiveData();

		if((*_resBuffer) == ':')
		{
			break;
		}

		if((pointer != NULL) && (offset <= directorySize) &&
		((directorySize - offset) < ((unsigned long) size)))
		{
			memset(pointer[directorySize - offset].name,
			'\0', CMUCAM4_NAME_LENGTH + 1);
			memset(pointer[directorySize - offset].attributes,
			'\0', CMUCAM4_ATTR_LENGTH + 1);

/*

			if(sscanf(_resBuffer,
			" \"%" CMUCAM4_NAME_LENGTH_STR "c\" "
			"%" CMUCAM4_ATTR_LENGTH_STR "c ",
			pointer[directorySize - offset].name,
			pointer[directorySize - offset].attributes) != 2)
			{
				return CMUCAM4_UNEXPECTED_RESPONCE;
			}
*/
			pointer[directorySize - offset].size = 0;

			if(strchr(pointer[directorySize - offset].attributes, 'D') == NULL)
			{
/*				if(sscanf(_resBuffer,
				" \"%*" CMUCAM4_NAME_LENGTH_STR "c\" "
				"%*" CMUCAM4_ATTR_LENGTH_STR "c "
				"%lu ",
				&(pointer[directorySize - offset].size)) != 1)
				{
					return CMUCAM4_UNEXPECTED_RESPONCE;
				}
*/			}
		}
	}

	return (long) directorySize; // Will be between 0 and 65,536 entries.
}

int ICACHE_FLASH_ATTR CMUcam4_esp::makeDirectory(const char * directoryPathAndName)
{
	if(directoryPathAndName == NULL)
	{
		return CMUCAM4_RETURN_FAILURE;
	}

	return (snprintf(_cmdBuffer, CMUCAM4_CMD_BUFFER_SIZE,
	"MK \"%s\"\r", directoryPathAndName)
	< CMUCAM4_CMD_BUFFER_SIZE)
	? _voidCommandWrapper(_cmdBuffer, CMUCAM4_FS_TIMEOUT)
	: CMUCAM4_COMMAND_OVERFLOW;
}

int ICACHE_FLASH_ATTR CMUcam4_esp::moveEntry(const char * oldEntryPathAndName,
					   const char * newEntryPathAndName)
{
	if((oldEntryPathAndName == NULL) || (newEntryPathAndName == NULL))
	{
		return CMUCAM4_RETURN_FAILURE;
	}

	return (snprintf(_cmdBuffer, CMUCAM4_CMD_BUFFER_SIZE,
	"MV \"%s\" \"%s\"\r", oldEntryPathAndName, newEntryPathAndName)
	< CMUCAM4_CMD_BUFFER_SIZE)
	? _voidCommandWrapper(_cmdBuffer, CMUCAM4_FS_TIMEOUT)
	: CMUCAM4_COMMAND_OVERFLOW;
}

int ICACHE_FLASH_ATTR CMUcam4_esp::printLine(const char * filePathAndName, const char * textToAppend)
{
	if((filePathAndName == NULL) || (textToAppend == NULL))
	{
		return CMUCAM4_RETURN_FAILURE;
	}

	return (snprintf(_cmdBuffer, CMUCAM4_CMD_BUFFER_SIZE,
	"PL \"%s\" \"%s\"\r", filePathAndName, textToAppend)
	< CMUCAM4_CMD_BUFFER_SIZE)
	? _voidCommandWrapper(_cmdBuffer, CMUCAM4_FS_TIMEOUT)
	: CMUCAM4_COMMAND_OVERFLOW;
}

long ICACHE_FLASH_ATTR CMUcam4_esp::filePrint(const char * filePathAndName, uint8_t * buffer,
						size_t size, unsigned long offset)
{
	int errorValue; unsigned long fileSize;

	if(filePathAndName == NULL)
	{
		return CMUCAM4_RETURN_FAILURE;
	}

	if(snprintf(_cmdBuffer, CMUCAM4_CMD_BUFFER_SIZE,
	"PR \"%s\"\r", filePathAndName) >= CMUCAM4_CMD_BUFFER_SIZE)
	{
		return CMUCAM4_COMMAND_OVERFLOW;
	}

	if(errorValue = _commandWrapper(_cmdBuffer, CMUCAM4_FS_TIMEOUT))
	{
		return errorValue;
	}

	_receiveData();

	fileSize = strtol(_resBuffer, NULL, 10);

	_readBinary(buffer, size, fileSize, offset);

	int rs = _waitForIdle();
	return (rs == CMUCAM4_RETURN_SUCCESS) ? fileSize : rs; // Will be between 0 and 2,147,483,647 bytes.
}

int ICACHE_FLASH_ATTR CMUcam4_esp::removeEntry(const char * fileOrDirectoryPathAndName)
{
	if(fileOrDirectoryPathAndName == NULL)
	{
		return CMUCAM4_RETURN_FAILURE;
	}

	return (snprintf(_cmdBuffer, CMUCAM4_CMD_BUFFER_SIZE,
	"RM \"%s\"\r", fileOrDirectoryPathAndName)
	< CMUCAM4_CMD_BUFFER_SIZE)
	? _voidCommandWrapper(_cmdBuffer, CMUCAM4_FS_TIMEOUT)
	: CMUCAM4_COMMAND_OVERFLOW;
}

int ICACHE_FLASH_ATTR CMUcam4_esp::unmountDisk()
{
	return _voidCommandWrapper("UM\r", CMUCAM4_FS_TIMEOUT);
}

/*******************************************************************************
* Image Capture Commands
*******************************************************************************/

int ICACHE_FLASH_ATTR CMUcam4_esp::dumpBitmap()
{
	return _voidCommandWrapper("DB\r", CMUCAM4_FS_TIMEOUT);
}

int ICACHE_FLASH_ATTR CMUcam4_esp::dumpFrame(int horizontalResolution, int verticalResolution)
{
	return (snprintf(_cmdBuffer, CMUCAM4_CMD_BUFFER_SIZE,
	"DF %d %d\r", horizontalResolution, verticalResolution)
	< CMUCAM4_CMD_BUFFER_SIZE)
	? _voidCommandWrapper(_cmdBuffer, CMUCAM4_FS_TIMEOUT)
	: CMUCAM4_COMMAND_OVERFLOW;
}

int ICACHE_FLASH_ATTR CMUcam4_esp::sendBitmap(CMUcam4_esp_image_data_t * pointer)
{
	int errorValue;

	if(pointer == NULL)
	{
		return CMUCAM4_RETURN_FAILURE;
	}

	if(errorValue = _commandWrapper("SB\r", CMUCAM4_NON_FS_TIMEOUT))
	{
		return errorValue;
	}

	_readBinary(pointer->pixels, CMUCAM4_ID_T_LENGTH, CMUCAM4_ID_T_LENGTH, 0);

	if(_readWithTimeout() != '\r')
	{
		return CMUCAM4_UNEXPECTED_RESPONCE;
	}

	return _waitForIdle();
}

int ICACHE_FLASH_ATTR CMUcam4_esp::sendFrame(int horizontalResolution, int verticalResolution,
					   uint16_t * buffer,
					   size_t horizonalSize, size_t horizontalOffset,
					   size_t verticalSize, size_t verticalOffset)
{
	int errorValue; int serialBuffer0; int serialBuffer1;
	size_t indexX; size_t indexY; size_t resolutionX; size_t resolutionY;

	resolutionX = (CMUCAM4_FRAME_H_RES >> horizontalResolution);
	resolutionY = (CMUCAM4_FRAME_V_RES >> verticalResolution);

	if(snprintf(_cmdBuffer, CMUCAM4_CMD_BUFFER_SIZE,
	"SF %d %d\r", horizontalResolution, verticalResolution)
	>= CMUCAM4_CMD_BUFFER_SIZE)
	{
		return CMUCAM4_COMMAND_OVERFLOW;
	}

	if(errorValue = _commandWrapper(_cmdBuffer, CMUCAM4_NON_FS_TIMEOUT))
	{
		return errorValue;
	}

	for(indexX = 0; indexX < resolutionX; indexX++)
	{
		_setReadTimeout(CMUCAM4_NON_FS_TIMEOUT);

		_receiveData();

		if((*_resBuffer) == ':')
		{
			return CMUCAM4_UNEXPECTED_RESPONCE;
		}

		switch(_version)
		{
			case VERSION_100:
			case VERSION_101:

				if(strcmp(_resBuffer, "DAT:") != 0)
				{
					return CMUCAM4_UNEXPECTED_RESPONCE;
				}

				break;

			case VERSION_102:
			case VERSION_103:

				if(strcmp(_resBuffer, "DAT: ") != 0)
				{
					return CMUCAM4_UNEXPECTED_RESPONCE;
				}

				break;
		}

		for(indexY = 0; indexY < resolutionY; indexY++)
		{
			serialBuffer0 = _readWithTimeout();
			serialBuffer1 = _readWithTimeout();

			if((buffer != NULL) && (horizontalOffset <= indexX) &&
			((indexX - horizontalOffset) < horizonalSize) &&
			(verticalOffset <= indexY) &&
			((indexY - verticalOffset) < verticalSize))
			{
				buffer[((indexY - verticalOffset) * horizonalSize)
				+ (indexX - horizontalOffset)]
				= ((uint16_t) (serialBuffer0 | (serialBuffer1 << 8)));
			}
		}

		if(_readWithTimeout() != '\r')
		{
			return CMUCAM4_UNEXPECTED_RESPONCE;
		}
	}

	return _waitForIdle();
}

/*******************************************************************************
* Private Functions
*******************************************************************************/

int ICACHE_FLASH_ATTR CMUcam4_esp::_voidCommandWrapper(const char * command, unsigned long timeout)
{
	int errorValue;

	if(errorValue = _commandWrapper(command, timeout))
	{
		return errorValue;
	}

	return _waitForIdle();
}

int ICACHE_FLASH_ATTR CMUcam4_esp::_intCommandWrapper(const char * command, unsigned long timeout)
{
	int errorValue; int resultValue; int returnValue;

	if(errorValue = _commandWrapper(command, timeout))
	{
		return errorValue;
	}

	_receiveData();
	returnValue = atoi(_resBuffer);

	int rs = _waitForIdle();
	return (rs == CMUCAM4_RETURN_SUCCESS && resultValue) ? returnValue : CMUCAM4_UNEXPECTED_RESPONCE;
}

int ICACHE_FLASH_ATTR CMUcam4_esp::_commandWrapper(const char * command, unsigned long timeout)
{
	int errorValue;

	if(errorValue = idleCamera())
	{
		return errorValue;
	}

	_setReadTimeout(timeout);
	Serial.write(command);
	return _waitForResponce();
}

int ICACHE_FLASH_ATTR CMUcam4_esp::_responceWrapper(char responce)
{
	int errorValue;

	if(_state == DEACTIVATED)
	{
		return CMUCAM4_NOT_ACTIVATED;
	}

	_setReadTimeout(CMUCAM4_NON_FS_TIMEOUT);

	for(;;)
	{
		_receiveData();

		if((*_resBuffer) == responce)
		{
			break;
		}

		if((*_resBuffer) == ':')
		{
			return CMUCAM4_STREAM_END;
		}

		if(strcmp(_resBuffer, "F ") == 0)
		{
			_readBinary(NULL, 0, CMUCAM4_ID_T_LENGTH, 0);

			if(_readWithTimeout() != '\r')
			{
				return CMUCAM4_UNEXPECTED_RESPONCE;
			}
		}
	}

	return CMUCAM4_RETURN_SUCCESS;
}

int ICACHE_FLASH_ATTR CMUcam4_esp::_waitForIdle()
{
	int rs = CMUCAM4_RETURN_SUCCESS;
	for(;;)
	{
		rs = _readText();
		if(rs != CMUCAM4_RETURN_SUCCESS)
			break;

		if(_startsWithString("MSG"))
		{
			continue; // Throw the message away.
		}

		_handleError();

		if((*_resBuffer) != ':')
		{
			rs = CMUCAM4_UNEXPECTED_RESPONCE;
			break;
		}

		break;
	}
	return rs;
}

int ICACHE_FLASH_ATTR CMUcam4_esp::_waitForResponce()
{
	int rs = _readText();

	if(rs == CMUCAM4_RETURN_SUCCESS) {
		if(strcmp(_resBuffer, "NCK") == 0)
		{
			rs = _readText();
			if(rs == CMUCAM4_RETURN_SUCCESS) {

				if((*_resBuffer) == ':')
				{
					rs = CMUCAM4_NCK_RESPONCE;
				} else {
					rs = CMUCAM4_UNEXPECTED_RESPONCE;
				}
			}
		}

		if(rs == CMUCAM4_RETURN_SUCCESS && strcmp(_resBuffer, "ACK") != 0)
		{
			rs = CMUCAM4_UNEXPECTED_RESPONCE;
		}
	}
	return rs;
}

int ICACHE_FLASH_ATTR CMUcam4_esp::_receiveData()
{
	int rs = CMUCAM4_RETURN_SUCCESS;
	for(;;)
	{
		rs = _readText();
		if(rs == CMUCAM4_RETURN_SUCCESS) {

			if(_startsWithString("MSG"))
			{
				continue; // Throw the message away.
			}

			_handleError();
		}
		break;
	}
	return rs;
}

int ICACHE_FLASH_ATTR CMUcam4_esp::_handleError()
{
	int rs = CMUCAM4_RETURN_SUCCESS;

	int errorValue; int sum; size_t index; size_t length;

	if(_startsWithString("ERR"))
	{
		sum = 0; length = strlen(_resBuffer);

		for(index = 0; index < length; index++)
		{
			sum += _resBuffer[index];
		}

		switch(sum)
		{
			case CMUCAM4_CAMERA_TIMEOUT_ERROR_SUM:
				errorValue = CMUCAM4_CAMERA_TIMEOUT_ERROR; break;

			case CMUCAM4_CAMERA_CONNECTION_ERROR_SUM:
				errorValue = CMUCAM4_CAMERA_CONNECTION_ERROR; break;

			case CMUCAM4_DISK_IO_ERROR_SUM:
				errorValue = CMUCAM4_DISK_IO_ERROR; break;

			case CMUCAM4_FILE_SYSTEM_CORRUPTED_SUM:
				errorValue = CMUCAM4_FILE_SYSTEM_CORRUPTED; break;

			case CMUCAM4_FILE_SYSTEM_UNSUPPORTED_SUM:
				errorValue = CMUCAM4_FILE_SYSTEM_UNSUPPORTED; break;

			case CMUCAM4_CARD_NOT_DETECTED_SUM:
				errorValue = CMUCAM4_CARD_NOT_DETECTED; break;

			case CMUCAM4_DISK_MAY_BE_FULL_SUM:
				errorValue = CMUCAM4_DISK_MAY_BE_FULL; break;

			case CMUCAM4_DIRECTORY_FULL_SUM:
				errorValue = CMUCAM4_DIRECTORY_FULL; break;

			case CMUCAM4_EXPECTED_AN_ENTRY_SUM:
				errorValue = CMUCAM4_EXPECTED_AN_ENTRY; break;

			case CMUCAM4_EXPECTED_A_DIRECTORY_SUM:
				errorValue = CMUCAM4_EXPECTED_A_DIRECTORY; break;

			case CMUCAM4_ENTRY_NOT_ACCESSIBLE_SUM:
				errorValue = CMUCAM4_ENTRY_NOT_ACCESSIBLE; break;

			case CMUCAM4_ENTRY_NOT_MODIFIABLE_SUM:
				errorValue = CMUCAM4_ENTRY_NOT_MODIFIABLE; break;

			case CMUCAM4_ENTRY_NOT_FOUND_SUM:
				errorValue = CMUCAM4_ENTRY_NOT_FOUND; break;

			// For v1.02 firmware and above.
			case CMUCAM4_ENTRY_ALREADY_EXISTS_SUM:
				errorValue = CMUCAM4_ENTRY_ALREADY_EXISTS; break;

			// For v1.01 firmware and below.
			case (CMUCAM4_ENTRY_ALREADY_EXISTS_SUM - 's'):
				errorValue = CMUCAM4_ENTRY_ALREADY_EXISTS; break;

			case CMUCAM4_DIRECTORY_LINK_MISSING_SUM:
				errorValue = CMUCAM4_DIRECTORY_LINK_MISSING; break;

			case CMUCAM4_DIRECTORY_NOT_EMPTY_SUM:
				errorValue = CMUCAM4_DIRECTORY_NOT_EMPTY; break;

			case CMUCAM4_NOT_A_DIRECTORY_SUM:
				errorValue = CMUCAM4_NOT_A_DIRECTORY; break;

			case CMUCAM4_NOT_A_FILE_SUM:
				errorValue = CMUCAM4_NOT_A_FILE; break;

			default:
				errorValue = CMUCAM4_UNEXPECTED_RESPONCE; break;
		}

		rs = _readText();

		if(rs == CMUCAM4_RETURN_SUCCESS) {
			if((*_resBuffer) == ':')
			{
				rs = errorValue;
			} else {
				rs = CMUCAM4_UNEXPECTED_RESPONCE;
			}
		}
	}
	return rs;
}

void ICACHE_FLASH_ATTR CMUcam4_esp::_waitForString(const char * string)
{
	size_t index; size_t length = strlen(string);
	memset(_resBuffer, '\0', CMUCAM4_RES_BUFFER_SIZE);

	do
	{
		for(index = 1; index < length; index++)
		{
			_resBuffer[index - 1] = _resBuffer[index];
		}

		_resBuffer[length - 1] = _readWithTimeout();
	}
	while(strcmp(_resBuffer, string) != 0);
}

int ICACHE_FLASH_ATTR CMUcam4_esp::_startsWithString(const char * string)
{
	return (strncmp(_resBuffer, string, strlen(string)) == 0);
}

void ICACHE_FLASH_ATTR CMUcam4_esp::_readBinary(uint8_t * buffer, size_t size,
						  unsigned long packetSize,
						  unsigned long packetOffset)
{
	int serialBuffer; unsigned long serialCounter;

	for(serialCounter = 0; serialCounter < packetSize; serialCounter++)
	{
		serialBuffer = _readWithTimeout();

		if((buffer != NULL) && (packetOffset <= serialCounter) &&
		((serialCounter - packetOffset) < ((unsigned long) size)))
		{
			buffer[serialCounter - packetOffset] = ((uint8_t) serialBuffer);
		}
	}
}

int ICACHE_FLASH_ATTR CMUcam4_esp::_readText()
{
	int rs = CMUCAM4_RETURN_SUCCESS;
	int serialBuffer; size_t serialCounter = 0;
	memset(_resBuffer, '\0', CMUCAM4_RES_BUFFER_SIZE);

	for(;;)
	{
		serialBuffer = _readWithTimeout();

		if(serialBuffer == '\r')
		{
			break;
		}

		_resBuffer[serialCounter++] = serialBuffer;

		if(serialCounter >= CMUCAM4_RES_BUFFER_SIZE)
		{
			rs = CMUCAM4_RESPONCE_OVERFLOW;
		} else {

			switch(serialCounter)
			{
				case sizeof(':'):
    
					if((*_resBuffer) == ':')
					{
						return rs; // Found the idle character.
					}
    
					break;
    
				case (sizeof("F ") - 1):
    
					if(strcmp(_resBuffer, "F ") == 0)
					{
						return rs; // Found type F packet.
					}
    
					break;
    
				case (sizeof("DAT:") - 1):
    
					if(_state == ACTIVATED)
					{
						switch(_version)
						{
							case VERSION_100:
							case VERSION_101:
    
								if(strcmp(_resBuffer, "DAT:") == 0)
								{
									return rs; // Found a old style DAT packet.
								}
    
								break;
    
							case VERSION_102:
							case VERSION_103:
    
								break;
						}
					}
    
					break;
    
				case (sizeof("DAT: ") - 1):
    
					if(_state == ACTIVATED)
					{
						switch(_version)
						{
							case VERSION_100:
							case VERSION_101:
    
								break;
    
							case VERSION_102:
							case VERSION_103:
    
								if(strcmp(_resBuffer, "DAT: ") == 0)
								{
									return rs; // Found a new style DAT packet.
								}
    
								break;
						}
					}
    
					break;
    
				default: break;
			}
		}
	}
	return rs;
}

void ICACHE_FLASH_ATTR CMUcam4_esp::_setReadTimeout(unsigned long timeout)
{
	_timeout = timeout;
	_milliseconds = millis();
}

int ICACHE_FLASH_ATTR CMUcam4_esp::_readWithTimeout()
{
	do
	{
		if((millis() - _milliseconds) >= _timeout)
		{
			return CMUCAM4_SERIAL_TIMEOUT;
		}
		ESP.wdtFeed();
	}
	while(Serial.available() == 0);

	int byte = Serial.read();
	Serial1.print(">");
	Serial1.println(byte);
	return byte;
}

/***************************************************************************//**
* @file
* @par MIT License - TERMS OF USE:
* @n Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"), to
* deal in the Software without restriction, including without limitation the
* rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
* sell copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
* @n
* @n The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
* @n
* @n THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*******************************************************************************/
