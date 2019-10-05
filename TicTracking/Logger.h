#pragma once

class Logger
{
public:
	Logger(int ad0Pin, uint8_t address = MPU6050_DEFAULT_ADDRESS);
	bool testConnection();
	void initialize();
	uint8_t getDeviceID();
	uint8_t dmpInitialize();
	void setXGyroOffset(int16_t offset);
	void setZAccelOffset(int16_t offset);
	void setYGyroOffset(int16_t offset);
	void setZGyroOffset(int16_t offset);
	void getFIFOBytes(uint8_t* data, uint8_t length);
	void resetFIFO();
	uint8_t getIntStatus();
	uint16_t getFIFOCount();
	uint8_t dmpGetQuaternion(Quaternion *q, const uint8_t* packet = nullptr);
	uint16_t dmpGetFIFOPacketSize();
	void CalibrateGyro(int Loops = 6); // Fine tune after setting offsets with less Loops.
	void CalibrateAccel(int Loops = 6); // Fine tune after setting offsets with less Loops
	void PrintActiveOffsets();
	void setDMPEnabled(bool enabled);
	void setRate(uint8_t rate);

	uint8_t getIntPinBypassReg();
	void readMemoryBlock(uint8_t *data, uint16_t dataSize, uint8_t bank = 0, uint8_t address = 0); //read memory
	String getStatusString();
	void setLow();
	void setHigh();

	char* label; // a short descriptive label like "Right"
	char* prefix; //single character label like "R"
	String init_status_string; //a String of getStatusString at startup.  for integrity checking
	
};
