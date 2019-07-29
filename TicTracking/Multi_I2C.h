#ifndef Multi_I2C_h
#define Multi_I2C_h
// #define MPU6050_ADDRESS_AD0_LOW     0x68 // address pin low (GND), default for InvenSense evaluation board
// #define MPU6050_ADDRESS_AD0_HIGH    0x69 // address pin high (VCC)
// #define MPU6050_DEFAULT_ADDRESS     MPU6050_ADDRESS_AD0_HIGH
#include <MPU6050_6Axis_MotionApps20.h>
#include <Arduino.h>
class Multi_MPU : public MPU6050
{
public:
	Multi_MPU(int ad0Pin, uint8_t address = MPU6050_DEFAULT_ADDRESS);
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
	//
	// void PrintActiveOffsets(); // See the results of the Calibration
	String printEverything();
	void setLow();
	void setHigh();
	void setDMPEnabled(bool enabled);
	void setRate(uint8_t rate);
	char* label;


	
private:
	int ad0Pin;
};

#endif
