#include "Multi_I2C.h"
Multi_MPU::Multi_MPU(int ad0Pin, uint8_t address)
	: MPU6050(address), ad0Pin(ad0Pin){ };

void Multi_MPU::initialize()
{
	setHigh();
	MPU6050::initialize();
	setLow();
}

bool Multi_MPU::testConnection()
{
	setHigh();
	const bool result = MPU6050::testConnection();
	setLow();
	return result;
}


void Multi_MPU::setZAccelOffset(int16_t offset)
{
	setHigh();
	MPU6050::setZAccelOffset(offset);
	setLow();	
}

void Multi_MPU::setXGyroOffset(int16_t offset)
{
	setHigh();
	MPU6050::setXGyroOffset(offset);
	setLow();
}

void Multi_MPU::setYGyroOffset(int16_t offset)
{
	setHigh();
	MPU6050::setYGyroOffset(offset);
	setLow();
}

void Multi_MPU::setZGyroOffset(int16_t offset)
{
	setHigh();
	MPU6050::setZGyroOffset(offset);
	setLow();
}

void Multi_MPU::setDMPEnabled(bool enabled)
{
	setHigh();
	MPU6050::setDMPEnabled(enabled);
	setLow();
}

uint8_t Multi_MPU::getDeviceID()
{
	setHigh();
	const uint8_t result = MPU6050::getDeviceID();
	setLow();
	return result;
}
uint8_t Multi_MPU::getIntStatus()
{
	setHigh();
	const uint8_t result = MPU6050::getIntStatus();
	setLow();
	return result;
}

uint8_t Multi_MPU::dmpGetQuaternion(Quaternion* q, const uint8_t* packet)
{
	setHigh();
	const uint8_t result = MPU6050::dmpGetQuaternion(q, packet);
	setLow();
	return result;
}

uint16_t Multi_MPU::dmpGetFIFOPacketSize()
{
	setHigh();
	const uint16_t result = MPU6050::dmpGetFIFOPacketSize();
	setLow();
	return result;
}

uint8_t Multi_MPU::dmpInitialize()
{
	setHigh();
	const uint8_t result = MPU6050::dmpInitialize();
	setLow();
	return result;
}


uint16_t Multi_MPU::getFIFOCount()
{
	setHigh();
	const uint16_t result = MPU6050::getFIFOCount();
	setLow();
	return result;
}

void Multi_MPU::CalibrateGyro(int Loops)
{
	setHigh();
	MPU6050::CalibrateGyro(Loops);
	setLow();
}

void Multi_MPU::resetFIFO()
{
	setHigh();
	MPU6050::resetFIFO();
	setLow();
}

void Multi_MPU::getFIFOBytes(uint8_t* data, uint8_t length)
{
	setHigh();
	MPU6050::getFIFOBytes(data, length);
	setLow();
}

void Multi_MPU::CalibrateAccel(int Loops)
{
	setHigh();
	MPU6050::CalibrateAccel(Loops);
	setLow();
}

void Multi_MPU::setHigh()
{
	digitalWrite(ad0Pin, HIGH);
}

void Multi_MPU::setLow()
{
	digitalWrite(ad0Pin, LOW);
}

void Multi_MPU::setRate(uint8_t rate)
{
	setHigh();
	MPU6050::setRate(rate);
	setLow();
}

String Multi_MPU::printEverything()
{

	setHigh();
	String msg = label;
	msg.concat("|");
	msg.concat("getSleepEnabled: ");
	msg.concat(MPU6050::getSleepEnabled());
	msg.concat("|getTempSensorEnabled: ");
	msg.concat(MPU6050::getTempSensorEnabled());
	msg.concat("|getClockSource: ");
	msg.concat(MPU6050::getClockSource());
	//Serial.print("getSleepEnabled: ");
	//Serial.println(MPU6050::getSleepEnabled());
	//Serial.print("getTempSensorEnabled: ");
	//Serial.println(MPU6050::getTempSensorEnabled());
	//Serial.print("getClockSource: ");
	//Serial.println(MPU6050::getClockSource());
	//Serial.print("getAccelerationX: ");
	//Serial.println(MPU6050::getAccelerationX());
	//Serial.print("getAccelerationY: ");
	//Serial.println(MPU6050::getAccelerationY());
	//Serial.print("getAccelerationZ: ");
	//Serial.println(MPU6050::getAccelerationZ());
	//Serial.print("getRotationX: ");
	//Serial.println(MPU6050::getRotationX());
	//Serial.print("getRotationY: ");
	//Serial.println(MPU6050::getRotationY());
	//Serial.print("getRotationZ: ");
	//Serial.println(MPU6050::getRotationZ());
	//Serial.print("getDMPEnabled: ");
	//Serial.println(MPU6050::getDMPEnabled());
	//Serial.print("getFIFOEnabled: ");
	//Serial.println(MPU6050::getFIFOEnabled());
	//Serial.print("getI2CMasterModeEnabled: ");
	//Serial.println(MPU6050::getI2CMasterModeEnabled());
	//Serial.print("getTempFIFOEnabled: ");
	//Serial.println(MPU6050::getTempFIFOEnabled());
	//Serial.print("getXGyroFIFOEnabled: ");
	//Serial.println(MPU6050::getXGyroFIFOEnabled());
	//Serial.print("getYGyroFIFOEnabled: ");
	//Serial.println(MPU6050::getYGyroFIFOEnabled());
	//Serial.print("getZGyroFIFOEnabled: ");
	//Serial.println(MPU6050::getZGyroFIFOEnabled());
	//Serial.print("getAccelFIFOEnabled: ");
	//Serial.println(MPU6050::getAccelFIFOEnabled());
	//Serial.print("Sample Rate: ");
	//Serial.println(float(1000.0)/float(1+MPU6050::getRate()));

	msg.concat("|getDMPEnabled: ");
	msg.concat(MPU6050::getDMPEnabled());
	msg.concat("|getFIFOEnabled: ");
	msg.concat(MPU6050::getFIFOEnabled());
	msg.concat("|getI2CMasterModeEnabled: ");
	msg.concat(MPU6050::getI2CMasterModeEnabled());
	msg.concat("|getTempFIFOEnabled: ");
	msg.concat(MPU6050::getTempFIFOEnabled());
	msg.concat("|getXGyroFIFOEnabled: ");
	msg.concat(MPU6050::getXGyroFIFOEnabled());
	msg.concat("|getYGyroFIFOEnabled: ");
	msg.concat(MPU6050::getYGyroFIFOEnabled());
	msg.concat("|getZGyroFIFOEnabled: ");
	msg.concat(MPU6050::getZGyroFIFOEnabled());
	msg.concat("|getAccelFIFOEnabled: ");
	msg.concat(MPU6050::getAccelFIFOEnabled());
	msg.concat("|Sample Rate: ");
	msg.concat(float(1000.0) / float(1 + MPU6050::getRate()));
	msg.concat("\n\r");
	//Serial.print(msg);

	setLow();
	return msg;
}
