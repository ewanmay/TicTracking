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

void Multi_MPU::PrintActiveOffsets()
{
	setHigh();
	MPU6050::PrintActiveOffsets();
	setLow();
}
//
// Get interrupt config: latching, levels etc
//
uint8_t Multi_MPU::getIntPinBypassReg() {
	uint8_t buffer[14];
	setHigh();
	I2Cdev::readByte(getDeviceID(), MPU6050_RA_INT_PIN_CFG, buffer);
	setLow();
	return buffer[0];
}

void Multi_MPU::readMemoryBlock(uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address)
{
	setHigh();
	MPU6050::readMemoryBlock(data, dataSize, bank, address);
	setLow();
}; //read memory

String Multi_MPU::getStatusString()
{
	setHigh();

	String msg = label;
	//msg.concat("|");
	msg.concat("getSleepEnabled: ");
	msg.concat(MPU6050::getSleepEnabled());
	msg.concat("|getTempSensorEnabled: ");
	msg.concat(MPU6050::getTempSensorEnabled());
	msg.concat("|getClockSource: ");
	msg.concat(MPU6050::getClockSource());
	msg.concat("|getDMPEnabled: ");
	msg.concat(getDMPEnabled());
	msg.concat("|getFIFOEnabled: ");
	msg.concat(getFIFOEnabled());
	msg.concat("|getIntPinBypassReg: (hex)");
	msg.concat(String(getIntPinBypassReg(), HEX));
	msg.concat("|getInterruptLatchClear: ");
	msg.concat(getInterruptLatchClear());
	msg.concat("|getI2CMasterModeEnabled: ");
	msg.concat(getI2CMasterModeEnabled());
	msg.concat("|getTempFIFOEnabled: ");
	msg.concat(getTempFIFOEnabled());
	msg.concat("|getXGyroFIFOEnabled: ");
	msg.concat(getXGyroFIFOEnabled());
	msg.concat("|getYGyroFIFOEnabled: ");
	msg.concat(getYGyroFIFOEnabled());
	msg.concat("|getZGyroFIFOEnabled: ");
	msg.concat(getZGyroFIFOEnabled());
	msg.concat("|getAccelFIFOEnabled: ");
	msg.concat(getAccelFIFOEnabled());
	msg.concat("|dmpGetFIFOPacketSize: ");
	msg.concat(dmpGetFIFOPacketSize());
	//msg.concat("|Sample Rate Divider: ");
	//msg.concat( getRate());
	msg.concat("|getIntEnabled: (hex)");
	msg.concat(String(getIntEnabled(), HEX));
	getIntEnabled();

	msg.concat("|Sample Rate: ");
	msg.concat(float(1000.0) / float(1 + getRate()));

	setLow();
	return msg;
}
